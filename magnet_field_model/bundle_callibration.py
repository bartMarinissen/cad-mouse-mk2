"""Entry point for the bundle calibration program.

Wires together the serial link, the calibration session runner, and
(once all 7 poses have been recorded) the bundle calibration algorithm.
See calibration/ for the actual implementation:
  - protocol.py: wire format parsing
  - serial_link.py: the serial port itself
  - session.py: the PC-side state machine (pure, no I/O)
  - collector.py: runs a session end to end (serial thread + main loop)
  - tui.py: draws a session as a Rich Live display (no I/O of its own)
  - nominal_geometry.py: pure nominal/CAD constants
  - parameterization.py: what the calibration parameter vector means
  - bundle_geometry.py: the physical forward model + analytic Jacobian
  - bundle_params.py / calibration_algorithm.py: the fit itself
  - export.py: fitted geometry -> firmware constants
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.prompt import IntPrompt
from rich.table import Table

from calibration import collector
from calibration.bundle_geometry import MAGNET_POS_NOMINAL_KNOB, N_MAGNETS, N_SENSORS
from calibration.calibration_algorithm import (
    DEFAULT_N_FRAMES,
    BundleCalibrationResult,
    run_bundle_calibration,
)
from calibration.export import format_cpp
from calibration.parameterization import BLOCKS, GROUP_SLICES
from calibration.protocol import CalibStep
from calibration.serial_link import SerialLink, list_available_ports

DEFAULT_BAUDRATE = 921600

# Where the real calibration result gets output/stored is still undecided
# (see the TODO in main()). This is just a raw dump of the collected
# frames, independent of whether the fit below converges cleanly.
RAW_DUMP_DIR = Path(__file__).parent / "calibration_runs"


def _dump_raw_datasets(datasets: dict[CalibStep, list[list[float]]]) -> Path:
    RAW_DUMP_DIR.mkdir(exist_ok=True)
    path = RAW_DUMP_DIR / f"raw_{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps({step.name: frames for step, frames in datasets.items()}, indent=2))
    return path


def _load_raw_datasets(path: Path) -> dict[CalibStep, list[list[float]]]:
    """Load a dump written by _dump_raw_datasets, for re-running the fit against old data."""
    raw = json.loads(path.read_text())
    return {CalibStep[name]: frames for name, frames in raw.items()}


def _print_calibration_summary(console: Console, result: BundleCalibrationResult) -> None:
    ok = all(s.success for s in result.stages)
    status = "[green]converged[/]" if ok else "[yellow]did NOT fully converge[/]"
    console.print(
        f"\nBundle calibration {status}. Field residual "
        f"{result.initial_rms_mT:.3f} mT ({result.initial_rel_pct:.2f}%) at nominal "
        f"-> [bold]{result.field_rms_mT:.3f} mT ({result.field_rel_pct:.2f}%)[/] fitted, "
        f"over {len(result.selected_indices)} frames."
    )

    # Solving is a single joint fit by default (see calibration_algorithm.py's
    # module docstring) - a "stages" breakdown is only interesting when the
    # caller explicitly passed more than one, e.g. to isolate a parameter
    # subset for debugging.
    if len(result.stages) > 1:
        stages = Table(title="Solve stages")
        stages.add_column("Stage")
        stages.add_column("Free params", justify="right")
        stages.add_column("Residual (mT)", justify="right")
        stages.add_column("Residual (%)", justify="right")
        stages.add_column("Time (s)", justify="right")
        for s in result.stages:
            stages.add_row(s.name, str(s.n_free_shared), f"{s.field_rms_mT:.3f}",
                           f"{s.field_rel_pct:.2f}", f"{s.seconds:.1f}")
        console.print(stages)

    geometry = result.geometry

    # Magnets and sensors get separate tables on purpose: they are indexed by
    # different things. Sensor i is not exclusively paired with magnet i -
    # every sensor sees all three magnets, which is the cross-talk the model
    # accounts for - so listing them side by side would imply a pairing that
    # isn't there.
    magnets = Table(title="Fitted magnet geometry (offsets from nominal)")
    magnets.add_column("Magnet")
    magnets.add_column("dx (mm)", justify="right")
    magnets.add_column("dy (mm)", justify="right")
    magnets.add_column("dz (mm)", justify="right")
    magnets.add_column("tilt x (deg)", justify="right")
    magnets.add_column("tilt y (deg)", justify="right")
    magnets.add_column("strength", justify="right")
    for i in range(N_MAGNETS):
        d = geometry.magnet_pos_knob[i] - MAGNET_POS_NOMINAL_KNOB[i]
        tilt = np.degrees(geometry.magnet_tilt[i][:2])
        magnets.add_row(str(i + 1), f"{d[0]:+.3f}", f"{d[1]:+.3f}", f"{d[2]:+.3f}",
                        f"{tilt[0]:+.3f}", f"{tilt[1]:+.3f}",
                        f"{geometry.magnet_strength[i]:.4f}")
    console.print(magnets)

    # Absolute field scale deserves its error bar right next to it: with no
    # prior on the common mode, the fitted value is whatever the data prefers,
    # and the data has very little to say (a uniform scale change is largely
    # absorbable by every frame's pose moving further away).
    mean_sd = result.posterior_sigma[GROUP_SLICES["magnet_strength_mean"]][0]
    mean_strength = result.geometry.magnet_strength.mean()
    if np.isfinite(mean_sd):
        sigmas_from_nominal = abs(mean_strength - 1.0) / mean_sd if mean_sd > 0 else 0.0
        verdict = ("[green]consistent with nominal[/]" if sigmas_from_nominal < 2
                   else "[yellow]notably above nominal[/]")
        console.print(
            f"Common-mode strength [bold]{mean_strength:.3f} ± {mean_sd:.3f}[/] "
            f"({sigmas_from_nominal:.1f}σ from nominal) — {verdict}"
        )

    sensors = Table(title="Fitted sensor gain and DC offset (model side; nominal gain is +I)")
    sensors.add_column("Sensor")
    sensors.add_column("Gain matrix", justify="left")
    sensors.add_column("det", justify="right")
    sensors.add_column("DC offset (mT)", justify="right")
    for i in range(N_SENSORS):
        g = geometry.gain[i]
        rows = "\n".join("  ".join(f"{v:+.4f}" for v in row) for row in g)
        off = "\n".join(f"{v:+.3f}" for v in geometry.sensor_offset[i])
        sensors.add_row(str(i + 1), rows, f"{np.linalg.det(g):.4f}", off)
    console.print(sensors)

    # How much each group was actually pinned down by the data, as opposed to
    # left sitting at its prior. Low information gain is not a bug - some of
    # these directions are only weakly identifiable from any dataset (magnet
    # z-offset trades against gain scale, for instance) - but it is the
    # difference between "measured" and "assumed", so it gets shown.
    info = Table(title="What the data actually determined")
    info.add_column("Parameter group")
    info.add_column("Prior sd", justify="right")
    info.add_column("Posterior sd", justify="right")
    info.add_column("Information gain", justify="right")
    for block in BLOCKS:
        name = block.name
        sl = GROUP_SLICES[name]
        prior = result.prior_sigma[sl].mean()
        post = result.posterior_sigma[sl]
        gain = result.information_gain[sl]
        if not np.all(np.isfinite(post)):
            info.add_row(name, f"{prior:.3f}", "-", "[grey50]not fitted[/]")
            continue
        if not np.isfinite(prior):
            # No prior at all: the posterior sd stands on its own, and asking
            # how much the data added over the prior is meaningless.
            info.add_row(name, "[cyan]none[/]", f"{post.mean():.3f}",
                         "[cyan]data only[/]")
            continue
        colour = "green" if gain.mean() > 0.4 else ("yellow" if gain.mean() > 0.15 else "red")
        info.add_row(name, f"{prior:.3f}", f"{post.mean():.3f}",
                     f"[{colour}]{gain.mean():.0%}[/]")
    console.print(info)
    console.print("[grey50]Information gain = 1 - posterior/prior sd. Low means the fit "
                  "mostly kept the prior, so treat that group as assumed, not measured. "
                  "'data only' means the group carries no prior - read its posterior sd "
                  "as the real uncertainty.[/]")


def _pick_port(console: Console) -> str:
    ports = list_available_ports()
    if not ports:
        console.print("[red]No serial ports found.[/] Plug in the device and try again, "
                       "or pass --port explicitly.")
        raise SystemExit(1)

    console.print("Available serial ports:")
    for i, port in enumerate(ports):
        console.print(f"  [{i}] {port.device} - {port.description}")

    choice = IntPrompt.ask("Select a port", choices=[str(i) for i in range(len(ports))], default=0)
    return ports[choice].device


def main() -> None:
    parser = argparse.ArgumentParser(description="Bundle calibration data collector")
    parser.add_argument("--port", help="Serial port (e.g. COM4). Prompts interactively if omitted.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="Baud rate")
    parser.add_argument(
        "--replay",
        type=Path,
        metavar="PATH",
        help="Skip live data collection and re-run the bundle fit against a previously "
             "saved raw_*.json dump (see calibration_runs/) instead.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=DEFAULT_N_FRAMES,
        metavar="N",
        help=f"Cap on how many converged frames the fit uses, evenly decimated "
             f"if more were captured (default {DEFAULT_N_FRAMES}). More than "
             f"~60 measurably buys nothing - the limit is model error, not "
             f"sample noise - but solving all ~387 frames unstaged does cost "
             f"real time (40+s vs ~4s), which is the actual reason for the cap.",
    )
    parser.add_argument(
        "--emit-cpp",
        action="store_true",
        help="Also print a pasteable C++ snippet of the fitted constants.",
    )
    args = parser.parse_args()

    console = Console()

    def report(result: BundleCalibrationResult) -> None:
        _print_calibration_summary(console, result)
        if args.emit_cpp:
            console.print("\n[bold]Firmware constants[/]")
            console.print(format_cpp(result.geometry))

    if args.replay:
        report(run_bundle_calibration(_load_raw_datasets(args.replay), n_frames=args.frames))
        return

    port = args.port or _pick_port(console)

    link = SerialLink(port, args.baud)
    try:
        link.open()
    except Exception as e:
        console.print(f"[red]Failed to open {port}: {e}[/]")
        raise SystemExit(1) from e

    try:
        # Blocks for the whole calibration session - sending CAL_START,
        # collecting/ACKing all 7 poses, and driving the Live display -
        # until STATUS ALL_STEPS_COMPLETE or the user aborts (Ctrl+C).
        session = collector.run_calibration_session(link)
    finally:
        link.close()

    dump_path = _dump_raw_datasets(session.datasets)
    console.print(f"Raw calibration data saved to {dump_path}")

    if not session.completed:
        console.print("[yellow]Calibration aborted before all steps completed.[/]")
        raise SystemExit(1)

    report(run_bundle_calibration(session.datasets, n_frames=args.frames))
    # NOTE: there is still no persistence path into the firmware - this
    # firmware has no flash/EEPROM storage of any kind yet - so --emit-cpp
    # printing a pasteable snippet is as far as the result can travel today.


if __name__ == "__main__":
    main()
