"""Entry point for the bundle calibration program.

Wires together the serial link, the calibration session runner, and
(once all 7 poses have been recorded) the bundle calibration algorithm.
See calibration/ for the actual implementation:
  - protocol.py: wire format parsing
  - serial_link.py: the serial port itself
  - session.py: the PC-side state machine (pure, no I/O)
  - collector.py: runs a session end to end (serial thread + main loop)
  - tui.py: draws a session as a Rich Live display (no I/O of its own)
  - report.py: turns a fit result into the Rich content tui.py and this show
  - nominal_geometry.py: pure nominal/CAD constants
  - parameterization.py: what the calibration parameter vector means
  - bundle_geometry.py: the physical forward model + analytic Jacobian
  - bundle_params.py / calibration_algorithm.py: the fit itself
  - export.py: fitted geometry -> firmware constants
"""

from __future__ import annotations

import argparse
import contextlib
import json
import time
from pathlib import Path

from rich.console import Console
from rich.prompt import IntPrompt

from calibration import collector
from calibration.calibration_algorithm import (
    DEFAULT_N_FRAMES,
    BundleCalibrationResult,
    run_bundle_calibration,
)
from calibration.export import format_binary, format_cpp
from calibration.protocol import CalibStep
from calibration.report import build_fit_report
from calibration.serial_link import SerialLink, list_available_ports

DEFAULT_BAUDRATE = 921600

# Where --emit-bin writes by default: PlatformIO's data_dir, so the blob is
# already in place for `pio run -t uploadfs`.
DEFAULT_BIN_PATH = Path(__file__).parent.parent / "firmware" / "data" / "calibration.bin"

# The upload is only accepted during a tare, which the user triggers by hand,
# so this waits about as long as someone needs to pick the knob up and hold
# both buttons.
TARE_WAIT_SECONDS = 120.0
UPLOAD_RESPONSE_SECONDS = 5.0

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


def _write_calibration_over_serial(
    console: Console, port: str, baud: int, blob: bytes
) -> None:
    """Push a fitted calibration to a running device and let it reboot into it.

    The firmware only accepts CAL_UPLOAD during a tare, which is a deliberate
    choice rather than an obstacle: it means nothing can rewrite a device's
    calibration without someone physically holding the buttons. The tare window
    is short (~1.7s), so rather than guess at the timing we wait for the
    device to announce STATUS TARE_BEGIN and answer immediately.
    """
    link = SerialLink(port, baud)
    try:
        link.open()
    except Exception as e:
        console.print(f"[red]Failed to open {port}: {e}[/]")
        raise SystemExit(1) from e

    try:
        console.print(
            "\n[bold]Waiting for a tare window.[/] Hold both buttons on the knob for "
            "~3 seconds - the device announces STATUS TARE_BEGIN when it is ready."
        )

        deadline = time.monotonic() + TARE_WAIT_SECONDS
        while time.monotonic() < deadline:
            line = link.readline()
            if line is not None and line.strip() == "STATUS TARE_BEGIN":
                break
        else:
            console.print("[red]Timed out waiting for STATUS TARE_BEGIN.[/]")
            raise SystemExit(1)

        link.send(f"CAL_UPLOAD {blob.hex()}")

        deadline = time.monotonic() + UPLOAD_RESPONSE_SECONDS
        while time.monotonic() < deadline:
            try:
                line = link.readline()
            except Exception:
                # The device reboots immediately after acknowledging, so the
                # port vanishing mid-read is the expected ending, not a fault.
                break
            if line is None:
                continue
            line = line.strip()
            if line == "CAL_UPLOAD_OK":
                console.print(
                    "[green]Calibration stored.[/] The device is rebooting to apply it - "
                    "the serial port will drop and come back."
                )
                return
            if line.startswith("CAL_UPLOAD_ERR"):
                console.print(f"[red]Device rejected the calibration:[/] {line}")
                raise SystemExit(1)

        console.print("[yellow]No response from the device.[/]")
        raise SystemExit(1)
    finally:
        # The device reboots out from under us on success, so closing the port
        # can fail on a handle that is already gone. Nothing to do about it.
        with contextlib.suppress(Exception):
            link.close()


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
    parser.add_argument(
        "--emit-bin",
        type=Path,
        nargs="?",
        const=DEFAULT_BIN_PATH,
        default=None,
        metavar="PATH",
        help=f"Also write the fitted constants as a binary calibration blob "
             f"(default {DEFAULT_BIN_PATH}), ready for `pio run -t uploadfs`.",
    )
    parser.add_argument(
        "--write-serial",
        action="store_true",
        help="Keep the knob in calibration mode after capture, solve, then ask "
             "whether to write the result to it. Without this the knob is "
             "released as soon as capture finishes, since nothing is coming "
             "back to it. With --replay there is no capture, so this instead "
             "waits for you to put the knob into tare and pushes the re-fitted "
             "result.",
    )
    args = parser.parse_args()

    console = Console()

    def report(result: BundleCalibrationResult) -> None:
        console.print(build_fit_report(result).full())
        if args.emit_cpp:
            console.print("\n[bold]Firmware constants[/]")
            console.print(format_cpp(result.geometry))

        if args.emit_bin is not None:
            blob = format_binary(result.geometry)
            args.emit_bin.parent.mkdir(parents=True, exist_ok=True)
            args.emit_bin.write_bytes(blob)
            console.print(f"\nWrote {len(blob)} byte calibration blob to {args.emit_bin}")
            console.print("[grey50]Flash it with: pio run -t uploadfs[/]")


    if args.replay:
        # No capture, so no knob to hold: fit the dump, then push it the
        # standalone way if asked. This is how you re-fit an old run and
        # deliver it without recapturing.
        result = run_bundle_calibration(_load_raw_datasets(args.replay), n_frames=args.frames)
        report(result)
        if args.write_serial:
            _write_calibration_over_serial(
                console,
                args.port or _pick_port(console),
                args.baud,
                format_binary(result.geometry),
            )
        return

    port = args.port or _pick_port(console)

    link = SerialLink(port, args.baud)
    try:
        link.open()
    except Exception as e:
        console.print(f"[red]Failed to open {port}: {e}[/]")
        raise SystemExit(1) from e

    def solve(datasets: dict[CalibStep, list[list[float]]]) -> BundleCalibrationResult:
        return run_bundle_calibration(datasets, n_frames=args.frames)

    try:
        # Blocks for the whole run: waiting for the knob to enter tare,
        # collecting/ACKing all 7 poses, solving, and (with --write-serial)
        # asking whether to write the result back - all inside one display.
        # The same FitReport that the live display splits across its panels
        # is what `report()` prints in full below: one build_fit_report call
        # feeds both, not a paraphrase for the display and the real thing
        # for afterwards.
        outcome = collector.run_calibration_session(
            link,
            solve=solve,
            summarize=build_fit_report,
            make_blob=(
                (lambda result: format_binary(result.geometry))
                if args.write_serial
                else None
            ),
        )
    finally:
        link.close()

    session = outcome.session
    dump_path = _dump_raw_datasets(session.datasets)
    console.print(f"Raw calibration data saved to {dump_path}")

    if not session.completed:
        console.print("[yellow]Calibration aborted before all steps completed.[/]")
        raise SystemExit(1)

    if outcome.uploaded:
        console.print("[green]Calibration written to the knob.[/] It is rebooting to apply it.")

    # The fit already ran inside the session; this is the detailed report, plus
    # whatever --emit-cpp / --emit-bin were asked for.
    if outcome.result is not None:
        report(outcome.result)


if __name__ == "__main__":
    main()
