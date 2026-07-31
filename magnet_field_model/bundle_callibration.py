"""Entry point for the bundle calibration program.

Wires together the serial link, the calibration session runner, and
(once all 7 poses have been recorded) the bundle calibration algorithm.
See calibration/ for the actual implementation:
  - protocol.py: wire format parsing
  - serial_link.py: the serial port itself
  - session.py: the PC-side state machine (pure, no I/O)
  - collector.py: runs a session end to end (serial thread + main loop)
  - tui.py: draws a session as a Rich Live display (no I/O of its own)
  - bundle_geometry.py / bundle_params.py / calibration_algorithm.py:
    the bundle calibration math itself
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
from calibration.bundle_geometry import N_MAGNETS
from calibration.bundle_params import N_SHARED_PARAMS_PER_PAIR
from calibration.calibration_algorithm import BundleCalibrationResult, run_bundle_calibration
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
    status = "[green]converged[/]" if result.success else "[red]did NOT converge[/]"
    console.print(f"\nBundle calibration {status} ({result.message}); final cost: {result.cost:.4f}")

    table = Table(title="Fitted calibration offsets (from nominal)")
    table.add_column("Pair")
    table.add_column("Sensor pos offset (mm)")
    table.add_column("Magnet pos offset (mm)")
    table.add_column("Magnet rot offset (deg)")
    table.add_column("Gain")

    for i in range(N_MAGNETS):
        pair = result.shared_offsets[i * N_SHARED_PARAMS_PER_PAIR : (i + 1) * N_SHARED_PARAMS_PER_PAIR]
        sensor_pos_offset, magnet_pos_offset, magnet_rotvec_offset = pair[0:3], pair[3:6], pair[6:9]
        gain = 1.0 + pair[9]
        rot_deg = np.degrees(np.linalg.norm(magnet_rotvec_offset))
        table.add_row(
            str(i + 1),
            np.array2string(sensor_pos_offset, precision=3),
            np.array2string(magnet_pos_offset, precision=3),
            f"{rot_deg:.2f}",
            f"{gain:.4f}",
        )

    console.print(table)


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
    args = parser.parse_args()

    console = Console()

    if args.replay:
        result = run_bundle_calibration(_load_raw_datasets(args.replay))
        _print_calibration_summary(console, result)
        return

    port = args.port or _pick_port(console)

    link = SerialLink(port, args.baud)
    try:
        link.open()
    except Exception as e:
        console.print(f"[red]Failed to open {port}: {e}[/]")
        raise SystemExit(1)

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

    result = run_bundle_calibration(session.datasets)
    _print_calibration_summary(console, result)
    # TODO: persist/export the *computed calibration result* - output
    # format and storage location for that are not decided yet. The raw
    # dump above is just so hardware test runs aren't lost in the meantime.


if __name__ == "__main__":
    main()
