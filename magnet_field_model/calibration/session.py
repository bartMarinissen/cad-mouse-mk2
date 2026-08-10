"""State machine for the PC side of the bundle calibration protocol.

This is a direct port of the logic that used to live inline in the
`capture_calibration_data` loop, restructured so it has no I/O of its own:
callers feed it raw lines via feed_line() and it decides what (if
anything) to send back via the injected send_fn. That makes it testable
with plain strings, no serial port or thread required.
"""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Callable
from typing import TYPE_CHECKING, Literal

from .protocol import (
    STEP_NAMES,
    CalFrame,
    CalibPhase,
    CalibStep,
    CalState,
    Status,
    Unknown,
    parse_line,
)

if TYPE_CHECKING:
    # Only for the type hint on `summary` below - session.py stays a pure
    # protocol state machine at runtime, not pulling in the whole
    # scipy/numpy fit-report stack just to store a reference to its result.
    from .report import FitReport

StepStatus = Literal["done", "active", "pending"]

# Where the run is, beyond what the knob itself reports. The knob only knows
# about capture; solving and the write decision happen here, and the display
# needs to show them too rather than tearing itself down halfway through.
Stage = Literal["waiting_for_tare", "capturing", "solving", "confirming", "finished"]


class CalibrationSession:
    LOG_MAXLEN: int = 200

    _send: Callable[[str], None]
    datasets: dict[CalibStep, list[list[float]]]
    current_step: CalibStep
    current_phase: CalibPhase
    phase_started_at: float
    completed: bool
    stage: Stage
    # The fit report, set once solving finishes. Pre-split (see FitReport)
    # rather than one blob so the live display can put the fitted parameters,
    # the trust diagnostics, and the write prompt in three different panels
    # instead of one wall of text.
    summary: FitReport | None
    log: deque[str]

    def __init__(self, send_fn: Callable[[str], None]):
        self._send = send_fn
        self.datasets = {step: [] for step in CalibStep if int(step) >= 0}
        self.current_step = CalibStep.NONE
        self.current_phase = CalibPhase.IDLE
        self.phase_started_at = time.monotonic()
        self.completed = False
        self.stage = "waiting_for_tare"
        self.summary = None
        self.log = deque(maxlen=self.LOG_MAXLEN)

    def start(self) -> None:
        """Begin waiting for a tare window; CAL_START goes out when one opens.

        Deliberately does not send anything. The knob only accepts CAL_START
        during its ~1.7s tare window, so firing it here meant the command
        landed nowhere whenever the script was started before the knob was put
        into tare - which is the normal order to do things in.
        """
        self.stage = "waiting_for_tare"
        self.log.append("Waiting for the knob. Hold both buttons for ~3 seconds.")

    def note(self, message: str) -> None:
        self.log.append(message)

    def set_stage(self, stage: Stage) -> None:
        self.stage = stage
        self.phase_started_at = time.monotonic()

    def abort(self) -> None:
        """Release the knob without storing anything."""
        self._send("CAL_ABORT")
        self.log.append("Released the knob (no calibration written).")

    def upload(self, blob: bytes) -> None:
        """Send a fitted calibration. The knob stores it and reboots."""
        self._send(f"CAL_UPLOAD {blob.hex()}")
        self.log.append(f"Sent {len(blob)} byte calibration; the knob will reboot.")

    def feed_line(self, line: str) -> None:
        message = parse_line(line)
        if message is None:
            return

        if isinstance(message, CalFrame):
            self.datasets[message.step].append(list(message.field))

        elif isinstance(message, CalState):
            self._handle_state(message)

        elif isinstance(message, Status):
            if message.is_tare_begin:
                # The window is open. Only claim it once -- a second tare
                # during an active run should not restart capture.
                if self.stage == "waiting_for_tare":
                    self.set_stage("capturing")
                    self.log.append("Knob is in tare. Starting calibration...")
                    self._send("CAL_START")
            elif message.is_complete:
                self.completed = True
                self.log.append("All hardware calibration steps complete!")
            elif message.is_awaiting_upload:
                self.log.append("Knob is holding, ready to receive a calibration.")
            else:
                self.log.append(f"[KNOB ALERT] {message.raw}")

        elif isinstance(message, Unknown):
            self.log.append(f"[UNRECOGNIZED] {message.raw}")

    def _handle_state(self, message: CalState) -> None:
        self.current_step = message.step
        self.current_phase = message.phase
        self.phase_started_at = time.monotonic()
        step_name = STEP_NAMES.get(message.step, f"Step {int(message.step)}")

        if message.phase == CalibPhase.WAIT_FOR_START_BTN:
            self.log.append(f"--- {step_name} ---")
            self.log.append("Ready. Press LEFT button on the knob to begin recording.")
            self.datasets[message.step] = []  # clear buffer in case this is a retry

        elif message.phase == CalibPhase.COUNTDOWN:
            self.log.append("Get ready...")

        elif message.phase == CalibPhase.RECORDING:
            self.log.append("Recording...")

        elif message.phase == CalibPhase.WAIT_FOR_ACK:
            count = len(self.datasets[message.step])
            self.log.append(f"Done recording. Received {count} frames.")
            self._send(f"CAL_ACK {count}")

    def frame_count(self, step: CalibStep) -> int:
        return len(self.datasets[step])

    def status_for(self, step: CalibStep) -> StepStatus:
        if self.completed or step < self.current_step:
            return "done"
        if step == self.current_step:
            return "active"
        return "pending"
