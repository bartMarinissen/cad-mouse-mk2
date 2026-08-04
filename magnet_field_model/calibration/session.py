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
from typing import Literal

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

StepStatus = Literal["done", "active", "pending"]


class CalibrationSession:
    LOG_MAXLEN: int = 200

    _send: Callable[[str], None]
    datasets: dict[CalibStep, list[list[float]]]
    current_step: CalibStep
    current_phase: CalibPhase
    phase_started_at: float
    completed: bool
    log: deque[str]

    def __init__(self, send_fn: Callable[[str], None]):
        self._send = send_fn
        self.datasets = {step: [] for step in CalibStep if int(step) >= 0}
        self.current_step = CalibStep.NONE
        self.current_phase = CalibPhase.IDLE
        self.phase_started_at = time.monotonic()
        self.completed = False
        self.log = deque(maxlen=self.LOG_MAXLEN)

    def start(self) -> None:
        self.log.append("Initiating calibration protocol...")
        self._send("CAL_START")

    def feed_line(self, line: str) -> None:
        message = parse_line(line)
        if message is None:
            return

        if isinstance(message, CalFrame):
            self.datasets[message.step].append(list(message.field))

        elif isinstance(message, CalState):
            self._handle_state(message)

        elif isinstance(message, Status):
            if message.is_complete:
                self.completed = True
                self.log.append("All hardware calibration steps complete!")
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

        elif message.phase == CalibPhase.REVIEW:
            # NOTE: the firmware currently advances on ANY button press here
            # (see BundleCalibrationController::update, REVIEW case) - the
            # left-button-to-retry path is a TODO on the firmware side and
            # isn't wired up yet, so we don't promise it in the UI.
            self.log.append("Press a button on the knob to continue to the next step.")

    def frame_count(self, step: CalibStep) -> int:
        return len(self.datasets[step])

    def status_for(self, step: CalibStep) -> StepStatus:
        if self.completed or step < self.current_step:
            return "done"
        if step == self.current_step:
            return "active"
        return "pending"
