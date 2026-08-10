"""Parsing for the bundle calibration serial protocol.

This module mirrors the wire format produced by
firmware/src/controllers/BundleCalibrationController.cpp. It is pure
parsing - no serial I/O, no printing, no state - so it can be tested with
plain strings.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class CalibStep(IntEnum):
    """Mirrors CalibStep in firmware/include/controllers/BundleCalibrationController.h"""

    NONE = -1
    STATIONARY = 0
    FLAT_CIRCLE = 1
    PITCH = 2
    ROLL = 3
    TWIST = 4
    HEAVE = 5
    RANDOM = 6
    COMPLETE = 7


class CalibPhase(IntEnum):
    """Mirrors CalibPhase in firmware/include/controllers/BundleCalibrationController.h"""

    IDLE = 0
    WAIT_FOR_START_BTN = 1
    COUNTDOWN = 2
    RECORDING = 3
    WAIT_FOR_ACK = 4
    AWAITING_UPLOAD = 5


STEP_NAMES: dict[CalibStep, str] = {
    CalibStep.STATIONARY: "STATIONARY (Resting Pose)",
    CalibStep.FLAT_CIRCLE: "FLAT CIRCLE (XY Translation)",
    CalibStep.PITCH: "PITCH (Forward/Back Tilt)",
    CalibStep.ROLL: "ROLL (Left/Right Tilt)",
    CalibStep.TWIST: "TWIST (Yaw Rotation)",
    CalibStep.HEAVE: "HEAVE (Z-Axis Push/Pull)",
    CalibStep.RANDOM: "RANDOM (Freeform Wiggle)",
}

# Cosmetic-only mirror of BundleCalibrationController constants. The
# firmware is the sole authority on actual timing (it tells us via
# CAL_STATE); these are only used to draw a countdown/progress bar while
# COUNTDOWN/RECORDING are in flight.
FRAME_INTERVAL_MS = 50  # 20Hz, matches FRAME_INTERVAL_MS in the firmware
COUNTDOWN_MS = 1000  # matches COUNTDOWN_MS in the firmware


def step_duration_ms(step: CalibStep) -> int:
    if step == CalibStep.STATIONARY:
        return 1000
    return 3000


@dataclass(frozen=True)
class CalFrame:
    step: CalibStep
    tick: int
    field: tuple[float, ...]


@dataclass(frozen=True)
class CalState:
    step: CalibStep
    phase: CalibPhase


@dataclass(frozen=True)
class Status:
    raw: str
    is_complete: bool
    # The knob has entered tare and will accept CAL_START / CAL_UPLOAD for the
    # ~1.7s that lasts. Waiting for this instead of firing CAL_START blind is
    # what makes starting the script before touching the knob work.
    is_tare_begin: bool = False
    # All poses captured; the knob is holding in calibration mode rather than
    # dropping back to being a mouse, so a fitted result can go straight back.
    is_awaiting_upload: bool = False


@dataclass(frozen=True)
class Unknown:
    raw: str


ProtocolMessage = CalFrame | CalState | Status | Unknown


def parse_line(line: str) -> ProtocolMessage | None:
    """Parse one line of serial traffic from the RP2040 into a typed message.

    Returns None for blank/unparseable lines, matching the original script's
    `if not line: continue` behavior.
    """
    line = line.strip()
    if not line:
        return None

    parts = line.split()
    if not parts:
        return None

    cmd = parts[0]

    if cmd == "CAL_FRAME":
        # Format: CAL_FRAME <step_id> <tick> <Bx1> ... <Bz3>
        if len(parts) != 12:  # 1 cmd + 2 meta + 9 floats
            return Unknown(line)
        step_id = int(parts[1])
        tick = int(parts[2])
        field = tuple(float(x) for x in parts[3:12])
        return CalFrame(step=CalibStep(step_id), tick=tick, field=field)

    if cmd == "CAL_STATE":
        # Format: CAL_STATE <step_id> <phase>
        step_id = int(parts[1])
        phase_id = int(parts[2])
        return CalState(step=CalibStep(step_id), phase=CalibPhase(phase_id))

    if cmd == "STATUS":
        return Status(
            raw=line,
            is_complete="ALL_STEPS_COMPLETE" in line,
            is_tare_begin="TARE_BEGIN" in line,
            is_awaiting_upload="AWAITING_UPLOAD" in line,
        )

    return Unknown(line)
