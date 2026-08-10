"""Tests for the PC side of the bundle calibration protocol.

CalibrationSession takes its send function by injection precisely so this is
possible without a serial port, and the handshake is the one part of the
delivery path that can be checked without a knob on the desk. Everything
downstream of it - the knob storing the blob, rebooting into it - still needs
hardware.
"""

from __future__ import annotations

from calibration.protocol import CalibPhase, CalibStep
from calibration.session import CalibrationSession


def _session() -> tuple[CalibrationSession, list[str]]:
    sent: list[str] = []
    return CalibrationSession(send_fn=sent.append), sent


def test_start_sends_nothing_and_waits_for_tare():
    """The knob only accepts CAL_START during its ~1.7s tare window, so firing
    it on start() meant the command landed nowhere whenever the script was
    started before the knob was put into tare - which is the normal order."""
    session, sent = _session()
    session.start()
    assert sent == []
    assert session.stage == "waiting_for_tare"


def test_tare_begin_triggers_cal_start():
    session, sent = _session()
    session.start()
    session.feed_line("STATUS TARE_BEGIN")
    assert sent == ["CAL_START"]
    assert session.stage == "capturing"


def test_a_later_tare_does_not_restart_capture():
    """The knob re-announces TARE_BEGIN every time it tares, including any
    re-tare during a run. Claiming the window twice would restart capture from
    the first pose and quietly discard what had been collected."""
    session, sent = _session()
    session.start()
    session.feed_line("STATUS TARE_BEGIN")
    session.feed_line("STATUS TARE_BEGIN")
    assert sent == ["CAL_START"]


def test_frames_are_collected_and_acked_with_their_count():
    session, sent = _session()
    session.start()
    session.feed_line("STATUS TARE_BEGIN")

    for _ in range(3):
        session.feed_line("CAL_FRAME 0 100 1 2 3 4 5 6 7 8 9")
    assert session.frame_count(CalibStep.STATIONARY) == 3

    session.feed_line(f"CAL_STATE 0 {int(CalibPhase.WAIT_FOR_ACK)}")
    assert sent[-1] == "CAL_ACK 3"


def test_wait_for_start_clears_a_retried_step():
    """Re-entering WAIT_FOR_START_BTN for a step means the knob is redoing it,
    so the frames already banked for that step are stale."""
    session, _ = _session()
    session.start()
    session.feed_line("STATUS TARE_BEGIN")
    session.feed_line("CAL_FRAME 0 100 1 2 3 4 5 6 7 8 9")
    session.feed_line(f"CAL_STATE 0 {int(CalibPhase.WAIT_FOR_START_BTN)}")
    assert session.frame_count(CalibStep.STATIONARY) == 0


def test_all_steps_complete_marks_the_session_done():
    session, _ = _session()
    session.start()
    session.feed_line("STATUS TARE_BEGIN")
    assert not session.completed
    session.feed_line("STATUS ALL_STEPS_COMPLETE")
    assert session.completed


def test_awaiting_upload_is_recognised_not_treated_as_an_alert():
    """The knob holding in calibration mode is the expected end of a run, not
    something to surface as a knob alert."""
    session, _ = _session()
    session.start()
    session.feed_line("STATUS TARE_BEGIN")
    session.feed_line("STATUS AWAITING_UPLOAD")
    assert not any("KNOB ALERT" in line for line in session.log)


def test_abort_releases_the_knob():
    session, sent = _session()
    session.abort()
    assert sent == ["CAL_ABORT"]


def test_upload_sends_the_blob_as_hex():
    """Hex rather than raw bytes so the upload rides the same newline-delimited
    convention as the rest of the link - raw bytes would contain 0x0A and
    truncate the read on the firmware side."""
    session, sent = _session()
    blob = bytes(range(256))
    session.upload(blob)

    assert len(sent) == 1
    command, payload = sent[0].split(" ", 1)
    assert command == "CAL_UPLOAD"
    assert bytes.fromhex(payload) == blob
    assert payload.strip() == payload
    assert "\n" not in payload
