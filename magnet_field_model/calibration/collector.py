"""Runs a bundle calibration data-collection session end to end.

This is the control-flow layer, and the thing entrypoints should call: it
starts CAL_START, reads and ACKs frames for all 7 poses via
CalibrationSession, and redraws the on-screen display as it goes. It
blocks until either STATUS ALL_STEPS_COMPLETE arrives or the user aborts
with Ctrl+C.

tui.py, by contrast, only knows how to draw a session - it has no idea a
serial port, or a "when are we done" question, even exists. That split is
deliberate: this module is free to change how data gets collected
(polling, retries, timeouts, ...) without tui.py ever needing to change,
and vice versa for how it's drawn.

Reads the serial link directly in this loop (no background thread): the
port's own read timeout (see SerialLink.open) already bounds each
readline() call, which is what keeps the display ticking between lines -
and at this protocol's data rate (20Hz, short lines), the OS/USB driver's
own receive buffer comfortably absorbs the gap between reads.
"""

from __future__ import annotations

from .serial_link import SerialLink
from .session import CalibrationSession
from .tui import LiveDisplay


def run_calibration_session(link: SerialLink) -> CalibrationSession:
    """Run the full calibration data-collection session end to end.

    Returns the CalibrationSession so the caller can check `.completed`
    and read `.datasets` regardless of how the run ended.
    """
    session = CalibrationSession(send_fn=link.send)

    try:
        with LiveDisplay(session) as display:
            session.start()
            while not session.completed:
                line = link.readline()
                if line:
                    session.feed_line(line)
                display.update(session)
    except KeyboardInterrupt:
        pass

    return session
