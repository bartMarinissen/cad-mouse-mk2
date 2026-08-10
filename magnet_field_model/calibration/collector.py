"""Runs a bundle calibration session end to end.

This is the control-flow layer, and the thing entrypoints should call. It
covers the whole run, not just the capture: wait for the knob to enter tare,
drive the 7 poses, solve the fit, and decide what happens to the result. All
of it inside one display, because tearing the screen down to run the solver
and print tables underneath it made the run feel like two separate programs.

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

import contextlib
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from rich.console import RenderableType
from rich.prompt import Confirm

from .protocol import CalibStep
from .serial_link import SerialLink
from .session import CalibrationSession
from .tui import LiveDisplay

Datasets = dict[CalibStep, list[list[float]]]


@dataclass
class SessionOutcome:
    """Everything a caller might want to know about how a run ended."""

    session: CalibrationSession
    result: Any | None = None
    uploaded: bool = False


def _capture(link: SerialLink, session: CalibrationSession, display: LiveDisplay) -> None:
    """Wait for a tare window, then drive capture until the knob says done."""
    session.start()
    while not session.completed:
        line = link.readline()
        if line:
            session.feed_line(line)
        display.update(session)


def run_calibration_session(
    link: SerialLink,
    *,
    solve: Callable[[Datasets], Any] | None = None,
    summarize: Callable[[Any], RenderableType] | None = None,
    make_blob: Callable[[Any], bytes] | None = None,
) -> SessionOutcome:
    """Capture, optionally solve, and optionally offer to write the result.

    `make_blob` is what decides whether the knob is held after capture. With
    it, the knob stays in calibration mode while the fit runs and the user is
    asked whether to write - which is the whole point of it holding. Without
    it there is nothing coming back, so the knob is released immediately
    rather than being left parked waiting for a result that will never arrive.

    Returns the session regardless of how the run ended, so the caller can
    still dump raw frames after a Ctrl+C.
    """
    session = CalibrationSession(send_fn=link.send)
    outcome = SessionOutcome(session=session)

    try:
        with LiveDisplay(session) as display:
            _capture(link, session, display)

            if not session.completed:
                return outcome

            # No result is coming back, so do not make the user watch the knob
            # sit in calibration mode waiting for one.
            if make_blob is None:
                session.abort()
                display.update(session)

            if solve is not None:
                session.set_stage("solving")
                display.update(session)
                outcome.result = solve(session.datasets)
                if summarize is not None:
                    session.summary = summarize(outcome.result)

            if make_blob is None or outcome.result is None:
                session.set_stage("finished")
                display.update(session)
                return outcome

            session.set_stage("confirming")
            display.update(session)
            with display.paused() as console:
                # The report is already sitting in the frozen last frame right
                # above this prompt (see LiveDisplay.paused) - printing it
                # again here would just duplicate it.
                write = Confirm.ask(
                    "Write this calibration to the knob?", default=True, console=console
                )

            if write:
                session.upload(make_blob(outcome.result))
                outcome.uploaded = True
            else:
                session.abort()

            session.set_stage("finished")
            display.update(session)
    except KeyboardInterrupt:
        # The knob is probably still holding in calibration mode. Best effort:
        # if the port is still usable, let it go rather than leaving it stuck.
        with contextlib.suppress(Exception):
            session.abort()

    return outcome
