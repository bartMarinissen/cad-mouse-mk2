"""Drawing a CalibrationSession as a Rich Live display.

This module only knows how to render a CalibrationSession - it has no
idea a serial port or a background thread exists. Something else (see
collector.py) is responsible for actually running a session and deciding
when to redraw it.
"""

from __future__ import annotations

import time
from collections.abc import Iterator
from contextlib import contextmanager
from types import TracebackType

from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from .protocol import COUNTDOWN_MS, STEP_NAMES, CalibPhase, CalibStep, step_duration_ms
from .session import CalibrationSession

STEP_ORDER: list[CalibStep] = [
    step for step in CalibStep if int(step) >= 0 and step != CalibStep.COMPLETE
]

LOG_TAIL = 15
BAR_WIDTH = 40


def _ascii_bar(fraction: float, width: int = BAR_WIDTH) -> str:
    """A plain-ASCII progress bar (no block-drawing characters).

    Rich's own ProgressBar renderable, and any non-ASCII glyphs, can
    UnicodeEncodeError on a legacy Windows console (cp1252, no UTF-8/VT
    support) - keeping this ASCII-only means the TUI can't crash on that.
    """
    filled = int(round(max(0.0, min(1.0, fraction)) * width))
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def _next_step(step: CalibStep) -> CalibStep | None:
    index = STEP_ORDER.index(step)
    if index + 1 < len(STEP_ORDER):
        return STEP_ORDER[index + 1]
    return None


def _next_step_preview(step: CalibStep) -> Text:
    """What's coming up once the knob moves on from `step`.

    Shown as soon as we've got a step's data (WAIT_FOR_ACK), so the user
    knows what movement to do next before the knob advances.
    """
    next_step = _next_step(step)
    if next_step is not None:
        return Text(f"Next up: {STEP_NAMES[next_step]}", style="bold cyan")
    return Text("That was the last pose - this will finish calibration.", style="bold cyan")


def _step_tracker(session: CalibrationSession) -> Table:
    table = Table.grid(padding=(0, 1))
    for step in STEP_ORDER:
        status = session.status_for(step)
        if status == "done":
            glyph, style = "[x]", "green"
        elif status == "active":
            glyph, style = "[>]", "bold yellow"
        else:
            glyph, style = "[ ]", "grey50"
        table.add_row(Text(glyph, style=style), Text(STEP_NAMES.get(step, str(step)), style=style))

    # Solving happens on the PC, not the knob, so it has no CalibStep of its
    # own - approximate it from the session stage instead so the tracker
    # doesn't just stop at RANDOM as if there were nothing left to do.
    if session.stage == "solving":
        glyph, style = "[>]", "bold yellow"
    elif session.stage in ("confirming", "finished"):
        glyph, style = "[x]", "green"
    else:
        glyph, style = "[ ]", "grey50"
    table.add_row(Text(glyph, style=style), Text("COMPUTE CALIBRATION", style=style))
    return table


def _current_panel(session: CalibrationSession) -> Panel:
    step = session.current_step
    phase = session.current_phase
    elapsed_ms = (time.monotonic() - session.phase_started_at) * 1000

    # Once a step's data is in and the knob is about to advance on its own
    # (WAIT_FOR_ACK), that step is effectively done - fade it out so the
    # "Next up" preview reads as the thing to focus on.
    awaiting_confirmation = phase == CalibPhase.WAIT_FOR_ACK
    header_style = "grey50" if awaiting_confirmation else "bold"

    # The stages that happen off the knob get the panel to themselves: the
    # step/phase machinery below has nothing to say once capture is over, and
    # the solve and the write decision are still part of the same run.
    if session.stage == "waiting_for_tare":
        return Panel(
            Group(
                Text("Waiting for the knob", style="bold yellow"),
                Text(""),
                Text("Hold BOTH buttons for about 3 seconds to enter tare."),
                Text("Calibration starts automatically once it does.", style="grey50"),
            ),
            title="Current Step",
            border_style="yellow",
        )

    if session.stage == "solving":
        elapsed_s = time.monotonic() - session.phase_started_at
        return Panel(
            Group(
                Text("Solving the bundle adjustment...", style="bold cyan"),
                Text(f"{elapsed_s:0.1f}s elapsed", style="grey50"),
                Text(""),
                Text("The knob is holding in calibration mode.", style="grey50"),
            ),
            title="Current Step",
            border_style="cyan",
        )

    if session.stage == "confirming" and session.summary is not None:
        # Just the question - the numbers to judge it by are already sitting
        # in the steps/log panels either side of this one, in the same
        # positions they've been in the whole run.
        return Panel(
            Group(
                Text("Write this calibration to the knob?", style="bold yellow"),
                Text(""),
                Text("Type y or n on your keyboard, then press Enter.", style="grey50"),
            ),
            title="Decision",
            border_style="yellow",
        )

    if session.stage == "finished" and session.summary is not None:
        # Whatever upload()/abort() last logged - "Sent N byte calibration...
        # will reboot" or "Released the knob (no calibration written)" -
        # already says exactly what happened.
        outcome_text = session.log[-1] if session.log else "Done."
        return Panel(Text(outcome_text, style="bold green"), title="Result", border_style="green")

    lines: list = [Text(STEP_NAMES.get(step, "Waiting to start..."), style=header_style)]

    if session.completed:
        lines.append(Text("All done!", style="bold green"))
    elif phase == CalibPhase.WAIT_FOR_START_BTN:
        lines.append(Text("Press any button on the knob to begin recording."))
    elif phase == CalibPhase.COUNTDOWN:
        remaining_s = max(0.0, (COUNTDOWN_MS - elapsed_ms) / 1000)
        lines.append(Text(f"Get ready... {remaining_s:0.1f}s", style="yellow"))
    elif phase == CalibPhase.RECORDING:
        fraction = elapsed_ms / step_duration_ms(step)
        lines.append(Text("Recording...", style="bold red"))
        lines.append(Text(_ascii_bar(fraction), style="red"))
        lines.append(Text(f"{session.frame_count(step)} frames captured"))
    elif phase == CalibPhase.WAIT_FOR_ACK:
        lines.append(Text("Sending confirmation to the knob...", style="grey50"))
        lines.append(_next_step_preview(step))
    else:
        lines.append(Text("Waiting for the knob..."))

    return Panel(Group(*lines), title="Current Step", border_style="blue")


def _log_panel(session: CalibrationSession) -> Panel:
    if session.stage in ("confirming", "finished") and session.summary is not None:
        # Trust diagnostics, not the event log - once there's a fit, whether
        # it converged and how much the data actually pinned down is more
        # useful here than a scrollback of CAL_FRAME/CAL_STATE chatter.
        return Panel(
            Group(session.summary.headline, session.summary.diagnostics),
            title="Fit diagnostics",
            border_style="grey50",
        )
    tail = list(session.log)[-LOG_TAIL:]
    return Panel(Text("\n".join(tail)), title="Log", border_style="grey50")


def _left_panel(session: CalibrationSession) -> Panel:
    if session.stage in ("confirming", "finished") and session.summary is not None:
        # What got fitted, in the same slot the step tracker was in a moment
        # ago - the panel that answers "is this run going okay" during
        # capture is the one that answers "is this result okay" once it ends.
        return Panel(session.summary.parameters, title="Fitted parameters", border_style="grey50")
    return Panel(_step_tracker(session), title="Steps", border_style="grey50")


_HEADER = Panel(Text("Bundle Calibration", justify="center", style="bold cyan"))


def render(session: CalibrationSession) -> Layout:
    """Build the Layout for the current session state.

    Always the same three regions (steps | current / log), start to finish -
    solving and the write decision change what each region shows, never
    where they are. Panels swapping position between "still running" and
    "done" was disorienting, is the whole reason this isn't still a Group
    that bypasses the grid for those stages.
    """
    layout = Layout()
    layout.split_column(Layout(name="header", size=3), Layout(name="body"))
    layout["header"].update(_HEADER)

    # Even split, not the 1:2 a bare step list would prefer: once there's a
    # fit, this column carries the sensor gain-matrix table, which wraps
    # every value onto its own line if it's squeezed much narrower than this.
    layout["body"].split_row(Layout(name="steps", ratio=1), Layout(name="main", ratio=1))
    layout["steps"].update(_left_panel(session))

    layout["main"].split_column(Layout(name="current", size=8), Layout(name="log", ratio=1))
    layout["main"]["current"].update(_current_panel(session))
    layout["main"]["log"].update(_log_panel(session))
    return layout


class LiveDisplay:
    """A Rich display that knows how to draw a CalibrationSession.

    This is the only piece of tui.py that touches the terminal. It has no
    opinion on where session updates come from or when the session is
    done - the caller (collector.py) decides that and just calls
    `update()` whenever it wants the screen redrawn.

    Deliberately not `screen=True`: an alternate-screen Live redraws in a
    reserved region and wipes it the moment it stops, which is exactly what
    made the write-confirmation prompt appear to blank the whole display.
    Without it, `update()` redraws in place like any other Rich Live, and
    `stop()` just leaves the last frame sitting in the normal scrollback -
    nothing to wipe, so nothing disappears.
    """

    _live: Live

    def __init__(self, session: CalibrationSession, console: Console | None = None) -> None:
        self._live = Live(render(session), console=console, refresh_per_second=10)

    def __enter__(self) -> LiveDisplay:
        self._live.__enter__()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self._live.__exit__(exc_type, exc_val, exc_tb)

    def update(self, session: CalibrationSession) -> None:
        self._live.update(render(session))

    @contextmanager
    def paused(self) -> Iterator[Console]:
        """Pause auto-refresh long enough to ask the user something.

        Live's background thread redraws on a timer, which would stomp on a
        prompt's characters as they're typed if left running underneath one.
        Stopping it is still required for that reason - but since this isn't
        an alternate screen, stopping just freezes the last frame in place in
        the normal scrollback rather than clearing it, so the prompt appears
        as the next line of terminal output, not in place of the display.
        """
        self._live.stop()
        try:
            yield self._live.console
        finally:
            self._live.start(refresh=True)
