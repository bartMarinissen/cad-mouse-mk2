# Strip the stale motion model out of `firmware/README.md`

Originated from ARCHITECTURE.md issue #10. Still open, but **the intended fix
changed**: see "What to do".

## Problem

`firmware/README.md` documents the old per-axis-averaging motion heuristic
(`Tx = (mag1x + mag2x + mag3x) / 3`, etc.) that this fork replaced. The actual
current pipeline is the Gauss-Newton pose solver described in
`ARCHITECTURE.md` (`MotionController::compute()` → `solve_knob_pose()` →
`ForwardModel` → `Sensor` → `MagnetModel` → `BicubicField`). Low risk of
confusing the firmware itself, but it will mislead a human — or a fresh Claude
session that reads `firmware/README.md` before `ARCHITECTURE.md` — into
thinking the old linear heuristic is still in effect.

## What to do

This file used to ask for the "current implementation" section to be
*rewritten* to match reality. **Don't do that.** Per `CLAUDE.md`, READMEs must
not describe the current state of the code at all — a rewritten
current-implementation section would be accurate for exactly as long as the
solver stays put, and would then rot the same way this one did, in the same
hard-to-notice place.

Instead:

- **Delete** the sensor-averaging pseudocode and the "current implementation"
  framing outright. Don't replace it with a description of the solver.
- Point at `ARCHITECTURE.md` for how motion processing works, and
  `design documentation/Math.md` for why.
- **Keep** the tuning content, which is what a README in `firmware/` should be
  for: which knobs exist in `Config.h`, what the axis conventions are, what
  each gain/dead-zone/smoothing constant affects. That part is still accurate
  and still applies — those constants act on the solved pose, after the pose
  solve, in `MotionController::compute()`.
- The upstream video link and driver-support notes are fine to keep; they're
  about using the device, not about what the code does.

While in there: the two ⚠️ notes inherited from upstream ("the motion
processing still needs work and may eventually be replaced entirely", and the
axis-bleed/linearity caveat) describe the *upstream* heuristic's problems and
read as if they apply to this fork. Replacing that math is the entire point of
this fork, so those should go too, or be reworded as history.
