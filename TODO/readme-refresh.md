# Update `firmware/README.md` to describe the current motion model

Originated from ARCHITECTURE.md issue #10. Left open, no redesign attached —
just needs doing.

## Problem

`firmware/README.md` documents the old per-axis-averaging motion heuristic
(`Tx = (mag1x + mag2x + mag3x) / 3`, etc.) that this fork replaced. The actual
current pipeline is the Gauss-Newton pose solver described in
`ARCHITECTURE.md` (`MotionController::compute()` → `solve_knob_pose()` →
`ForwardModel` → `Sensor` → `MagnetModel` → `BicubicField`). Low risk of
confusing the firmware itself, but will mislead a human — or a fresh Claude
session that reads `firmware/README.md` before `ARCHITECTURE.md` — into
thinking the old linear heuristic is still in effect.

## What to do

Rewrite the "current implementation" section of `firmware/README.md` to match
reality: what actually drives `GAIN_T`/`GAIN_R`/`SIGN_AXIS`/dead
zones/smoothing now (still accurate — those apply after the pose solve, see
`MotionController::compute()`), and replace the stale sensor-averaging
pseudocode with a short description of the solved-pose pipeline, pointing at
`ARCHITECTURE.md` for the full breakdown rather than duplicating it.
