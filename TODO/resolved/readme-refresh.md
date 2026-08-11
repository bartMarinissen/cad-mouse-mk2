> **RESOLVED — closed by deleting `firmware/README.md` outright.**
>
> Not by the rewrite this file asks for. The stale motion pseudocode was the
> bulk of the file, and everything else in it was a second copy of something
> another file already owns: the tuning constants are declared in
> `firmware/include/Config.h`, the axis conventions in that header and in
> `magnet_model/positions.h`, and the driver-support note belonged next to the
> 3Dconnexion USB identity in `platformio.ini`, where it now lives along with
> the demo video link. Nothing was left for the README to be the owner of.
>
> Kept for the reasoning below on *why* a rewrite was the wrong fix — it would
> have put a second paraphrase of the pipeline somewhere nobody re-reads, and
> rotted the same way. See `CLAUDE.md`, "One owner per fact".

---

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
*rewritten* to match reality — i.e. to swap the old pseudocode for a
description of the solver. Don't do that. `ARCHITECTURE.md` owns how the
pipeline works (see `CLAUDE.md`, "One owner per fact"), and a second
paraphrase of it here would be accurate only until the solver next changes,
then rot exactly the way this one did, in the same place nobody re-reads.

Instead:

- **Delete** the sensor-averaging pseudocode and the "current implementation"
  framing outright.
- Replace it with a pointer, not a paraphrase: a sentence naming what the
  motion pipeline is ("a Gauss-Newton pose solve against a magnetic forward
  model") and sending the reader to `ARCHITECTURE.md` for how and
  `design documentation/Math.md` for why. That much won't rot — it's the
  fork's entire identity — but don't restate the stage-by-stage chain.
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
