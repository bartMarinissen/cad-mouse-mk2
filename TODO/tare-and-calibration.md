# Replace `CalibratingState` with a proper "tare" step

Originated from ARCHITECTURE.md issues #1 and #2.

## The problem

The current `CalibratingState` (`firmware/src/states/CalibratingState.cpp`) just
averages 200 samples and calls it done, with no sanity checking and no proper
way back in once done. The whole boot-time calibration story is getting
redesigned as a "tare" step instead of patched in place.

(The specific bug where `CalibratingState` accidentally falls into
`BundleState` on any button activity, and the related question of what the
*deliberate* gesture for entering bundle calibration should be, has been
split out into its own file — see `TODO/calibration-mode-entry.md`. That's a
UX-design question about bundle calibration's entry point specifically; this
file is about what tare itself should do.)

## What tare should do

Runs at the beginning (replacing today's `CalibratingState`), and should also
be re-runnable via a long double-press gesture at any time (this replaces /
absorbs the existing "hold both buttons 3s" `takeCalibrationRequest()` gesture
in `InputController` — confirm whether that gesture is kept as-is or the
"double press" trigger is a different gesture to be added).

Beyond just averaging a baseline, tare should run sanity checks before
accepting the result:

- Knob started roughly in the expected rest position (position sanity check
  against `Positions::approx_rest_pos`, not just "whatever the first reading
  says").
- Magnetic field polarization makes sense (catches a magnet installed
  reversed, or a sensor wired/addressed wrong — currently nothing checks this
  at all).
- Residual at rest is below a threshold (uses the pose-solver residual that
  `MotionController::read_pose()` already returns — currently computed but
  never gated on).

If any check fails, tare should presumably reject/retry rather than silently
accepting a bad baseline (today's `CalibratingState` has no failure path other
than running forever).

## Explicitly not scoped here

Real calibration (fixing per-magnet/per-sensor tolerance, i.e. what
`BundleCalibrationController` / `BundleState` / `bundle_callibration.py` are
for) is a separate, still-under-construction effort with its own open design
question ("how is yet to be determined") — don't conflate tare with it. Tare
is a fast/automatic per-boot sanity+zero step; bundle calibration is a slower,
guided, PC-assisted one-time-per-unit hardware calibration. Both now work end
to end (capture, fit, and delivery back to the device — see `ARCHITECTURE.md`),
so the sensor-gain ownership question this file used to defer to is settled:
gain lives in `SensorController`, fed from `CalibrationParams`. See
`TODO/calibration-mode-entry.md` / `TODO/calibration-led-animations.md` for
the two C++-side calibration-UX TODOs (entry gesture, LED ring animations).

## Open questions

- Exact gesture for "redo tare" (is it the existing 3s both-buttons hold, or a
  new double-press gesture as literally described?).
- What tare does on sanity-check failure (retry in place? drop to
  `ErrorState`? something else?).
- Whether/how tare composes with bundle calibration once that exists (e.g.
  does bundle calibration require a fresh tare first?).
