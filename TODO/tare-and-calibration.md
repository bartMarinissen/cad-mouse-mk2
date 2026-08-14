# Replace `CalibratingState` with a proper "tare" step

Originated from ARCHITECTURE.md issues #1 and #2.

## The problem

The current `CalibratingState` (`firmware/src/states/CalibratingState.cpp`) just
averages 200 samples and calls it done, with no sanity checking and no proper
way back in once done. The whole boot-time calibration story is getting
redesigned as a "tare" step instead of patched in place.


## What tare should do
The process of deciding a rest pose should be purely a
motion controller concern.

The process of entering tare (3 second hold of both buttons) is mostly fine.
Perhaps it should be sped up a little.

At boot the sensor should do a magnet polarity check. 
If that fails it should fall back to default calibration with adjusted magnet 
polarity. Beyond that the sensor does not need to be involved in taring.

Further sanity checks might be left for the actual tare, though how to handle
that is up for discussion (just go to error state?)

These sanity checks are:
- Knob started roughly in the expected rest position (position sanity check
  against `Positions::approx_rest_pos`, not just "whatever the first reading
  says").
- Magnetic field polarization makes sense (catches a magnet installed
  reversed, or a sensor wired/addressed wrong — currently nothing checks this
  at all).
- Residual at rest is below a threshold (uses the pose-solver residual that
  `MotionController::read_pose()` already returns — currently computed but
  never gated on).
- Variance on measured pose is small enough


## Explicitly not scoped here: true calibration

Real calibration (fixing per-magnet/per-sensor tolerance, i.e. what
`BundleCalibrationController` / `BundleState` / `bundle_callibration.py` are
for) is a separate effort with its own open design
question ("how is yet to be determined") — don't conflate tare with it. Tare
is a fast/automatic per-boot sanity+zero step; bundle calibration is a slower,
guided, PC-assisted one-time-per-unit hardware calibration. Both now work end
to end (capture, fit, and delivery back to the device — see `ARCHITECTURE.md`),
so the sensor-gain ownership question this file used to defer to is settled:
gain lives in `SensorController`, fed from `CalibrationParams`. See
`TODO/calibration-mode-entry.md` / `TODO/calibration-led-animations.md` for
the two C++-side calibration-UX TODOs (entry gesture, LED ring animations).

## Open questions

- What tare does on sanity-check failure (retry in place? drop to
  `ErrorState` (the likely solution) ? something else?).
