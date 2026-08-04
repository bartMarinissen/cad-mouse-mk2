# Move sensor gain/skew handling into SensorController, out of the motion system

Originated from ARCHITECTURE.md issue #6. Scope widened per discussion — this
is its own TODO, not just a config-shape fix.

## Current state

`Config::magnet_gains` is 3 scalars, applied in `MotionController.cpp` as
`gain * Mat3::Identity()` when constructing each `Sensor`. But `Sensor`'s
actual field (`Mat3 sensor_gain` in `firmware/include/magnet_model/sensor.h`)
supports a full 3x3 matrix — scale *and* cross-axis skew per sensor.
`firmware/test/test_jacobian.cpp`'s "Realistic Manufacturing Tolerances"
scenario already exercises exactly that (non-identity gain with skew terms),
so the math side is ready; the config side isn't.

## Decision

Sensor gain (and presumably skew/tilt correction — whatever per-sensor
calibration produces) should be owned and applied by `SensorController`,
**fully outside the motion/solver system**. `MotionController`/`ForwardModel`/
`Sensor` should not know raw sensor readings needed correcting — by the time
`MotionController` sees a reading, it should already be gain-corrected.

This is a real architecture shift, not just moving a constant:
`SensorController::readRaw()` currently returns raw field values straight
from the sensor driver. Applying a per-sensor `Mat3` gain there means
`SensorController` needs to own calibration coefficients (currently there's
nowhere for those to live — see `TODO/controller-ownership.md` for the
related problem of the forward model's `Sensor`/`MagnetModel` instances not
being owned/reachable anywhere either).

## Open questions

- ~~Where do the calibration coefficients come from at runtime — hardcoded
  `Config` values (as today, just relocated), or loaded from somewhere
  persistent?~~ **Answered.** They come from a `CalibrationParams`
  (`firmware/include/CalibrationParams.h`), which `SensorController` is
  constructed from and holds const. `resolveCalibration()` in
  `firmware/src/Controllers.cpp` decides where that struct comes from: today
  always `Config::defaultCalibration()`, and that function is the single seam a
  LittleFS read gets added behind.
- Does `Sensor::sensor_gain` (in the solver's forward model) become redundant
  once `SensorController` pre-corrects readings, or does it stay for a
  different purpose (e.g. modeling sensor placement/orientation error vs. raw
  gain error)? If raw readings are already corrected before reaching
  `MotionController`, `Sensor::sensor_gain` may end up dead weight — worth
  deciding explicitly rather than leaving both paths half-active.
- How this connects to the bundle-calibration effort (`TODO/tare-and-calibration.md`)
  once that design lands — bundle calibration is the thing that would actually
  produce these gain values.
