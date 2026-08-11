> **RESOLVED — archived for the reasoning.**
>
> The decision below is implemented: sensor gain and DC offset are applied by
> `SensorController::read_mT()`, entirely outside the motion/solver system.
> `Sensor` no longer carries a gain member at all, and the coefficients come
> from `CalibrationParams` rather than `Config` scalars.
>
> Per-magnet strength landed as a real mT polarization on `MagnetModel`
> (`eb74948`, `9e772aa`). The last open gap in this file — `export.py` still
> emitting a dimensionless multiplier instead of absolute mT — closed with
> `firmware_magnet_strength_mT()` (`5abbe41`, `0453da1`).
>
> Note the "Current state" section below is written in the present tense but
> describes the *pre-fix* code (`Config::magnet_gains` as 3 scalars, `Sensor`
> owning a `Mat3 sensor_gain`). It is history, not a description of the tree.
> The live description is **`ARCHITECTURE.md`**.

---

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
nowhere for those to live — see `TODO/resolved/controller-ownership.md` for the
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
- ~~Does `Sensor::sensor_gain` become redundant once `SensorController`
  pre-corrects readings?~~ **Moot.** `Sensor` no longer has that field at all —
  it holds only `sensor_pos_global`. Gain lives solely in `SensorController`,
  and the solver never sees an uncorrected reading, so there is no half-active
  second path. The "Current state" section above is stale on this point.
- ~~Where does per-magnet strength live?~~ **Answered.** `MagnetModel` owns a
  `magnet_strength_mT` (a real polarization value, in mT — not the
  dimensionless multiplier it started as) and applies it in `evaluate()` as
  `magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT`. Sensor-side correction
  would have been numerically identical while `ForwardModel` pairs sensor *i*
  with magnet *i* one-to-one, but that stops holding once cross-magnet
  interference is modelled (`TODO/cross-magnet-interference.md`), and strength
  is a property of the magnet. Note gain and strength are one degree of freedom
  split by the fit's `det(G)=1` gauge — neither is meaningful without the
  other.
- **New gap, opened by the mT change:** `calibration/export.py` still emits a
  dimensionless multiplier centred on 1.0, not an absolute mT value on
  `magnet_strength_mT`'s scale — the two are not interchangeable, and pasting
  the current export straight in is wrong. Left alone deliberately (the
  request that produced the mT change was explicit about not touching the
  Python calibration code); updating `export.py` to emit real mT is follow-up
  work, not done here.
- How this connects to the bundle-calibration effort (`TODO/tare-and-calibration.md`)
  once that design lands — bundle calibration is the thing that would actually
  produce these gain values.
