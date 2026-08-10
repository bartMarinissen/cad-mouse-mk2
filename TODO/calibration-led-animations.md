# LED ring animations for bundle calibration

Note: another Claude session is actively working on the calibration code —
check current state of `BundleCalibrationController.cpp` and `LEDController.*`
before implementing anything here, this file is scoping/design only.

## Current state

`LEDController` (`firmware/include/controllers/LEDController.h`,
`firmware/src/controllers/LEDController.cpp`) only has three display modes:
`setSolid(color)`, `startSpinner(color)` (a single-pixel sweep around the
ring, one color, fixed 60ms step), and `off()`. `BundleState::enter()` just
calls `startSpinner(0xFDFDFF)` once and leaves it running for the entire
calibration session, regardless of step or phase.

But `BundleCalibrationController::update()`
(`firmware/src/controllers/BundleCalibrationController.cpp`) already has the
intended animation states marked inline, per calibration phase, none of them
implemented:

```cpp
case CalibPhase::WAIT_FOR_START_BTN:
    // LED RING: animating the step with its specific animation
case CalibPhase::COUNTDOWN:
    // LED RING: filling up clockwise
case CalibPhase::RECORDING:
    // LED RING: animating the step with its specific animation
case CalibPhase::WAIT_FOR_ACK:
    // LED RING: dead
```

## What needs deciding

- What each phase's animation actually looks like. The comments imply at
  least: a per-*step* identifying animation (distinguishing STATIONARY /
  FLAT_CIRCLE / PITCH / ROLL / TWIST / HEAVE / RANDOM from each other, shown
  during `WAIT_FOR_START_BTN` and `RECORDING`), a clockwise fill during the
  1s `COUNTDOWN`, and going dark during `WAIT_FOR_ACK`. None of that is
  designed yet beyond the comment placeholders. (There used to be a `REVIEW`
  phase with its own "success" loop between a step's data landing and the
  user confirming to continue; it's gone now -- a successful ACK moves
  straight to the next step's `WAIT_FOR_START_BTN`, since there was never a
  way to go back and a confirmation press had nothing to decide.)
- Whether a per-step "identifying animation" means 7 distinct animations
  (one per `CalibStep`) or a smaller shared vocabulary (e.g. color-coded by
  step, same animation shape).
- Whether `LEDController` needs new primitives to support this (e.g. a
  progress-fill mode for the countdown, a distinct "recording" pulse, a
  "success" flourish) or whether `BundleCalibrationController` should drive
  raw pixel control itself for calibration-specific effects instead of going
  through `LEDController`'s existing solid/spinner modes.
- How this interacts with `Config::LED_CALIBRATING_COLOR` /
  `LED_ERROR_COLOR` / `LED_IDLE_COLOR` conventions already used elsewhere —
  should calibration-step colors be added to `Config.h` alongside those, or
  live separately since they're per-step rather than per-state?
