# LED ring animations for bundle calibration

## Current state (updated — architecture pass landed)

The animation architecture has been reworked. `LEDController`
(`firmware/include/controllers/LEDController.h`,
`firmware/src/controllers/LEDController.cpp`) no longer has per-effect
methods (`setSolid`/`startSpinner`/`off` are gone). Instead:

- `firmware/include/animations/AnimationBase.h` — abstract base class
  (`update()`, `wantsPower()`), mirroring the existing `State` pattern.
- `firmware/include/animations/Animations.h` /
  `firmware/src/animations/Animations.cpp` — the concrete animations
  (`SolidAnimation`, `SpinnerAnimation`, `OffAnimation`), all in one file
  pair for now.
- `LEDController` owns the ring and a single statically-allocated slot
  (tagged union, no heap) for whichever animation is active, plus power
  management. Callers construct the animation they want and load it via
  `ledController.set(SomeAnimation(ledController.ring(), ...));`, then drive
  it every tick with `ledController.update()`.

`BundleCalibrationController::change_phase()` now calls
`ledController.set(SpinnerAnimation(ledController.ring(), Config::LED_CALIBRATING_COLOR));`
once per phase transition — so calibration currently shows a single uniform
placeholder spinner regardless of phase or step. The five `// LED RING: ...`
comments in `BundleCalibrationController::update()`'s switch are still there,
documenting the originally-intended per-phase look, e.g.:

```cpp
case CalibPhase::WAIT_FOR_START_BTN:
    // LED RING: placeholder spinner set in change_phase(); intended
    // look is a per-step identifying animation, still undesigned.
case CalibPhase::COUNTDOWN:
    // LED RING: placeholder spinner set in change_phase(); intended
    // look is a clockwise fill, still undesigned.
case CalibPhase::RECORDING:
    // LED RING: placeholder spinner set in change_phase(); intended
    // look is a per-step identifying animation, still undesigned.
case CalibPhase::WAIT_FOR_ACK:
    // LED RING: placeholder spinner set in change_phase(); intended
    // look is "dead" (off), still undesigned.
```

None of the actual per-phase/per-step visuals below are designed or
implemented yet — that's the remaining work this doc scopes.

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
- New animations needed: a progress-fill for the countdown, a distinct
  per-step "recording" animation, a "success" flourish for review, etc. Each
  is a new `AnimationBase` subclass in `firmware/include/animations/Animations.h`
  / `Animations.cpp` (or a new file, if that one gets unwieldy), constructed
  and loaded the same way `SpinnerAnimation` is now:
  `ledController.set(YourAnimation(ledController.ring(), ...));`.
- How this interacts with `Config::LED_CALIBRATING_COLOR` /
  `LED_ERROR_COLOR` / `LED_IDLE_COLOR` conventions already used elsewhere —
  should calibration-step colors be added to `Config.h` alongside those, or
  live separately since they're per-step rather than per-state?
