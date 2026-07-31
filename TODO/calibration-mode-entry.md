# Design how a user actually enters bundle-calibration mode from the knob

Split out from `TODO/tare-and-calibration.md` — that file is about the
boot-time tare step; this one is specifically about the UX for entering the
guided PC-assisted bundle calibration (`BundleState` /
`BundleCalibrationController`). Needs real UX thinking, not just a bug fix.

Note: another Claude session is actively working on the calibration code
(`firmware/src/controllers/BundleCalibrationController.cpp`,
`magnet_field_model/calibration/`) — check current state of those files
before touching the C++ side here, this file is scoping/design only.

## Current state (as of this writing)

`CalibratingState::update()` (`firmware/src/states/CalibratingState.cpp`)
still sends *any* input activity straight into `BundleState` mid-boot-calibration:

```cpp
if (inputController.takeActivity()) {
    stateMachine.changeState(&StateMachine::bundleState);
}
```

`takeActivity()` fires on any button press/release, so a stray tap during the
~2s post-boot calibration window silently aborts calibration and drops into
the serial calibration protocol — which then just sits there waiting for a PC
to speak `CAL_START`, so the mouse looks hung to anyone who didn't mean to
trigger it. This is very likely how the entry point was left as a
placeholder/dev shortcut rather than a designed gesture.

Related caller-side bug still present in `BundleState::update()`
(`firmware/src/states/BundleState.cpp:25`):

```cpp
uint16_t button_bits = inputController.takeActivity();  // should this be buttonBits()?
bundleCalibrationController.update(button_bits, sensorController);
```

`takeActivity()` returns a bool, not the actual button bitmask
(`InputController::buttonBits()`). `BundleCalibrationController::update()`'s
own bit-testing (the `& 1` / `& 2` checks inside it) has already been fixed by
the other session, but this caller is still handing it the wrong value to
test in the first place.

## What needs deciding

This isn't just "swap in the right function call" — the actual question is
what the deliberate, discoverable gesture for entering bundle calibration
should be, on a device with exactly two buttons and no display. Things to
think through:

- Is bundle calibration knob-triggerable at all, or does it only start from
  the PC side (`bundle_callibration.py` sends `CAL_START` over serial, and the
  knob just responds — no on-device gesture needed)? That would sidestep the
  whole "which gesture" question, since the protocol already supports
  PC-initiated start (`handle_serial_command` on `"CAL_START"`).
  `CalibratingState`'s current activity-triggered jump *emulates* a
  knob-side entry, badly — worth deciding if that's even a feature to keep.
- If it should be knob-triggerable, it needs a gesture distinct from the
  tare-redo gesture (see `TODO/tare-and-calibration.md`) and from the normal
  calibration-request gesture already in `InputController`
  (`takeCalibrationRequest()`, both buttons held 3s) — three-way collision
  risk if these aren't clearly separated (tare redo vs. bundle-calibration
  entry vs. anything else).
- Whatever the gesture is, it should not be reachable by accident during the
  normal boot sequence, unlike today.

## Also in scope once the above is decided

- Fix `BundleState.cpp:25` to pass the real button bitmask instead of
  `takeActivity()`, consistent with whatever entry design is chosen.
- See `TODO/calibration-led-animations.md` for the separate (but related)
  question of what the LED ring should show during each phase — entry-mode UX
  and in-progress animation are two different problems, tracked separately.
