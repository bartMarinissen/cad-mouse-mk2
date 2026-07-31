# Telemetry rework

Originated from ARCHITECTURE.md issues #7 and #8, merged into one effort per
discussion.

## Issue A: `rcond` is a hardcoded stub

`IdleState.cpp` sets `float rcond = 1.0;` with a commented-out line right
above it:

```cpp
//auto eig_vals = motionController.statistics.last_jacobian.jacobiSvd().singularValues();
```

This isn't dead-code-by-accident — it's intentionally stubbed out because
computing a condition number via SVD every frame is too expensive to do
unconditionally on an FPU-less RP2040 in the solver's hot path. The value
still gets threaded all the way through `TelemetryController::publish()` and
rendered on the dashboard (`Rcond: %5.0f`), so right now it's a fixed,
misleading number on the display rather than an omitted one.

Options to weigh as part of the rework (not decided yet):
- Compute it only every N ticks (amortize the SVD cost), rather than never.
- Drop it from the dashboard entirely until there's a cheap way to get it.
- Replace with a cheaper conditioning proxy that doesn't need a full SVD.

## Issue B: `TelemetryController::publish()`'s signature keeps growing

Currently 9 parameters (`motion`, `residual_percent`, `buttonBits`,
`hidReportSent`, `stats`, `raw_field`, `last_pos`, `last_rot`, `rcond`) in
`firmware/include/controllers/TelemetryController.h` /
`firmware/src/controllers/TelemetryController.cpp`, and it's grown with every
diagnostic added so far. The fixed-layout ASCII-dashboard approach
(hand-indexed `char buffer[kRows][kCols]`, manual `memcpy` per row) also makes
each new field a manual layout edit.

## Scope of the rework

Fold both into one pass rather than patching `rcond` in isolation:
- Collapse the parameter list into a single snapshot/struct passed by const
  reference, so adding a new diagnostic doesn't mean touching the function
  signature again.
- Decide what `rcond` actually is going forward (see options above) as part of
  deciding what the snapshot struct carries.
- Worth revisiting whether the fixed-grid ASCII dashboard format itself is
  still the right approach once the data being displayed is being redesigned
  anyway (no decision here — just flagging it's in scope to reconsider while
  touching this).
