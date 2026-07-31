# TODO

Tracked follow-ups from the architecture review in `../ARCHITECTURE.md`. One
file per combined issue.

- [tare-and-calibration.md](tare-and-calibration.md) — replace boot
  `CalibratingState` with a proper tare step (sanity checks, redoable via a
  gesture); bundle calibration itself is separate and still undesigned.
- [calibration-mode-entry.md](calibration-mode-entry.md) — the knob-side UX
  for deliberately entering bundle calibration mode (currently entered by
  accident via any button activity during boot tare).
- [calibration-led-animations.md](calibration-led-animations.md) — the
  per-phase LED ring animations bundle calibration wants but doesn't have yet
  (`LEDController` only has solid/spinner/off).
- [controller-ownership.md](controller-ownership.md) — `SensorController` /
  `MotionController` circular coupling, and the forward model's state living
  outside any controller.
- [sensor-gain-calibration.md](sensor-gain-calibration.md) — move sensor
  gain/skew correction into `SensorController`, out of the motion/solver
  system entirely.
- [telemetry-rework.md](telemetry-rework.md) — `rcond` stub +
  `TelemetryController::publish()`'s ever-growing parameter list, as one
  combined rework.
- [readme-refresh.md](readme-refresh.md) — `firmware/README.md` still
  documents the replaced motion heuristic.
- [cross-magnet-interference.md](cross-magnet-interference.md) — modeling
  each sensor's field contribution from the two magnets it isn't paired with,
  via a cheap dipole approximation. Hopefully unnecessary — check the error
  magnitude first.
- [multicore.md](multicore.md) — empty, reserved.
- [Performance.md](Performance.md) — `solve_pose` is taking ~10ms of the 20ms
  50Hz budget; assembly-verified hypotheses on where the time goes (soft
  float, Eigen not inlining, RAM/flash veneer hops), not yet a fix plan.

Resolved without a TODO entry: the `solve_pose.cpp` Jacobian assignment bug
(fixed), and the unwired `custom_memmap.ld`/`full_custom_memmap.ld` (removed).
