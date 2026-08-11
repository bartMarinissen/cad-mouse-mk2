# Put the RP2040's second core to work

The Seeed XIAO RP2040 is dual-core (`ARCHITECTURE.md`'s platform note), but
`firmware/src/main.cpp` only ever defines `setup()`/`loop()` — the
Earle Philhower core's `setup1()`/`loop1()` hooks for the second core are
never defined, so everything (`hidController().task()`,
`serialController().update()`, the whole state machine, and inside
`IdleState` the sensor read + `solve_knob_pose()` + HID/telemetry) runs
serially on core0. Core1 sits idle.

## Why it matters

Referenced from `TODO/sensor-read-speed.md`: that TODO gets sensor staleness
down to about one I2C round-trip (<1ms) by moving the sensors to
Master-Controlled Mode, but explicitly punts on the fact that this budget
"creeps up as more work lands on the single core" — every tick, sensor
reads, the LM solve (`TODO/Performance.md`: ~7.5ms/call standalone, whole
loop at ~80Hz/12.5ms as of the last measurement), HID, and telemetry all
compete for the same core, so staleness and loop rate are coupled to
*everything* the firmware does, not just the solve itself. Splitting work
across cores would decouple them: e.g. core1 owns triggering/reading the
sensors and keeping the latest raw sample ready, while core0 runs the solve
against the freshest available sample and handles HID/telemetry/state
independently of I2C timing.

## Likely fix shape

- Move the sensor read loop (the part `TODO/sensor-read-speed.md` redesigns)
  onto `setup1()`/`loop1()`, publishing the latest raw reading to core0
  through the RP2040 SDK's inter-core FIFO or a lock-protected shared
  buffer — needs a decision on which, since the FIFO is lock-free but
  bounded/blocking on overflow, while a shared buffer needs an explicit
  lock (`EIGEN_NO_MALLOC`/no-heap-in-hot-path rules from `ARCHITECTURE.md`
  presumably extend to whatever synchronization primitive gets picked here).
  Solve/HID/telemetry stay on core0.
- Alternative split: keep sensor reads on core0 (where the rest of the
  I/O-driven state machine already lives) and move the CPU-bound solve
  itself to core1 instead, handing results back the same way. Less
  natural given `solve_knob_pose` currently runs synchronously inside
  `IdleState::update()` once per tick and expects a fresh sample.

## Open questions

- Which half of the pipeline moves to core1: sensor I/O (frees the solve
  from I2C timing) or the solve itself (frees I2C timing from solve
  duration)? These have different failure modes and different
  synchronization needs.
- What synchronization primitive to use between cores given the no-heap,
  no-blocking-alloc constraints already in force elsewhere in the hot path.
- Whether `BundleCalibrationController`'s serial-driven capture protocol
  (`TODO/calibration-mode-entry.md`) needs to run on a particular core, or
  is unaffected either way since it's not on the hot path.
- Not started: no design work has happened here yet beyond the cross-reference
  from `TODO/sensor-read-speed.md`.
