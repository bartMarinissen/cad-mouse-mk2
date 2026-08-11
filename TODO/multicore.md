# Put the RP2040's second core to work

The Seeed XIAO RP2040 is dual-core (`ARCHITECTURE.md`'s platform note), but
`firmware/src/main.cpp` only ever defines `setup()`/`loop()` — the
Earle Philhower core's `setup1()`/`loop1()` hooks for the second core are
never defined, so everything (`hidController().task()`,
`serialController().update()`, the whole state machine, and inside
`IdleState` the sensor read + `solve_knob_pose()` + HID/telemetry) runs
serially on core0. Core1 sits idle.

## Why it matters

Sensor staleness and loop rate are currently coupled to *everything* the
firmware does. The sensors are already in Master-Controlled Mode with
trigger-on-read, so a read is fresh to within about one I2C round-trip — but
that only holds if reads keep happening promptly, and today each read waits
its turn behind the LM solve (`TODO/Performance.md`: ~7.5ms/call standalone,
whole loop at ~80Hz/12.5ms as of the last measurement), HID, and telemetry on
the one core. Moving the solve off core0 decouples the two: the sensor loop
can cycle as fast as I2C allows, and the solver can run flat out instead of
once per tick.

## Decision

Core1 runs `solve_knob_pose()` continuously, back-to-back, independent of
core0's tick rate — not once per `IdleState::update()` call the way it runs
today. Core0's superloop keeps its current shape (state machine, HID,
serial, LED) but owns sensor I/O as a tight read-and-publish loop; the solve
itself moves entirely to core1.

Two channels cross the core boundary, both the same shape: a double buffer
plus one lock-guarded monotonic counter.

- **Sensor channel (core0 → core1).** Core0 writes each completed sensor
  reading into whichever of two buffers isn't the currently-published one,
  then locks a counter `L_sensor`, increments it, unlocks. `L_sensor`'s
  parity says which buffer holds the latest complete reading.
- **Pose channel (core1 → core0).** Same shape, reversed: core1 writes the
  solved pose into the off-buffer, then locks/increments/unlocks `L_pose`.

**The reader holds the lock across its copy**, not just across reading the
counter: lock, read the counter, copy the buffer its parity selects, unlock.
That is what makes the scheme airtight rather than merely probable. While the
reader holds the lock the writer cannot increment, so the published parity
cannot change mid-copy, so the buffer the writer is filling
(`1 - parity`) stays disjoint from the buffer the reader is copying
(`parity`) for the whole copy. The writer never blocks on the payload — only,
briefly, on the counter increment at the end of its own write, and only when
a read is in flight at that exact moment.

The payload buffers themselves are therefore never locked, and no assumption
about relative speeds is needed: correctness comes from the lock interval,
not from the writer being too slow to lap the reader.

A single RP2040 hardware spinlock per channel (`hardware/sync.h`:
`spin_lock_claim`/`spin_lock_blocking`/`spin_unlock`) is the right primitive —
no allocation, and the critical section is a counter read plus a handful of
floats. Note `spin_lock_blocking()` disables interrupts on the holding core
for the duration, so the copy staying small is load-bearing; a pose or a
9-float sensor frame is comfortably inside that budget, a `Statistics` struct
would not be (see telemetry below).

**No separate "solve done" notification is needed.** `L_pose` already carries
it: core0 remembers the value it last consumed and compares after each locked
read. That is freshness detection for free, and it matters beyond saving
work — `Config::SMOOTH_TAU_S`'s low-pass is dt-based and assumes each tick's
pose is a genuinely new sample, so silently re-consuming an unchanged pose
would corrupt the filtered output, not merely waste a copy. An inter-core
FIFO doorbell would add nothing the counter comparison doesn't already give.

Running the solve continuously rather than once per core0 tick also
simplifies hot-start: `MotionController::last_R` and the translation
hot-start become core1's own loop-local state between iterations, no longer
something threaded across the core boundary every tick.

## Open questions

- **Telemetry is a separate, harder version of the same problem.** It cannot
  ride the pose channel as-is: `Statistics` (`MotionController.h`) is far
  bigger than a pose, and the spinlock-held copy above only stays cheap while
  it's small. `TelemetryController` also only wants it every 20 ticks, not
  every solve, so the two have different natural rates as well as different
  sizes. Worth deciding whether it gets its own lower-frequency channel
  (possibly with a different discipline, since a torn/stale telemetry frame is
  cosmetic where a torn pose is not) and which fields actually need to cross.
- **`Statistics::last_jacobian` should probably not exist by then.** It is a
  9x6 float matrix (216 bytes) that `MotionController::compute()` still hands
  to `solve_knob_pose()` as an out-param on every solve
  (`MotionController.cpp:127`, gated on `Config::statistics`), but nothing
  reads it — the only consumer is the commented-out `jacobiSvd()` line at
  `IdleState.cpp:51`. (The `avg_jacobian` EMA was deleted in the third
  optimization pass; this is the other one, still written.) Dropping `rcond`
  per `TODO/telemetry-rework.md`'s Issue A removes that last would-be
  consumer, at which point the member and the per-solve write can both go —
  which is worth doing *before* sizing the telemetry channel, so 216 bytes of
  dead weight doesn't get designed into the cross-core copy.
- **Which primitive for the lock.** Confirm arduino-pico exposes
  `hardware/sync.h`'s hardware spinlocks directly rather than only the
  heavier, allocation-touching `mutex_t`.
- **How this interacts with calibration.** Tare (`CalibratingState`,
  `TODO/tare-and-calibration.md`) and bundle calibration (`BundleState`) both
  drive sensor reads outside the normal `IdleState` tick, and tare
  additionally drives a pose solve directly, per-sample, from
  `SensorController::updateCalibration()` — deliberately with a *fixed*
  identity starting guess rather than the hot-start state, which a
  free-running core1 solver does not naturally provide. So tare can't simply
  consume core1's pose stream; it needs either its own synchronous solve or a
  way to ask core1 for a cold-started one. Likely wants settling alongside the
  tare redesign rather than before it.
- **Whether core0 needs to own sensor I/O at all**, versus core1 calling
  `SensorController::read_mT()` itself and skipping the sensor channel
  entirely. Written up above as "core0 owns sensor I/O" per the design as
  described; worth confirming that split is intentional. The tradeoff is that
  core1 would then block on I2C inside its solve loop, which is exactly the
  coupling this change is meant to remove.

## Not started

No implementation exists yet. This is a design sketch — buffer struct layout,
`hardware/sync.h` availability under arduino-pico, and where
`setup1()`/`loop1()` and the two channels get declared in
`main.cpp`/`Controllers.h` are all still open.
