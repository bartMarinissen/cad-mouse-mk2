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
*everything* the firmware does, not just the solve itself.

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
  parity says which buffer holds the latest complete reading. Core1 locks
  `L_sensor`, reads it, unlocks, then reads the buffer that parity selects.
- **Pose channel (core1 → core0).** Same shape, reversed: core1 writes the
  solved pose into the off-buffer, then locks/increments/unlocks `L_pose`;
  core0 locks/reads/unlocks `L_pose` and reads the selected buffer.

This is safe without ever locking the (much bigger) payload buffers
themselves, only the counters: the writer on each channel never touches the
buffer its own last-published counter value points at, so a reader copying
out of that buffer can't collide with a write in progress. Only the
counter's read-modify-write needs to be atomic across cores, which a single
RP2040 hardware spinlock per channel (`hardware/sync.h`:
`spin_lock_claim`/`spin_lock_blocking`/`spin_unlock`, available under
arduino-pico, no allocation) gives directly — the struct copy itself doesn't
need any lock. Same idea as a seqlock, simplified: instead of a reader
detecting a torn read and retrying, the writer's strict buffer alternation
is what prevents the tear from being possible in the first place, *given
the writer can't lap the reader* — see open questions below.

Running the solve continuously rather than once per core0 tick also
simplifies hot-start: `MotionController::last_R`/the translation hot-start
becomes core1's own loop-local state between iterations, no longer
something that has to be threaded across the core boundary every tick.

## Open questions

- **Torn-read assumption.** The scheme is only safe if the writer can't
  complete a full extra write (lap the reader) while the reader is still
  mid-copy of the buffer it just selected. Given the size mismatch — a
  spinlock-guarded counter increment against copying a handful of floats —
  this should hold by a wide margin on both channels today, but it's an
  assumption, not enforced. Worth a documented bound, or a cheap post-copy
  re-check (re-read the counter after copying, retry if it moved) if this
  is ever the source of a hard-to-reproduce bug.
- **Whether "solve done" needs a notification channel beyond `L_pose`
  itself.** Core0 can already tell "is this a new pose" for free by
  remembering the `L_pose` value it last consumed and comparing after each
  locked read — no separate signal needed for *that* question. Where a
  dedicated notification could still help is skipping the spinlock+read
  entirely on ticks where nothing changed, using the RP2040 inter-core FIFO
  (`hardware/sync.h`'s `multicore_fifo_*`) purely by polling
  (`multicore_fifo_rvalid()`, no `irq_set_exclusive_handler` registered):
  core1 does a non-blocking push (`multicore_fifo_push_timeout_us(x, 0)`,
  dropping on a full FIFO rather than blocking the solver) whenever it
  publishes; core0 drains whatever's there each tick and treats "got
  anything" as the cue to bother taking the `L_pose` lock at all. Given how
  cheap a spinlock read actually is on this chip, this may be solving a
  problem that doesn't exist yet.
- **Telemetry is a separate, harder version of the same problem.**
  `Statistics` (`MotionController.h`) is far bigger than a pose —
  `last_jacobian` alone is a 9x6 float matrix — and `TelemetryController`
  only wants it every 20 ticks, not every solve. Doubling the full
  `Statistics` struct the way the pose gets doubled is the simplest option
  but doubles a chunk of RAM that's already the binding constraint
  (`TODO/Performance.md`). Worth deciding whether telemetry rides the same
  buffer/counter as the pose (one combined struct) or gets its own
  lower-frequency channel, and whether the cross-core copy needs everything
  `Statistics` currently tracks or a smaller subset.
- **Which primitive for the lock.** RP2040 hardware spinlocks
  (`hardware/sync.h`) are the natural fit for a critical section that's just
  "increment a counter" — confirm arduino-pico exposes them directly rather
  than only the heavier, allocation-touching `mutex_t`.
- **How this interacts with calibration.** Tare (`CalibratingState`,
  `TODO/tare-and-calibration.md`) and bundle calibration (`BundleState`)
  both drive sensor reads and, for tare, a pose solve directly today,
  outside the normal `IdleState` tick. If core1 solves continuously
  regardless of state, does it just keep solving harmlessly during
  calibration, or does something need to pause/reset it? This also runs
  straight into `TODO/controller-ownership.md`'s Problem 1 —
  `SensorController::updateCalibration()` calling directly into
  `MotionController::read_pose()`/`set_base_pose()` doesn't have an obvious
  cross-core equivalent, so this plan likely can't be finalized independently
  of that TODO and the tare redesign.
- **Whether core0 needs to own sensor I/O at all**, versus core1 calling
  into `SensorController` directly with its own locking around the raw I2C
  read. Written up above as "core0 owns sensor I/O" per the design as
  described; worth confirming that split is intentional rather than
  incidental.

## Not started

No implementation exists yet. This is a design sketch — buffer struct
layout, `hardware/sync.h` availability under arduino-pico, and where
`setup1()`/`loop1()` and the two channels get declared in
`main.cpp`/`Controllers.h` are all still open.
