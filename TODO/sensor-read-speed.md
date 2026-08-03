# Switch sensors to Master-Controlled Mode for faster, fresher reads

`SensorController` never calls `setPowerMode()`, so all three TLI493D-A2B6
sensors sit in the power-on-reset default: Low Power Mode. `MOD2.PRD` resets
to `0B` ("fast" per the register description) — inferred, not spec-confirmed
against an explicit Hz table, to mean the higher of the datasheet's two
listed rates: 160 Hz, i.e. a worst-case ~6.25ms staleness today regardless of
poll rate. The PCB is fixed/bought: all three sensors' `SCL/INT` pins share
one net wired only to the MCU's plain I2C `SCL` — no `/INT` GPIO, and one
wouldn't help anyway with three sensors converting independently on the same
line. So synchronization has to happen over the I2C bus itself.

## Decision

Move all three sensors to Master-Controlled Mode (`MOD1.MODE = 01B`).
Trigger each conversion with trigger-bits `100B` ("trigger after register
05H") appended to the same read that fetches the previous measurement — one
transaction both retrieves data and re-arms the next conversion. This is
compatible with the driver's existing `CA=0`/`INT=1` config (clock
stretching enabled, /INT disabled, already handling synchronization safely
across the shared bus); only trigger-bits `010B`/`011B` (trigger-before-read)
are excluded with that config, not `100B`. Expected staleness is about one
round-trip across the three sensors — under 1ms, a >6x improvement on
today's ~6.25ms, and accepted as fine for now. It'll creep up as more work
lands on the single core, but that's `TODO/multicore.md`'s problem, not
this one.

## Round shape

1. Read sensor 1, sensor 2, sensor 3 in turn — full register block
   (00H–06H), trigger-bits `100B` on each read. Every read both harvests
   the prior result and re-arms that sensor's next conversion.
2. Check the parity/status bits included in that block; discard/flag
   anything invalid (expected on the very first round after switching
   modes, before any conversion has run yet).
3. Repeat continuously — clock stretching covers the case where a
   conversion isn't quite done by the time its turn comes back around (in
   steady state it should always be done: 3 reads at 400kHz take ~600us,
   longer than the ~175-240us typical conversion time).

## Open questions

- Does the Infineon Arduino library expose the per-frame trigger bits, or
  does this need a raw register write? `getMagneticFieldAndTemperature()`
  may not support it.
- Does the RP2040 Arduino `Wire` library actually honor I2C slave clock
  stretching? Unverified on real hardware, and this design depends on it.
- Keep I2C clock at 400kHz — Master-Controlled Mode doesn't need Fast
  Mode's >=800kHz, and raising it risks signal integrity with 3 sensors
  sharing the bus's fixed 1.2kOhm pull-ups.
