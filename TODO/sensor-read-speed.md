# Switch sensors to Master-Controlled Mode for faster, fresher reads

Originated from ARCHITECTURE.md issue #11.

## Current state

The three TLI493D-A2B6 sensors are left in their power-on-reset default:
Low Power Mode. `SensorController::begin()` never calls `setPowerMode()`,
so each sensor free-runs its own internal cyclic conversion at 10 Hz or
160 Hz, and `readUncorrected()` just polls whatever happens to be in the
registers at call time. That update rate is a hard ceiling on read
freshness no matter how fast the main loop calls it.

The PCB (`pcbs/src/sensor_board.sch`) is a bought/fixed design: all three
sensors' `SCL/INT` pins are tied to one shared net, wired only to the MCU's
plain I2C `SCL` pin. There is no dedicated GPIO for `/INT`, and even if
there were, sharing it across three independently-converting sensors would
be unusable. So any synchronization has to happen over the shared I2C bus
itself, not via an interrupt line.

## Decision

Move all three sensors to **Master-Controlled Mode** (`MOD1.MODE = 01B`),
and drive conversions explicitly from the microcontroller using **staggered
trigger writes followed by a separate read pass** — not a scheme where
every read also re-arms the next conversion. The latter maximizes
throughput but leaves each value stale for most of a read cycle (triggered
right after the previous read, then left sitting until the next visit to
that sensor); staggering the trigger separately keeps each value only
about one conversion time old. Expected round latency is sub-1ms, which is
acceptable for now — it will creep up as more work piles onto the single
core, but the fix for that is the separate, already-reserved
`TODO/multicore.md` effort, not this one.

Key register-level facts this relies on (from the TLI493D-A2B6 User
Manual, not just the datasheet):

- The driver's existing default config already sets `MOD1.CA = 0B` and
  `MOD1.INT = 1B` — i.e. **clock stretching enabled, /INT pulses disabled**.
  This is correct for the shared-bus setup here and should **not** be
  changed: a sensor only ever pulls SCL low while it itself is being
  addressed and its own conversion isn't done, so there's no cross-sensor
  collision risk.
- Every I2C frame (read or write) carries 3 "trigger bits" as the top bits
  of the register-address byte:
  - `001B` on a **write** frame = trigger a conversion after the write
    finishes. Use this for the dedicated trigger step.
  - `000B` = no trigger. Use this on the data-fetch reads so reading
    doesn't silently re-arm the next conversion.
  - `010B`/`011B` (trigger-before-read) must **not** be combined with the
    clock-stretching config above (explicitly excluded in the register
    documentation) — avoid these.
- The data-fetch read should span the full measurement+diagnostic register
  block in one transaction, which includes the parity bit and status bits
  (conversion-done / valid-data flags, frame counter). Use these to
  validate that a given read is complete/fresh instead of relying on fixed
  timing windows.

## Round shape

1. Trigger sensor 1, trigger sensor 2, trigger sensor 3 (each a short
   write with trigger-bits `001B`, ~3 bytes / ~67.5us at 400 kHz).
2. Read sensor 1, sensor 2, sensor 3 (full register block, trigger-bits
   `000B`, ~200us each at 400 kHz). Clock stretching covers the case
   where a conversion isn't quite done yet.
3. Check the parity/status bits on each read; discard/flag anything
   invalid (expected on the very first round right after switching modes).
4. Repeat at whatever cadence the application wants — this loop is no
   longer forced to run flat-out just because a read happens to also
   double as a trigger.

## Open questions

- Whether the Infineon Arduino library
  (`infineon/XENSIV 3D Magnetic Sensor TLx493D`) exposes control over the
  per-frame trigger bits, or whether a lower-level/raw register write is
  needed to set them — the high-level `getMagneticFieldAndTemperature()`
  call may not support this.
- Whether the RP2040 Arduino `Wire` library actually honors I2C slave
  clock stretching (most hardware I2C peripherals do, but this whole
  design depends on it and it hasn't been verified on real hardware yet).
- Whether the I2C clock should stay at 400 kHz. Master-Controlled Mode
  doesn't need the >=800 kHz that Fast Mode requires, and raising it risks
  signal integrity given the fixed 1.2 kOhm pull-ups and three sensors
  sharing the bus (the datasheet warns about capacitive load above
  400 kHz) — leaning toward leaving it as-is unless there's a concrete
  reason to change it.
