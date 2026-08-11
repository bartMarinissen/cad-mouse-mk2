# CAD Mouse MK2 (accuracy fork) — Architecture Notes

Working reference for navigating and reasoning about this codebase. Written from
reading the code directly (not from any doc that could drift), so re-verify
specifics (function names, line numbers) before relying on them if it's been a
while — see `design documentation/context.md` for the *why*, this file is the
*where/how*.

**The repo root `README.md` is the maintainer's personal file, off-limits to
edits from this doc's audit/consistency-keeping loop** — its "Current state
of the project" numbers are expected to lag what's described here. Don't
"fix" it to match this file; that's the maintainer's call, not a doc-sync
task.

## What this project is

A 6-DOF magnetic 3D mouse (fork of `sb-ocr/cad-mouse-mk2`): a knob embedded with
3 axially-polarized cylindrical magnets sits above 3 fixed Hall-effect sensors on
a PCB. Firmware infers the knob's full pose (X/Y/Z + pitch/roll/yaw) from the raw
magnetic field and reports it to the host as a USB HID multi-axis controller
(3Dconnexion-style), for CAD navigation.

The fork's whole reason to exist (per `design documentation/context.md`): the
upstream project mapped raw field strength to position with a naive linear
heuristic (still described in `firmware/README.md`, which is now stale). This
fork rips that out and replaces it with a real magnetic forward model +
Gauss-Newton solver, because field strength doesn't map linearly to distance,
and because per-magnet manufacturing tolerance causes "Phantom Tilt" (a magnet
5% stronger than its siblings looks, to naive math, like it moved closer). Fixing
that tolerance problem is what the in-progress bundle-calibration subsystem is
for.

## Three top-level areas

```
firmware/            PlatformIO/Arduino C++ — the actual product firmware
magnet_field_model/  Python (uv-managed) — offline field simulation + codegen
enclosure/           Fusion 360 / STEP / STL — mechanical design, not code
```

`platformio.ini` at repo root points PlatformIO at `firmware/{src,include,lib,test}`.
Board: Seeed XIAO RP2040 (dual-core Cortex-M0+, **no hardware FPU** — all float
math is software-emulated, which is why solver perf tuning matters: `-O2`,
`-fassociative-math`/`-freciprocal-math`/etc. relaxed-fp flags, `EIGEN_NO_MALLOC`
to force fixed-size Eigen types only, no heap allocation anywhere in the hot path).

## Firmware: control flow

`firmware/src/main.cpp` is the entry point. `setup()` brings up controllers in a
fixed order (HID → serial → input → LED → sensors → motion → telemetry), then
`stateMachine.changeState(&calibratingState)`. `loop()` calls
`hidController().task()`, then `serialController().update()` to assemble any
incoming serial line, then `stateMachine.update()` — a classic non-blocking
super-loop, no RTOS.

`Serial.begin()` happens unconditionally in `serialController().begin()`, not
gated on `Config::ENABLE_TELEMETRY` as it once was: the link now carries the
calibration protocol as well as telemetry. `TelemetryController` still decides
for itself whether to *print*.

### State machine

[`firmware/include/State.h`](firmware/include/State.h) — trivial interface:
`enter()/update()/exit()`.
[`firmware/include/StateMachine.h`](firmware/include/StateMachine.h) /
[`.cpp`](firmware/src/StateMachine.cpp) — holds one `currentState*`, calls
`exit()`/`enter()` on transition. States are **static members of StateMachine**
(singletons, no dynamic allocation — consistent with `EIGEN_NO_MALLOC`).

States, all in `firmware/{include,src}/states/`:

- **CalibratingState** — the tare step. Runs at boot and on the both-buttons-3s
  gesture from `IdleState`. Drives `SensorController::updateCalibration()` for
  `Config::ZERO_SAMPLES` (200) samples to compute a baseline field offset and a
  baseline pose, then hands off to `IdleState`. Also the *only* place serial
  commands are accepted: it announces `STATUS TARE_BEGIN` on entry, then honours
  `CAL_START` (enter `BundleState`) and `CAL_UPLOAD` (store a new calibration;
  `CAL_ABORT` leaves a run without storing, and is handled in `BundleState`)
  for the ~1.7s the tare lasts. Scoping both to a user-triggered window means
  neither can happen without someone physically holding the buttons.
  ⚠️ Still slated for replacement by a fuller "tare" step — see
  [`TODO/tare-and-calibration.md`](TODO/tare-and-calibration.md).
- **IdleState** — the main operating state. Reads sensors → runs the pose solve →
  maps to HID axes → sends HID report → publishes telemetry → watches for the
  calibration-request gesture and the inactivity timeout to `SleepState`.
- **SleepState** — LEDs off, waits for any button activity to return to `IdleState`.
- **ErrorState** — entered if `SensorController::begin()` fails at boot. LED
  spinner in red, otherwise inert (no recovery path).
- **BundleState** — hosts the serial-driven multi-step hardware calibration
  routine (see "Calibration subsystem" below). Entered only from
  `CalibratingState` on `CAL_START`; `enter()` kicks the run off directly,
  since the command that got it there was already consumed.

### Controllers (global singletons, reached through accessors in `Controllers.h`)

Declared in [`firmware/include/Controllers.h`](firmware/include/Controllers.h)
and defined in `Controllers.cpp`. This is the project's entire DI mechanism —
there is no container, no interfaces, states and controllers reach each other
through these accessors directly.

**No controller owns another, and that is the whole ownership rule.** Every
controller is reached through its `Controllers.h` accessor, by states and by
other controllers alike; a controller calling a sibling's accessor is the
sanctioned pattern, not a coupling smell. There is no dependency hierarchy to
respect and nothing to invert. What is *not* allowed is bypassing the
accessors — the `extern MotionController motionController;` that
`SensorController.cpp` once declared inline is the thing that was wrong, and
it is gone. Passing a controller in as an explicit parameter instead
(`BundleCalibrationController::update()` takes `SensorController&`) is also
fine where there's a reason, such as letting the callee control *when* the
read happens. This settles what used to be tracked as issues #4/#5 below.

| Controller | File | Responsibility |
|---|---|---|
| `InputController` | `controllers/InputController.*` | Debounces 2 buttons via AceButton; exposes `buttonBits()`, `takeActivity()` (edge-triggered, consumed on read), `takeCalibrationRequest()` (both buttons held 3s) |
| `LEDController` | `controllers/LEDController.*` | NeoPixel ring: owns one `std::variant`-held `AnimationBase` slot (`animations/`) plus power management; callers `set()` a `SolidAnimation`/`SpinnerAnimation`/`OffAnimation`/`PoseColorAnimation` and drive it with `update()` each tick |
| `SensorController` | `controllers/SensorController.*` | Owns the 3 TLx493D sensor objects, power-sequences them onto distinct I2C addresses at boot, `readUncorrected()`/`read_mT()` → 9 floats, applies gain + DC offset, runs the boot baseline calibration |
| `MotionController` | `controllers/MotionController.*` | The pose pipeline: raw field → solved pose → filtered/mapped HID axes (detail below) |
| `HIDController` | `controllers/HIDController.*` | Owns the USB HID descriptor + report state, dedupes unchanged reports before sending |
| `TelemetryController` | `controllers/TelemetryController.*` | Formats a big fixed-layout ASCII dashboard to Serial every 20 ticks when `Config::ENABLE_TELEMETRY` |
| `BundleCalibrationController` | `controllers/BundleCalibrationController.*` | Serial-driven state machine for the guided hardware calibration capture |
| `SerialController` | `controllers/SerialController.*` | Assembles incoming bytes into whole lines without blocking; `takeLine()` hands the active state one line per tick. Transport only — command meaning stays with the caller, and outbound prints still go to `Serial` directly |

### Sensor read timing

The three TLI493D-A2B6s are **not** in their power-on-reset Low Power Mode
(160 Hz, ~6.25ms worst-case staleness). `SensorController::setup_sensor()`
puts each into **Master-Controlled Mode** (`setPowerMode`) with
**trigger-on-read** (`setTrigger(TLx493D_ADC_ON_READ_AFTER_REG_05_e)`), so
every `getMagneticFieldAndTemperature()` both returns the previous
measurement and re-arms the next conversion. Reading the three back-to-back
therefore costs about one I2C round-trip of staleness rather than a fixed
conversion period. The PCB forces this: all three `SCL/INT` pins share one
net wired only to the MCU's plain `SCL`, so there is no `/INT` line to
synchronize on and it has to happen over the bus itself.

Two dependencies worth knowing:

- **This leans on I2C clock stretching** (the driver's default `CA=0`/`INT=1`
  config) to make a read block until its conversion finishes. Whether the
  RP2040 Arduino `Wire` implementation honours slave clock stretching is
  **unverified on real hardware** — the design assumes it.
- **Self-triggering only works if reads keep happening.** After a >100ms gap
  the armed conversion is stale, so `readUncorrected()` throws away one round
  purely to re-trigger before reading for real. Bus stays at 400kHz
  deliberately; Master-Controlled Mode doesn't need Fast Mode, and raising it
  risks signal integrity across three sensors on fixed 1.2kOhm pull-ups.

## The pose pipeline (the core of the project)

Entry point: `MotionController::compute()` in
[`MotionController.cpp`](firmware/src/controllers/MotionController.cpp), called
once per `IdleState::update()` tick.

```
SensorController::readRaw()          9 raw floats (3 sensors × xyz, mT)
        │
        ▼
MotionController::read_pose()        hot-started from last frame's (t, R)
        │
        ▼
solve_knob_pose()                    Levenberg-Marquardt Gauss-Newton, ≤10 iters
   (solve_pose.cpp)                  6x6 normal equations each iteration
        │  each iteration calls:
        ▼
ForwardModel::evaluate(t, R)         loops the 3 sensor/magnet pairs
   (forward_model.cpp)
        │  per pair:
        ▼
Sensor::evaluate(magnet, t, R)       transforms sensor into magnet-local frame,
   (sensor.cpp)                      composes the 3x6 Jacobian analytically
        │
        ▼
MagnetModel::evaluate(v_local)       cylindrical (r,z) decomposition of the
   (magnet_local_model.cpp)          local offset, handles r→0 singularity
        │
        ▼
BicubicField::evaluate(r, z)         Catmull-Rom bicubic interpolation over a
   (BicubicField.cpp)                51×91 precomputed grid, with ghost-point
        │                            linear extrapolation just past the edges
        ▼
BICUBIC_INTERPOLATION_TABLE          generated hex-float constants
   (magnet_model_table.cpp)          (see codegen pipeline below)
```

Output of the solve is `(position, rotation matrix)`; `extract_angles_robust()`
converts `R` to Euler angles with gimbal-lock handling. `compute()` then:
subtracts the calibration baseline pose, applies `Config::SIGN_AXIS` /
`Config::GAIN_T` / `Config::GAIN_R`, dead-zones, low-pass filters
(`Config::SMOOTH_TAU_S`), clamps to `Config::AXIS_LIMIT`, and writes the 6 HID
axis values.

**Why the Jacobian is hand-derived everywhere:** the solver needs a 9×6
Jacobian *every iteration*, and this runs on an FPU-less M0+, so it's all
closed-form analytic differentiation chained through each layer (no
finite-difference in the hot path). `firmware/test/test_jacobian.cpp` is a
Unity test suite that validates every one of these analytic Jacobians against
central finite differences — this is the safety net for that whole chain, run
it after touching any `evaluate()` in `magnet_model/`.

Geometry constants (sensor/magnet positions, triangle side length, 6mm z-standoff)
live in [`positions.h`](firmware/include/magnet_model/positions.h). Physical
constant: `#include "math3D.h"` for `Vec3`/`Mat3`/`skew_matrix` typedefs
(thin Eigen wrapper, `ArduinoEigenDense`).

### Codegen: Python → firmware table

`magnet_field_model/bicubic_table.py` (uv-managed Python env, `magpylib` for
ground-truth field simulation of a real 6×6mm N52 cylinder magnet) samples the
true field on the same 51×91 (r,z) grid defined by `NR`/`NZ` in
[`magnet_model_table.h`](firmware/include/magnet_model/magnet_model_table.h).
Run `magnet_field_model/generate_bicubic_table.py` to actually (re)write that
header and the hex-float C++ array in
[`magnet_model_table.cpp`](firmware/src/magnet_model/magnet_model_table.cpp) —
it imports the magnet/grid definitions from `bicubic_table.py` rather than
duplicating them. `field_approximation.ipynb` imports the same module to
inspect/visualize the table (field plots, dipole-approximation comparison,
interpolation error); it no longer writes the firmware files itself.
**This file is generated, not hand-written** — if the magnet spec, grid bounds
(`BICUBIC_ORIGIN`/`BICUBIC_FAR` in `magnet_model_table.h`), or grid resolution
change, they need to change in both the notebook and the header in lockstep, by
hand. There's no build-time codegen step; it's a manual copy-paste pipeline.

## Calibration subsystem — two independent layers

**1. Boot baseline (`CalibratingState`, always runs):** zeroes out ambient
field offset and establishes a rest pose by averaging 200 samples. Fast,
automatic, no user interaction beyond waiting. This is *not* what fixes Phantom
Tilt — it just establishes a reference point.

**2. Bundle calibration (`BundleState` / `BundleCalibrationController`, new,
in progress):** a guided multi-step capture routine intended to solve the
Phantom Tilt problem via bundle adjustment. Step sequence:
`STATIONARY → FLAT_CIRCLE → PITCH → ROLL → TWIST → HEAVE → RANDOM → COMPLETE`.
Each step is a little sub-state-machine (`CalibPhase`:
`WAIT_FOR_START_BTN → COUNTDOWN → RECORDING → WAIT_FOR_ACK`) that streams
`CAL_FRAME`/`CAL_STATE` lines over Serial to a host script,
[`bundle_callibration.py`](magnet_field_model/bundle_callibration.py), which
ACKs frame counts back over the same link. A successful ACK moves straight on
to the next step's `WAIT_FOR_START_BTN` (or `AWAITING_UPLOAD` after the last
one) with no confirmation step in between -- there is no way to go back and
redo a step, so there is nothing for a pause to do there except cost the user
an extra button press.

**Status: capture, fit, and delivery all work.** The
Python side fits 45 shared parameters (per-magnet position, tilt and strength;
per-sensor gain matrix and DC offset) jointly with one free 6-DOF pose per
captured frame, and takes the field residual from ~1.8% at nominal geometry
down to ~0.33%, in about a second. See
[`magnet_field_model/README.md`](magnet_field_model/README.md) for the
architecture, the load-bearing frame conventions, and the measured numbers.

Two things are worth knowing before touching it:

- The magnet position reference is the magnet's **bottom face**, not its
  centre — that is what `positions.h` means and what the notebook baked into
  the bicubic table. magpylib positions cylinders by their centre.
- The raw capture path streams `readUncorrected()`, so the sensors read the
  **opposite sign** to the modelled field — matching `Config::magnet_gains`
  (`{-0.96, -1.2, -0.98}`). The fit attributes that flip to magnet polarity,
  so fitted gains read near `+I` while the *exported* ones land near `-I`.
- Sensor gain scale and magnet strength are not separable (only ~1%
  cross-talk distinguishes them), so the fit picks a gauge: `det(G) = 1`, with
  magnet strength carrying the scale. The reported strengths are an
  attribution, not a measurement — the fit's information-gain column says so.

### Persistence

Fitted calibrations reach the firmware on their own now. The whole fitted
parameter set is one struct,
[`CalibrationParams`](firmware/include/CalibrationParams.h), and
[`CalibrationStorage`](firmware/include/CalibrationStorage.h) persists it to
LittleFS as `/calibration.bin`: 312 bytes of `CMK2` magic, a version, the
struct's own 300 bytes, and a CRC-32 over everything before it.

Reading one is a `memcpy` — check magic, version and CRC, copy the payload into
the struct, then range-check it. That works because `CalibrationParams` is
**plain arrays rather than Eigen types**, which buys three things at once:
`memcpy` into it is defined behaviour (`Eigen::Matrix` has a user-provided copy
constructor, so a struct of `Mat3`/`Vec3` is not trivially copyable), `sizeof ==
300` is a language guarantee instead of an observation, and the layout can be
**row-major** to match numpy rather than the column-major Eigen would impose —
a transposed gain matrix passes both the CRC and every plausibility check, so
that ordering is worth pinning down. Consumers convert with `toMat3()`/
`toVec3()` at construction, where they already copied the values out.

`board_build.filesystem_size` in `platformio.ini` carves out the partition —
without it the filesystem is 0MB and nothing mounts.

`SensorController` and `MotionController` are constructed from that struct and
hold their calibration-derived state `const`, which is why they are no longer
static-init globals — each is reached through its own Meyers-singleton
accessor, `sensorController()`/`motionController()`, built on first call from
`resolveCalibration()` (see [`Controllers.h`](firmware/include/Controllers.h)).
Every other controller is trivially default-constructible and has no such
dependency, so it gets a plain accessor over a static-init global instead —
same construction as before, just reached through a function for uniform call
syntax across all eight controllers. `resolveCalibration()` in
`Controllers.cpp` reads and validates the stored blob, falling back to
`Config::defaultCalibration()` — and printing which check failed — whenever
there is nothing stored or what is stored does not survive.

Validation is deliberately all-or-nothing, and that follows from the gauge
note above: `sensor_gain` and `magnet_strength_mT` are the same degree of
freedom under `det(G) = 1`, so a blob is accepted whole or rejected whole.
Nothing repairs or defaults an individual field while keeping the rest, which
would silently manufacture a mixed-gauge calibration worse than either input.
Past the CRC the checks are loose corruption tripwires — finite floats,
non-singular gain, sane magnitudes — sized to admit both that gauge and
`defaultCalibration()`'s different gain-carries-scale one. Magnet strength is
bounded on magnitude only: a reversed-polarity magnet legitimately fits to a
negative `Br`, and `MagnetModel` handles the sign correctly.

Two routes in, the same bytes either way:

- **`bundle_callibration.py --emit-bin`** writes the blob into `firmware/data/`;
  `pio run -t uploadfs` flashes it as a filesystem image.
- **`bundle_callibration.py --write-serial`** pushes it to a running device as
  one hex-encoded `CAL_UPLOAD` line. Hex rather than raw binary so it rides the
  same newline-delimited convention as everything else on this link — raw bytes
  would contain `0x0A` and truncate the read. The device validates, stores, and
  reboots, since the controllers hold their calibration `const` from boot and
  cannot adopt a new one in place.

The host never restates that layout. `calibration/firmware_struct.py` extracts
the `CalibrationParams` declaration from the header and feeds it to cffi in ABI
mode (no compiler at runtime), so `format_binary()` fills the struct by field
name and `sizeof` comes from the declaration rather than a literal. Field order
therefore lives in exactly one place. This matters because a wrong order is
silent: it still yields a valid blob with a valid CRC, and a *reordered* struct
is even the same 300 bytes — so `tests/test_export.py` pins the field offsets,
not just the size.

`--emit-cpp` stays for reading and diffing the numbers, now as a nested-brace
aggregate initializer — possible since the struct became plain data, and the
reason the same test can compile that snippet against the extracted
declaration and diff its bytes against `format_binary()`, checking cffi's ABI
model against a real compiler.

**The knob holds after capture.** Finishing the last pose moves it to
`AWAITING_UPLOAD` rather than back to `IdleState`: the host is about to solve
and hand a calibration straight back, and making the user walk the knob into
calibration mode again to receive it was the wrong shape. `CAL_UPLOAD` is
therefore accepted in `BundleState` as well as during tare.

**Storing is still a decision, not the only exit.** Capture and fit never touch
flash by themselves, and `CAL_ABORT` returns to idle without writing — also the
only escape from a run whose host went away, since `WAIT_FOR_ACK` otherwise
times out back to `WAIT_FOR_START_BTN` forever. The host sends it when
`--write-serial` was not passed (nothing is coming back, so the knob should not
be left parked) or when the user declines the write prompt. A knob-side escape
still does not exist; see
[`TODO/calibration-mode-entry.md`](TODO/calibration-mode-entry.md).

Two caveats on the exported numbers:

- `MagnetModel` now carries a `magnet_strength_mT` — a real polarization value
  in mT, not a dimensionless multiplier — and applies it in `evaluate()`,
  scaling the six cylindrical quantities before the x/y decomposition (exact,
  since the field enters linearly downstream). The ratio it actually multiplies
  by is `magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT`
  (`magnet_model/magnet_model_table.h`): the polarization
  `magnet_field_model/bicubic_table.py` generated the table at, currently
  1000 mT and arbitrary — any magnet's real Br divided by it gives the right
  scale regardless of what that reference happens to be.
- The fitted values are **effective parameters for this model, not measured
  physics.** `ForwardModel::evaluate()` pairs sensor *i* with magnet *i* only,
  ignoring the other two magnets ~28.58mm away — worth 1.7–4.5% of the field.
  The Python fit is deliberately pinned to that same single-magnet model
  (`calibration/bundle_geometry.py`'s `SENSOR_MAGNET_COUPLING = PAIRED_ONLY`)
  so the two agree; fitting the complete model instead and exporting *that*
  leaves the cross term uncompensated and measures 3.6% worse. Cross-magnet
  field therefore ends up absorbed into gain/offset/strength, exactly as the
  old hand-tuned `Config::magnet_gains` absorbed it. See
  `TODO/cross-magnet-interference.md` for the measurements and the one-constant
  path back.
- `Sensor` no longer carries a gain; correction happens entirely in
  `SensorController::read_mT()` (issue #6 below).

## Quick file index

| Looking for... | Look in |
|---|---|
| Pins, gains, dead zones, LED colors, timing | `firmware/include/Config.h` |
| Board/build flags, lib deps | `platformio.ini` |
| Sensor geometry / knob geometry constants | `firmware/include/magnet_model/positions.h` |
| The Gauss-Newton solver loop | `firmware/src/magnet_model/solve_pose.cpp` |
| Per-sensor analytic Jacobian | `firmware/src/magnet_model/sensor.cpp` |
| Cylindrical magnet field model + r→0 handling | `firmware/src/magnet_model/magnet_local_model.cpp` |
| Bicubic grid interpolation | `firmware/src/magnet_model/BicubicField.cpp` |
| Generated field table (don't hand-edit) | `firmware/src/magnet_model/magnet_model_table.cpp` |
| Field table generator (run this to regenerate) | `magnet_field_model/generate_bicubic_table.py` |
| Field table magnet/grid definition (source of truth) | `magnet_field_model/bicubic_table.py` |
| Field table inspection/visualization (does not write it) | `magnet_field_model/field_approximation.ipynb` |
| HID descriptor / report format | `firmware/src/controllers/HIDController.cpp` |
| Serial diagnostics dashboard | `firmware/src/controllers/TelemetryController.cpp` |
| Jacobian correctness tests (finite-difference) | `firmware/test/test_jacobian.cpp` |
| New guided calibration flow | `firmware/src/controllers/BundleCalibrationController.cpp`, `firmware/src/states/BundleState.cpp`, `magnet_field_model/bundle_callibration.py` |
| Project intent / hardware spec | `design documentation/context.md` |
| Tracked follow-up work | `TODO/` |

## Issues / architecture drift spotted while reading

Tracked as of the initial read-through. Status as of the follow-up pass:

1. **`CalibratingState` → `BundleState` transition looks accidental** —
   **fixed.** The any-button-activity jump is gone; entry now requires the
   host to send `CAL_START` during the tare window, so a stray tap at boot
   can no longer drop the knob into a serial protocol with nobody on the
   other end. See
   [`TODO/calibration-mode-entry.md`](TODO/calibration-mode-entry.md) for the
   reasoning and for the knob-side *exit* gap that remains.

2. **`BundleState` passes the wrong value as "button bits"** — still open.
   `BundleState::update()` hands `takeActivity()` (a `bool`) where a
   `buttonBits()` bitmask belongs. Tracked in
   [`TODO/calibration-mode-entry.md`](TODO/calibration-mode-entry.md), since
   the fix depends on the entry/exit gesture design.

   The wider boot-calibration redesign these two were originally folded into
   is still tracked separately in
   [`TODO/tare-and-calibration.md`](TODO/tare-and-calibration.md).

3. `solve_knob_pose()` not writing its Jacobian out-param — **fixed.**

4. **Circular coupling between `SensorController` and `MotionController`**
   and 5. **the forward model's state living outside any controller** — both
   **resolved.** The forward model is now a `const ForwardModel` member of
   `MotionController`, built from `CalibrationParams`. The
   `SensorController` → `MotionController` call in `updateCalibration()`
   stays, and is fine: it goes through the `motionController()` accessor like
   everything else, which is the project's ownership rule (see "Controllers"
   above), not a violation of it. The raw `extern` that *was* the violation
   is gone.

6. **`Config::magnet_gains` can't express what the math supports** —
   **resolved.** Sensor gain/skew correction is owned entirely by
   `SensorController::read_mT()`, outside the motion system; `Sensor` no
   longer carries a gain at all, and the coefficients come from
   `CalibrationParams` rather than `Config` scalars. `calibration/export.py`
   emits real polarization in mT (`firmware_magnet_strength_mT()`), closing
   the last gap this issue tracked.

7. **`rcond` is a hardcoded stub** (deliberately — computing it via SVD every
   frame is too expensive on this FPU-less MCU, not an oversight) and
   8. **`TelemetryController::publish()`'s signature keeps growing** — merged
   into one telemetry rework, tracked in
   [`TODO/telemetry-rework.md`](TODO/telemetry-rework.md).

9. Unwired linker scripts (`custom_memmap.ld`, `full_custom_memmap.ld`) —
   **fixed** by removing the vestigial files.

10. **`firmware/README.md` documents the old, replaced motion model** — left
    open, tracked in [`TODO/readme-refresh.md`](TODO/readme-refresh.md).
