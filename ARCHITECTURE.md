# CAD Mouse MK2 (accuracy fork) — Architecture Notes

Working reference for navigating and reasoning about this codebase. Written from
reading the code directly (not from any doc that could drift), so re-verify
specifics (function names, line numbers) before relying on them if it's been a
while — see `design documentation/context.md` for the *why*, this file is the
*where/how*.

**The repo root `README.md` is the maintainer's personal file, off-limits to
edits from this doc's audit/consistency-keeping loop** — it may be out of date.
That is not your problem to solve. And be careful with relying on it to be up to date.

## What this project is

A 6-DOF magnetic 3D mouse (fork of `sb-ocr/cad-mouse-mk2`): a knob embedded with
3 axially-polarized cylindrical magnets sits above 3 fixed Hall-effect sensors on
a PCB. Firmware infers the knob's full pose (X/Y/Z + pitch/roll/yaw) from the raw
magnetic field and reports it to the host as a USB HID multi-axis controller
(3Dconnexion-style), for CAD navigation.

The upstream project mapped raw field strength to position with a naive linear
heuristic. This fork rips that out and replaces it with a real magnetic
forward model + Gauss-Newton solver, because field strength doesn't map
linearly to distance.

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

### State machine

[`firmware/include/State.h`](firmware/include/State.h) — trivial interface:
`enter()/update()/exit()`.
[`firmware/include/StateMachine.h`](firmware/include/StateMachine.h) /
[`.cpp`](firmware/src/StateMachine.cpp) — holds one `currentState*`, calls
`exit()`/`enter()` on transition. States are **static members of StateMachine**
(singletons, no dynamic allocation).

States, all in `firmware/{include,src}/states/`:

- **CalibratingState** — the tare step. Runs at boot and on the both-buttons-3s
  gesture from `IdleState`. Drives `SensorController::updateCalibration()` for
  `Config::ZERO_SAMPLES` (200) samples to compute a baseline field offset and a
  baseline pose, then hands off to `IdleState`. Also the *only* place serial
  commands are accepted: it announces `STATUS TARE_BEGIN` on entry, then honours
  `CAL_START` (enter `BundleState`) and `CAL_UPLOAD` (store a new calibration;
  `CAL_ABORT` leaves a run without storing, and is handled in `BundleState`)
  for the ~1.7s the tare lasts.
  [`TODO/tare-and-calibration.md`](TODO/tare-and-calibration.md).
- **IdleState** — the main operating state. Reads sensors → runs the pose solve →
  maps to HID axes → sends HID report → publishes telemetry → watches for the
  calibration-request gesture and the inactivity timeout to `SleepState`.
- **SleepState** — LEDs off, waits for any button activity to return to `IdleState`.
- **ErrorState** — entered if `SensorController::begin()` fails at boot. LED
  spinner in red, otherwise inert (no recovery path).
- **BundleState** — This is where actual callibration happens. We use bundle
  callibration and taring was called callibration. Hence the bad name. 
  This hosts the serial-driven multi-step hardware calibration
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
accessors.

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

worth knowing: **Self-triggering only works if reads keep happening.** After a >100ms gap
  the armed conversion is stale, so `readUncorrected()` throws away one round
  purely to re-trigger before reading for real. Bus stays at 400kHz
  deliberately; Master-Controlled Mode doesn't need Fast Mode, and raising it
  risks signal integrity across three sensors on fixed 1.2kOhm pull-ups.

The derivation behind this — datasheet rates, why the shared `SCL/INT` net
rules out interrupt-driven sync, why trigger-bits `100B` specifically — is
archived in
[`TODO/resolved/sensor-read-speed.md`](TODO/resolved/sensor-read-speed.md).

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
ForwardModel::evaluate(t, R)         places each magnet in the world once,
   (forward_model.cpp)               then loops the 3 sensors
        │  per sensor, all 3 magnets:
        ▼
VirtualSensor::evaluate(...)         paired magnet via the interpolated field
   (virtual_sensor.cpp)              below; the other 2 via dipole_field().
        │                            Sums both, then composes the 3x6
        │                            Jacobian analytically once.
        ▼
MagnetModel::evaluate(v_local)       cylindrical (r,z) decomposition of the
   (magnet_local_model.cpp)          local offset, handles r→0 singularity
        │                            (paired magnet only)
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
**The `magnet_model_table.cpp` file is generated, not hand-written** — if the magnet spec, grid bounds
(`BICUBIC_ORIGIN`/`BICUBIC_FAR` in `magnet_model_table.h`), or grid resolution
change, they need to change in both the notebook and the header in lockstep, by
hand. There's no build-time codegen step; it's a manual copy-paste pipeline.

## Calibration subsystem — two independent layers

**1. Tare (`CalibratingState`, always runs):** Measures the resting pose as 
a reference to report the movement of the knob.

**2. Bundle calibration (`BundleState` / `BundleCalibrationController`, new):** 
a guided multi-step capture routine intended to solve the
Phantom Tilt problem via bundle adjustment. Step sequence:
`STATIONARY → FLAT_CIRCLE → PITCH → ROLL → TWIST → HEAVE → RANDOM → COMPLETE`.
Each step is a little sub-state-machine (`CalibPhase`:
`WAIT_FOR_START_BTN → COUNTDOWN → RECORDING → WAIT_FOR_ACK`) that streams
`CAL_FRAME`/`CAL_STATE` lines over Serial to a host script,
[`bundle_callibration.py`](magnet_field_model/bundle_callibration.py), which
ACKs frame counts back over the same link. A successful ACK moves straight on
to the next step.

**Status: capture, fit, and delivery all work.** The Python side fits a set of
shared parameters (per-magnet position, tilt and strength; per-sensor gain
matrix and DC offset) jointly with one free 6-DOF pose per captured frame,
taking the field residual down by most of an order of magnitude against
nominal geometry, in about a second. The parameter groups and their sizes are
declared by `BLOCKS`/`GROUP_SLICES` in
`magnet_field_model/calibration/parameterization.py`.  See
[`magnet_field_model/README.md`](magnet_field_model/README.md) for the
architecture and the load-bearing frame conventions.

Two things are worth knowing before touching it:

- The magnet position reference is the magnet's **geometric centre** — that
  is what `positions.h` means and what the notebook bakes into the bicubic
  table, and it is also where magpylib positions cylinders natively. The
  bottom-face convention this replaced still lives on in one place: the
  Python calibration fit (`magnet_field_model/calibration/`) hasn't been
  updated and still produces bottom-face-referenced `magnet_pos_knob`
  values, so `CalibrationStorage::deserialize()` reconciles that to the
  centre-based convention when loading a blob tagged with the older format
  version.
- The raw capture path streams `readUncorrected()`, so the sensors might read the
  **opposite sign** to the modelled field — matching `Config::magnet_gains`
  . The fit attributes that flip to magnet polarity,
  so fitted gains read near `+I` while the *exported* ones land near `-I`.
- Sensor gain scale and magnet strength are not separable (only a few percent
  cross-talk distinguishes them), so the fit picks a gauge: `det(G) = 1`, with
  magnet strength carrying the scale. The reported strengths are an
  attribution, not a measurement — the fit's information-gain column says so.

### Persistence

Fitted calibrations reach the firmware on their own. The whole fitted
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
be left parked) or when the user declines the write prompt.

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
- Every sensor sees every magnet. Its own is evaluated through the
  interpolated field above; the other two, ~28.58mm away and worth 1.7–4.5%
  of the signal, through `dipole_field()`'s point-dipole approximation. The
  Python fit models the same three-magnet physics
  (`calibration/bundle_geometry.py`'s `SENSOR_MAGNET_COUPLING = ALL_MAGNETS`),
  so the fitted magnet strengths mean what they say rather than absorbing a
  cross term the firmware could not reproduce. The two are not bit-identical:
  the fit uses the exact cylinder solution where the firmware approximates,
  a difference under 0.02% of the field. See
  `TODO/resolved/cross-magnet-interference.md`.
- `VirtualSensor` no longer carries a gain; correction happens entirely in
  `SensorController::read_mT()` (issue #6 below).

## Quick file index

| Looking for... | Look in |
|---|---|
| Pins, gains, dead zones, LED colors, timing | `firmware/include/Config.h` |
| Board/build flags, lib deps | `platformio.ini` |
| Sensor geometry / knob geometry constants | `firmware/include/magnet_model/positions.h` |
| The Gauss-Newton solver loop | `firmware/src/magnet_model/solve_pose.cpp` |
| Per-sensor analytic Jacobian | `firmware/src/magnet_model/virtual_sensor.cpp` |
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
