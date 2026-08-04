# CAD Mouse MK2 (accuracy fork) — Architecture Notes

Working reference for navigating and reasoning about this codebase. Written from
reading the code directly (not from any doc that could drift), so re-verify
specifics (function names, line numbers) before relying on them if it's been a
while — see `design documentation/context.md` for the *why*, this file is the
*where/how*.

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
`stateMachine.changeState(&calibratingState)`. `loop()` just calls
`hidController.task()` then `stateMachine.update()` — a classic non-blocking
super-loop, no RTOS.

### State machine

[`firmware/include/State.h`](firmware/include/State.h) — trivial interface:
`enter()/update()/exit()`.
[`firmware/include/StateMachine.h`](firmware/include/StateMachine.h) /
[`.cpp`](firmware/src/StateMachine.cpp) — holds one `currentState*`, calls
`exit()`/`enter()` on transition. States are **static members of StateMachine**
(singletons, no dynamic allocation — consistent with `EIGEN_NO_MALLOC`).

States, all in `firmware/{include,src}/states/`:

- **CalibratingState** — runs at boot. Drives `SensorController::updateCalibration()`
  for `Config::ZERO_SAMPLES` (200) samples to compute a baseline field offset and
  a baseline pose, then hands off to `IdleState`.
  ⚠️ Slated for replacement by a "tare" step — see
  [`TODO/tare-and-calibration.md`](TODO/tare-and-calibration.md).
- **IdleState** — the main operating state. Reads sensors → runs the pose solve →
  maps to HID axes → sends HID report → publishes telemetry → watches for the
  calibration-request gesture and the inactivity timeout to `SleepState`.
- **SleepState** — LEDs off, waits for any button activity to return to `IdleState`.
- **ErrorState** — entered if `SensorController::begin()` fails at boot. LED
  spinner in red, otherwise inert (no recovery path).
- **BundleState** *(new, uncommitted)* — hosts the serial-driven multi-step
  hardware calibration routine (see "Calibration subsystem" below).

### Controllers (global singletons, DI via `extern` in `Controllers.h`)

Declared in [`firmware/include/Controllers.h`](firmware/include/Controllers.h),
defined as globals in `main.cpp`. This is the project's entire DI mechanism —
there is no container, no interfaces, states and controllers reach each other
via these externs directly.

| Controller | File | Responsibility |
|---|---|---|
| `InputController` | `controllers/InputController.*` | Debounces 2 buttons via AceButton; exposes `buttonBits()`, `takeActivity()` (edge-triggered, consumed on read), `takeCalibrationRequest()` (both buttons held 3s) |
| `LEDController` | `controllers/LEDController.*` | NeoPixel ring: solid color / spinner animation / off |
| `SensorController` | `controllers/SensorController.*` | Owns the 3 TLx493D sensor objects, power-sequences them onto distinct I2C addresses at boot, `readRaw()` → 9 floats, runs the boot baseline calibration |
| `MotionController` | `controllers/MotionController.*` | The pose pipeline: raw field → solved pose → filtered/mapped HID axes (detail below) |
| `HIDController` | `controllers/HIDController.*` | Owns the USB HID descriptor + report state, dedupes unchanged reports before sending |
| `TelemetryController` | `controllers/TelemetryController.*` | Formats a big fixed-layout ASCII dashboard to Serial every 20 ticks when `Config::ENABLE_TELEMETRY` |
| `BundleCalibrationController` | `controllers/BundleCalibrationController.*` | *(new)* Serial-driven state machine for the guided hardware calibration capture |

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

`magnet_field_model/field_approximation.ipynb` (uv-managed Python env,
`magpylib` for ground-truth field simulation of a real 6×6mm N52 cylinder
magnet) samples the true field on the same 51×91 (r,z) grid defined by `NR`/`NZ`
in [`magnet_model_table.h`](firmware/include/magnet_model/magnet_model_table.h),
and hand-emits the hex-float C++ array in
[`magnet_model_table.cpp`](firmware/src/magnet_model/magnet_model_table.cpp).
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
`WAIT_FOR_START_BTN → COUNTDOWN → RECORDING → WAIT_FOR_ACK → REVIEW`) that
streams `CAL_FRAME`/`CAL_STATE` lines over Serial to a host script,
[`bundle_callibration.py`](magnet_field_model/bundle_callibration.py), which
ACKs frame counts back over the same link.

**Status: capture and fit both work; the result has nowhere to go.** The
Python side fits 42 shared parameters (per-magnet position and tilt,
per-sensor gain matrix) jointly with one free 6-DOF pose per captured frame,
and takes the field residual from ~1.8% at nominal geometry down to ~0.35%,
in about a second. See
[`magnet_field_model/README.md`](magnet_field_model/README.md) for the
architecture, the load-bearing frame conventions, and the measured numbers.

Two things are worth knowing before touching it:

- The magnet position reference is the magnet's **bottom face**, not its
  centre — that is what `positions.h` means and what the notebook baked into
  the bicubic table. magpylib positions cylinders by their centre.
- The raw capture path streams `readUncorrected()`, so the **nominal sensor
  gain is `-I`**, matching `Config::magnet_gains` (`{-0.96, -1.2, -0.98}`).

There is still no path for calibration *results* to reach the firmware
automatically: no flash/EEPROM write anywhere in this firmware, so
`bundle_callibration.py --emit-cpp` prints a pasteable C++ snippet and that is
as far as it goes. `Sensor`'s per-axis gain (`Mat3 sensor_gain`) and
`MagnetModel`'s per-magnet `magnet_rotation` are wired up in the math but
still constructed once from hardcoded `Config` values at static-init time in
`MotionController.cpp`.

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
| Field table generator (source of truth) | `magnet_field_model/field_approximation.ipynb` |
| HID descriptor / report format | `firmware/src/controllers/HIDController.cpp` |
| Serial diagnostics dashboard | `firmware/src/controllers/TelemetryController.cpp` |
| Jacobian correctness tests (finite-difference) | `firmware/test/test_jacobian.cpp` |
| New guided calibration flow | `firmware/src/controllers/BundleCalibrationController.cpp`, `firmware/src/states/BundleState.cpp`, `magnet_field_model/bundle_callibration.py` |
| Project intent / hardware spec | `design documentation/context.md` |
| Tracked follow-up work | `TODO/` |

## Issues / architecture drift spotted while reading

Tracked as of the initial read-through. Status as of the follow-up pass:

1. **`CalibratingState` → `BundleState` transition looks accidental** and
   2. **`BundleState` passes the wrong value as "button bits"** — both being
   superseded by a boot-time "tare" redesign rather than patched in place.
   See [`TODO/tare-and-calibration.md`](TODO/tare-and-calibration.md).

3. `solve_knob_pose()` not writing its Jacobian out-param — **fixed.**

4. **Circular coupling between `SensorController` and `MotionController`**
   and 5. **the forward model's state living outside any controller** — real
   architecture issues, tracked in
   [`TODO/controller-ownership.md`](TODO/controller-ownership.md).

6. **`Config::magnet_gains` can't express what the math supports** — widened
   into a decision to own sensor gain/skew correction in `SensorController`
   entirely outside the motion system, tracked in
   [`TODO/sensor-gain-calibration.md`](TODO/sensor-gain-calibration.md).

7. **`rcond` is a hardcoded stub** (deliberately — computing it via SVD every
   frame is too expensive on this FPU-less MCU, not an oversight) and
   8. **`TelemetryController::publish()`'s signature keeps growing** — merged
   into one telemetry rework, tracked in
   [`TODO/telemetry-rework.md`](TODO/telemetry-rework.md).

9. Unwired linker scripts (`custom_memmap.ld`, `full_custom_memmap.ld`) —
   **fixed** by removing the vestigial files.

10. **`firmware/README.md` documents the old, replaced motion model** — left
    open, tracked in [`TODO/readme-refresh.md`](TODO/readme-refresh.md).
