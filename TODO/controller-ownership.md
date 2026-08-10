# Fix controller ownership / coupling around the pose pipeline

Originated from ARCHITECTURE.md issues #4 and #5. Confirmed as real
architectural issues, not just style nits.

## Problem 1: `SensorController` ↔ `MotionController` circular coupling

`SensorController.cpp` does:

```cpp
extern MotionController motionController;
```

and calls `motionController.read_pose()` / `motionController.set_base_pose()`
directly from inside `SensorController::updateCalibration()`. Meanwhile
`MotionController::compute()` separately depends on
`SensorController::baseline()`. Every other cross-controller interaction goes
through the state layer (e.g. `IdleState` explicitly reads from
`sensorController` and pushes into `motionController` — neither controller
reaches into the other). This one pair breaks that pattern and calls each
other directly, so the dependency direction between them is ambiguous, and
`SensorController` can't be reasoned about or tested without pulling in the
entire motion/solver stack.

Likely fix shape: move whatever `updateCalibration()` needs
(`read_pose`/`set_base_pose`) up into the state that orchestrates
calibration (see `TODO/tare-and-calibration.md` — this will need to be
resolved together with the tare redesign, since tare is exactly the state
that currently drives this calibration loop).

## Problem 2: the forward model's state lives outside any controller — RESOLVED

Fixed as part of the `CalibrationParams` work. `MotionController` now owns a
`const ForwardModel forward_model_` member, constructed from the
`CalibrationParams` the controller was built with; `J`/`B_field` were dead and
are deleted. `MotionController` and `SensorController` are each reached
through their own Meyers-singleton accessor, `motionController()` /
`sensorController()` (`firmware/include/Controllers.h`), independently built
on first call from `resolveCalibration()` at a point `setup()` chooses — which
is what gives a flash-loaded calibration somewhere to land. The original
description follows, for context.

In `firmware/src/controllers/MotionController.cpp`, at file scope (not class
members):

```cpp
MagnetModel magnets[3] = { ... };
const Sensor sensors[3] = { ... };
ForwardModel forward_model(sensors, magnets);
Eigen::Matrix<float, 9, 6> J;        // declared, appears unused — dead, check and remove
Eigen::Matrix<float, 9, 1> B_field;  // declared, appears unused — dead, check and remove
```

Every other controller owns its dependencies as class members
(`SensorController` owns its 3 sensor objects as members, for instance). Here
the actual math model — the thing the whole solver pipeline runs against — is
a translation-unit global, built once from `Config` constants at static-init
time, invisible to any header, with no accessor and no way to reconstruct or
mutate it at runtime.

This matters beyond style: it's the reason there's currently no path for a
future calibration write-back (see `TODO/sensor-gain-calibration.md`) to ever
reach the running `Sensor`/`MagnetModel` instances — they're not reachable
from anywhere calibration code could plug into.

Likely fix shape: `MotionController` (or a new owner, depending on how
`TODO/sensor-gain-calibration.md` resolves) should own `sensors`, `magnets`,
and `forward_model` as members, constructed from whatever config/calibration
source is decided, with `J`/`B_field` either wired to an actual use or
deleted.

## Problem 3: `BundleCalibrationController` mixes both access patterns — RESOLVED

Not actually an inconsistency: `BundleState.cpp` already documents why
`SensorController` comes in as an explicit parameter to `update()` instead of
through `sensorController()` — "so the callibrator can be selective in when
it wants to read the sensor." That's a deliberate reason for the split, not
two competing access patterns with none. The original description follows,
for context.

Every controller is reached through its own accessor function (`ledController()`,
`sensorController()`, ...) declared in `Controllers.h` — that's the project's
one consistent access pattern, and all five `State` subclasses use it. But
`BundleCalibrationController` mixes two different ways of getting at its
dependencies: it calls `ledController()` directly from inside `change_phase()`,
while `SensorController` instead comes in as an explicit parameter —
`BundleCalibrationController::update(uint16_t button_bits, SensorController
&sensorController)`, passed in by `BundleState::update()`
(`firmware/src/states/BundleState.cpp`) — even though `sensorController()` is
sitting right there and would work exactly the same way `ledController()`
does. So within this one class the access pattern is inconsistent, with no
apparent reason for the split (e.g. `LEDController` isn't more "shared" or
`SensorController` more "test-isolated" in any way that's evident from the
code).

(An earlier version of this section also flagged `SensorController.cpp`
reaching `MotionController` via its own inline `extern MotionController
motionController;` instead of going through `Controllers.h` like everyone
else. That's now fixed — `SensorController.cpp` includes `Controllers.h` and
calls `motionController()` like everything else does, as part of the same
accessor-function migration that resolved Problem 2.)

## Note

These two problems are linked: fixing #2 (giving the forward model a real
owner) will likely reshape how #1 gets resolved, since "who owns the model"
and "who's allowed to call into the solver during calibration" are the same
question from two directions. Worth designing together rather than
sequentially.

In the event #2 was resolved without touching #1. Giving the model an owner
turned out not to require deciding who may call into the solver — the call in
`updateCalibration()` just got re-spelled as `motionController().read_pose()`
and is as coupled as it ever was. #1 remains open and still belongs with the
tare redesign.
