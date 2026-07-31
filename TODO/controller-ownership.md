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

## Problem 2: the forward model's state lives outside any controller

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

## Note

These two problems are linked: fixing #2 (giving the forward model a real
owner) will likely reshape how #1 gets resolved, since "who owns the model"
and "who's allowed to call into the solver during calibration" are the same
question from two directions. Worth designing together rather than
sequentially.
