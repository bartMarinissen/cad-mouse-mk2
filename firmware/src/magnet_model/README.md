# magnet_model

The magnetic forward model and pose solver: predicts sensor readings from a
candidate knob pose, and solves that model backwards to recover the pose from
real sensor readings. This file is a map of what each source file does. For
the *math* behind it (the closed-form derivation of every Jacobian below),
see `design documentation/Math.md` at the repo root. For where this fits in
the whole firmware — the state machine, controllers, calibration subsystem —
see [`ARCHITECTURE.md`](../../../ARCHITECTURE.md), also at the repo root.

Every `evaluate()` in this directory has a hand-derived analytic Jacobian
(no finite-difference in the hot path — this runs on an FPU-less RP2040).
[`firmware/test/test_jacobian.cpp`](../../test/test_jacobian.cpp) checks all
of them against central finite differences. Re-run it after touching any
`evaluate()` here.

## Files, in call order

`solve_knob_pose()` → `ForwardModel::evaluate()` → `Sensor::evaluate()` →
`MagnetModel::evaluate()` → `BicubicField::evaluate()` → the generated table.

- **`solve_pose.h`/`.cpp`** — `solve_knob_pose()`, the actual solver: a
  Levenberg-Marquardt (damped Gauss-Newton) loop, up to `MAX_ITER` (10)
  iterations, each one building the 6x6 normal equations from the 9x6
  Jacobian and solving via LDLT, stopping early once the update step drops
  below `TOLERANCE`. Takes/returns the pose as `(t, R)` in/out params so the
  caller can hot-start from the previous frame's pose. Optional
  `residual_out`/`Jacobian_out` pointers let the caller pull solver
  diagnostics without a second pass.

- **`forward_model.h`/`.cpp`** (`ForwardModel`) — owns the 3 `Sensor`/
  `MagnetModel` pairs and loops over them, assembling the combined 9x1
  residual and 9x6 Jacobian (`evaluate()`) that the solver operates on. Each
  sensor is paired with exactly one magnet (`sensors_[i]` against
  `magnets_[i]`) — there's no cross-magnet term (see
  `TODO/cross-magnet-interference.md`).

- **`sensor.h`/`.cpp`** (`Sensor`) — one sensor's contribution: transforms
  the sensor into the paired magnet's local frame given a candidate pose,
  calls into `MagnetModel::evaluate()` for the field, then composes the 3x6
  analytic Jacobian for that sensor (chain rule through the rigid-body
  transform, see `Math.md` §4).

- **`magnet_local_model.h`/`.cpp`** (`MagnetModel`) — decomposes a
  magnet-local offset into cylindrical `(r, z)` (the magnets are axially
  polarized, so the field only depends on radius and height relative to
  their own axis), queries `BicubicField` at that `(r, z)`, and reconstructs
  the 3D field and its local 3x3 Jacobian from the cylindrical
  components/partials (`Math.md` §4.D). Handles the `r → 0` singularity
  explicitly — the naive `1/r` terms blow up on-axis, so this collapses to
  an isotropic limit there instead.

- **`BicubicField.h`/`.cpp`** — Catmull-Rom bicubic interpolation over the
  precomputed `(r, z)` grid, returning both the value and its `d/dr`, `d/dz`
  partials in one `evaluate()` call. `point_or_ghost()` allows querying one
  step past the real grid edge (`i`/`j` in `[-1, NR]`/`[-1, NZ]`) by linear
  extrapolation from the two nearest real nodes — needed because the 4-point
  cubic stencil reaches one cell past the query point's cell at the grid
  boundary.

- **`magnet_model_table.h`/`.cpp`** — the precomputed field grid itself
  (`BICUBIC_INTERPOLATION_TABLE`, a `[NZ][NR]` array of hex-float `Vec2`
  value pairs) and the grid's shape: `NR`/`NZ` are the grid's resolution,
  `BICUBIC_ORIGIN`/`BICUBIC_FAR` are the `(r, z)` bounds of the grid's first
  and last sample. **Auto-generated — written directly by cell 17 of
  [`magnet_field_model/field_approximation.ipynb`](../../../magnet_field_model/field_approximation.ipynb)**
  (a `magpylib` ground-truth simulation of the real magnet), not hand-copied.
  Re-running that cell overwrites both files; hand edits will silently vanish
  on the next regen — both files carry an in-file warning banner to that
  effect. If the magnet spec or grid resolution/bounds change, they need to
  be changed in the notebook and picked up here by re-running it; nothing
  enforces the two stay in sync automatically.
