# Bundle calibration Jacobian — prototype

Exploring feasibility of running full bundle calibration (the joint fit
`magnet_field_model/calibration/` currently does on the PC) on the knob
itself. This directory is the Jacobian half of that question: can the
calibration's magnet-position/tilt/strength/gain/offset derivatives be
computed on-device by wrapping the existing `Sensor::evaluate()` forward
pass, instead of re-deriving the forward model a second time.

**Status: prototype, verified against finite differences, NOT wired into the
firmware build.** It lives here (outside `firmware/src`, `firmware/include`,
`firmware/test` — the three directories `platformio.ini`'s `src_dir`/
`include_dir`/`test_dir` actually point at) specifically so it stays
invisible to PlatformIO's build and doesn't silently join the shipped
binary or the on-device test suite. Promoting it there is a deliberate next
step, not a side effect of this commit.

## What's here

- `bundle_shared_jacobian.h` / `.cpp` — the two parameter groups that need
  real chain rule through the forward model: `d_magnet_pos`, `d_magnet_tilt`
  (full 3x3, all raw columns), `d_strength`. Computed from
  `Sensor::evaluate()`'s own output (`B_field_global`, `J_pose`) rather than
  a second pass through `MagnetModel`/`BicubicField`.
  The entry point is split in two, matching what actually varies at what
  rate: `MagnetState` is the answer to "how do calibration parameters,
  which are `const` members of `MagnetModel`, actually get updated across
  solver iterations" — a small mutable struct the solver perturbs once per
  magnet per iteration. `build_magnet_model(field, state)` builds the trial
  (still-immutable) `MagnetModel` from it — call this 3 times per iteration,
  in the OUTER loop. `evaluate_bundle_jacobian(sensor, magnet, t, R, ...)`
  calls `Sensor::evaluate()` itself and takes an *already-built* `MagnetModel`
  — call this once per (sensor, frame), reusing the same 3 built objects
  across all ~60 frames, not rebuilding one per frame. Measured (host x86,
  static instruction count): the constructor is ~30% of one full field
  evaluation's cost, not negligible — rebuilding per-frame instead of
  per-iteration would waste roughly 22% of an iteration's compute
  reconstructing the same 3 magnets up to 60 times over. `std::move` doesn't
  help here: there's no heap allocation anywhere in this chain
  (`EIGEN_NO_MALLOC`) for a move to avoid copying — see the header comment
  above `MagnetState` for the full reasoning. There used to be a separate
  `evaluate_shared_jacobian()` underneath, taking already-computed
  `B_field_global`/`J_pose` as inputs — folded directly into
  `evaluate_bundle_jacobian()` once it had exactly one caller and no reason
  to stay a separate function.
- `bundle_linear_jacobian.h` — the other five parameter groups
  (`sensor_offset`, `gain_aniso`/`sym`/`rot`, magnet-strength mean/diff
  split): linear post-multiplies of the prediction, no chain-rule content,
  no model call needed.
- `bundle_magnet_pos_gauge.h` — `MAGNET_POS_BASIS`, the fixed 9x3 constant
  (pasted from `parameterization.py`'s SVD, run against this bundle's real
  nominal geometry) that projects the raw per-magnet position derivative
  onto the bundle's 3 actual free shape parameters, plus `project_magnet_pos()`.
- `test_bundle_shared_jacobian.cpp` — finite-difference verification, in the
  same style as `firmware/test/test_jacobian.cpp` (perturb the real physical
  quantity, rebuild the const-membered model, central-difference). Every
  check goes through `evaluate_bundle_jacobian()` — including the FD
  perturbations themselves, via a perturbed `MagnetState` — not a private
  shortcut, so the tests exercise the same call a solver would actually
  make. Two tiers: per-quantity checks, and an end-to-end check that the
  assembled, gauge-projected free-parameter column matches perturbing the
  real physical shape parameter (moves all three magnets at once) — the
  check that actually catches wiring bugs (wrong magnet's basis slice,
  transposed sign) that the per-quantity checks alone can pass right through.
- `compat/`, `verify.sh` — host build harness (real Eigen 3.4 via
  `libeigen3-dev`, real firmware forward-model source, no ARM toolchain
  needed) so the verification is a reproducible `./verify.sh`, not a claim.
- `SCHUR_SOLVER_DESIGN.md`, `schur_normal_equations.h`,
  `test_schur_normal_equations.cpp`, `verify_schur.sh` — the *shape* the
  bundle solver would actually run on: `FrameNormalEquations<P>` (one
  frame's local normal equations, built transiently, never stored across
  frames) and `SharedNormalEquations<P>` (the only state that persists
  across a whole solver iteration, O(P²) not O(P·N)), connected by an exact
  Schur-complement elimination of each frame's 6-DOF pose block — the same
  fixed-size LDLT `solve_pose.cpp` already does, reused, not a new
  primitive. Not the LM/trust-region outer loop itself (damping, step
  acceptance, convergence) — see `SCHUR_SOLVER_DESIGN.md` for the full
  derivation and the store-vs-recompute memory tradeoff (~65 KB to store
  every frame's coupling block vs. recomputing it — recompute wins on this
  device). Verified exactly (not by finite difference — Schur complement is
  a linear-algebra identity, so there's a ground-truth answer): assemble
  the full dense arrowhead system directly, solve it in one shot, check the
  frame-by-frame path agrees. It does, to ~1e-6 relative error.

## What's verified, and to what precision

| check | precision | bar |
|---|---:|---:|
| `d_magnet_pos` vs FD | 0.16% | 1% |
| `d_magnet_tilt` (full 3x3) vs FD | 0.018% | 1% |
| `d_strength` vs FD | 0.004% | 1% |
| magnet's own-axis spin invariance (exact, not FD) | ~2e-6 | 1e-4 |
| gauge-projected column vs perturbing the real shape parameter | passes | 1% |
| reused vs freshly-rebuilt `MagnetModel`, across 3 frames | bit-exact | exact |
| Schur-complement path vs dense reference solve (exact, not FD) | ~8e-7 | 1e-4 |

The "own-axis spin does nothing" property was originally checked by finite
difference and failed at a 1e-3 bar — not because the code was wrong (the
raw analytic column matched FD to 0.2%), but because it's an exact symmetry,
not a derivative, and differencing two large near-equal floats to extract a
near-zero value is exactly where central differences are worst (roundoff,
not truncation, dominates — confirmed by hand: shrinking the FD step made
the spurious "leakage" grow, 0.94 → 12.7, the signature of cancellation
noise). Rewritten as a direct invariance check (spin the magnet by real
angles, compare `Sensor::evaluate`'s output, no subtraction) it resolves to
float32 machine precision instead.

## What's NOT here

- **The LM/trust-region outer loop.** `schur_normal_equations.h` is the
  per-iteration inner machinery (accumulate, solve, back-substitute) — not
  damping, step acceptance, or convergence criteria, and not yet connected
  to real per-frame Jacobians (see `SCHUR_SOLVER_DESIGN.md`'s "what's
  deliberately not here" for the rest, including why `H_ss`'s real sparsity
  isn't exploited yet).
- **Gain/offset finite-difference tests.** `bundle_linear_jacobian.h`'s
  derivatives are linear one-liners with no chain-rule content, so they
  weren't given FD coverage here — worth a quick direct check before this
  is trusted for real, even though there's little for a derivation bug to
  hide in.
- **The magnet-tilt SO(3) parameterization decision.** `bundle_magnet_pos_gauge.h`
  flags this explicitly: on-device, whether the bundle solver needs a
  `so3_left_jacobian`-style correction for magnet tilt (the way
  `bundle_geometry.py` does, because scipy holds a rotation-vector-from-nominal
  parameterization) depends on whether the solver keeps `R_mag` as a
  persistent matrix stepped in the tangent space each iteration (matching
  `solve_pose.cpp`'s existing convention for the pose rotation `R`) or as a
  vector-from-nominal. Not decided yet — the solver doesn't exist.
- **ARM build verification.** Everything here has only been host-compiled
  (`g++`, real Eigen 3.4 via apt, `-DEIGEN_NO_MALLOC`). Not yet run through
  `pio run` against the actual `earlephilhower`/RP2040 toolchain. This
  includes the ~30%/~22% construction-cost numbers above: measured as static
  host x86 instruction counts (same method `TODO/Performance.md` uses
  elsewhere), not dynamic ARM soft-float counts — directionally real, not a
  substitute for measuring on the actual target once there's a full solver
  to measure.
