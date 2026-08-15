# Run the bundle calibration on the knob

Calibration today needs a PC. `magnet_field_model/calibration/` holds the
joint fit — per-magnet position, tilt and strength, per-sensor gain and
offset, plus one 6-DOF pose per captured frame — and the knob only ever
receives the finished result over `CAL_UPLOAD`. Every unit therefore has to
be walked through a host-side procedure to be usable.

The question this tracks: can that fit run entirely on the RP2040, so the
knob calibrates itself? Two things decide it — whether the Jacobian can be
computed on-device without a second forward-model derivation to maintain, and
whether the solver's normal equations fit in RAM at all. Both now have
answers. Neither is a decision to build it.

## Why it matters

Removing the PC removes the only step in bringing up a unit that isn't
self-contained. It also removes a standing correctness hazard: the firmware's
forward model and the Python fit are two hand-maintained implementations of
the same physics, deliberately pinned to each other (`PAIRED_ONLY`, see
`ARCHITECTURE.md`'s calibration section). Today that pinning is a convention
someone has to keep. If the fit runs on the firmware's own model, it stops
being possible for the two to disagree.

Against that: it is a large amount of new numerical code on a device with no
FPU, for an operation that runs once per unit.

## What has been established

Prototype lives in `firmware/experimental/bundle_calibration_jacobian/`,
deliberately outside the PlatformIO build. Its README covers the traps; the
findings that bear on whether this is worth doing:

- **The Jacobian does not need a second derivation.** `VirtualSensor::evaluate`
  already computes and returns enough to recover every shared-parameter
  derivative — the magnet-position, magnet-tilt and strength columns all fall
  out of its existing `J_pose` output with no extra `BicubicField` pass. The
  bundle forward problem genuinely wraps the normal one rather than
  duplicating it, which was the main code-complexity risk.
- **The normal equations fit.** They are arrowhead-shaped (one dense P×P
  shared block, one 6×6 pose block per frame, no cross-frame coupling), so
  Schur-eliminating each frame's pose reduces peak memory to O(P²) plus one
  transient frame — about 8 KB at P=45, independent of the ~60 frames. The
  direct alternative is a 405×405 dense system at ~656 KB, which does not fit
  in 256 KB at all. Verified against a dense reference solve.
- **Nothing off-the-shelf fits.** ArduinoEigen's sparse module allocates,
  Ceres' `tiny_solver` has no block structure, and there is no embedded
  bundle-adjustment library. Only the elimination loop is bespoke, though —
  the 6×6 LDLT it is built on is the same call `solve_pose.cpp` already makes.

## Decided: magnet tilt uses a gnomonic chart

`bundle_gnomonic_chart.h`. Each magnet's tilt is 2 numbers — where its
polarization axis crosses a plane one unit below the magnet, in the nominal
magnet's frame, with nominal at (0,0). `R_mag` stops being state and becomes a
derived quantity, rebuilt per magnet per iteration by a shortest-arc lift.

Three things this buys, in descending order of how much they matter:

- **Spin is unrepresentable rather than projected away.** A magnet is a solid
  of revolution, so spin about its own axis is an exact symmetry and there are
  only 2 measurable DOF. The old projection dropped a fixed coordinate column,
  which is the dead direction only at zero tilt; the chart's derivative
  columns are perpendicular to the magnet's *current* axis at any tilt, by
  construction.
- **It closes the `so3_left_jacobian` question.** With an explicit chart there
  is no convention to reconcile against scipy's — you differentiate your own
  map. Getting it wrong in either direction was a silent bug; now there is no
  choice to get wrong.
- **No drift, no trig.** Rebuilding from 2 floats can't accumulate
  orthogonality error the way a stepped matrix does, and the lift is rational
  plus one square root.

Gnomonic specifically, not two Euler angles: both are 2-parameter charts, but
Euler/spherical angles put their coordinate singularity at the pole, and the
pole is nominal. The azimuth column would scale as `sin(theta)` — at a
tolerance-sized tilt of a degree or two its weight in the normal equations is
~1e-3 of the polar column's, the ridge prior would swallow it, and 2 nominal
DOF would silently become ~1 fitted. The gnomonic chart's derivative at
nominal is an orthonormal basis of the tangent plane, and its singularity sits
at 90 degrees where the chart cannot reach.

**Cost: none that matters.** The chart is outer-loop (per magnet per
iteration, not per frame), and the only inner-loop change is a dense 3x2
multiply where the old projection was a free column slice — ~18 MAC per
(sensor, frame) against `H_ss`'s 3P² = 6,075 per sensor, about 0.3%. `P` is
unchanged, so the term that actually dominates is untouched.

**One interop consequence.** The PC fit keeps its nominal-frame
`TILT_UNIT_BASIS`, so the two sides now use different tilt charts. To first
order they agree up to a 90-degree relabel of the parameter plane, and an
isotropic prior is invariant under that — but `RegularizationSigmas` should be
converted deliberately rather than assumed to transfer.

## Design

Two passes per solver iteration, nothing per-frame stored between them:

1. For each frame, build its local system from 3 `VirtualSensor::evaluate`
   calls, Schur-eliminate its pose block into a persistent P×P accumulator,
   discard it. Solve the reduced P×P system once for the shared update.
2. For each frame, rebuild its pose row and back-substitute to get that
   frame's own pose update.

Recompute rather than store is chosen deliberately: storing every frame's
P×6 coupling block would cost ~65 KB for data needed only microseconds later,
and this device is RAM-constrained but not latency-constrained for an
operation that runs once per unit. Pass 2 is the cheaper of the two — it
never touches the P×P shared block, which is the dominant term in the
accumulation.

**One ordering constraint this imposes:** pass 2 must linearize at the same
point pass 1 did, so the shared and pose updates may only be applied to the
iterate after every frame's back-substitution. Applying early makes the
back-substitution stop being exact, silently.

## Open questions

These block writing a real solver, and are not settled:

*(Magnet tilt's parameterization used to be listed here. It is now decided —
see "Magnet tilt" below.)*
- **Whether it should be built at all.** The above says it's possible. It
  does not say it's worth the firmware surface area, and that is the actual
  decision.

## Not open after all: the P=45 column layout

This was listed as an open question — "which of the 45 columns belong to
which group, needed before the sparsity can be exploited." That was a
mistake: `parameterization.py` already owns a concrete layout in
`GROUP_SLICES`, which `bundle_geometry.py`'s hot path reads directly. The
firmware should mirror it rather than invent one, since the two agreeing is
the point.

| group | free | slice |
|---|---:|---|
| `magnet_pos` | 3 | 0:3 |
| `magnet_tilt` | 6 | 3:9 |
| `magnet_strength_mean` | 1 | 9:10 |
| `magnet_strength_diff` | 2 | 10:12 |
| `sensor_offset` | 9 | 12:21 |
| `gain_aniso` | 6 | 21:27 |
| `gain_sym` | 9 | 27:36 |
| `gain_rot` | 9 | 36:45 |

**With the layout known, the deferred sparsity optimization is worth
quantifying, because it is much larger than anything else on this page.**
Under `PAIRED_ONLY` sensor *i* sees only magnet *i*, so one sensor's 3 rows
touch: all 3 `magnet_pos` columns (the gauge basis moves all three magnets
together), its own magnet's 2 `magnet_tilt` columns, `strength_mean`, 1–2
`strength_diff`, its own 3 `sensor_offset`, and its own 8 gain columns —
**about 18–19 of 45.**

`H_ss += Jᵀ J` only has nonzeros where both columns are live, so a
sparsity-aware accumulation costs ~3·19² ≈ 1,083 MAC per sensor against
3·45² = 6,075 dense: roughly a **6× cut on the term that dominates the whole
solver**. Every other optimization considered so far has been worth ~0.3%.
This one is worth most of the total. It should be the first thing done if
this is built, not a follow-up.

Counted from the basis shapes rather than measured — worth re-deriving in
code before relying on the exact figure, though the order of magnitude is
not in doubt.

## Not built

- The LM/trust-region outer loop — damping, step acceptance, convergence.
  The prototype is the per-iteration inner machinery only.
- Wiring the raw per-sensor derivatives into the solver's P-wide column
  layout: gauge-basis projection and per-observation sigma weighting.
- Finite-difference coverage for the gain and offset derivatives. They are
  linear post-multiplies with no chain-rule content, so there is little for a
  bug to hide in, but they are unverified.
- Any on-hardware run. The prototype cross-compiles for cortex-m0plus
  (`verify_arm.sh`) and costs ~22 KB of flash and 16 bytes of static RAM at
  P=45, against 15% flash used today — so it fits comfortably. But that is a
  build, not an execution: no timing figure here comes from the device, and
  the speed question on a soft-float M0+ is exactly the one that decides
  whether a calibration takes seconds or minutes.
- Nothing here has been through `pio test -e seeed_xiao_rp2040_test`, because
  none of it is under `test_dir`. Promoting it there is the step that would
  make it testable on hardware at all, and it is deliberately not taken yet.

## The stack is the binding constraint, not total RAM

Worth stating separately because it was got wrong once already. The RP2040
gives each core a **4 KB** stack (`memmap_default.ld`: core0 in SCRATCH_Y,
core1 in SCRATCH_X, both `LENGTH = 4k`). At P=45 the solver's two big
structures are 9,528 and 8,280 bytes — each larger than the whole stack — so
they have to be statically allocated, not automatic. An earlier version of
`SCHUR_SOLVER_DESIGN.md` described them as stack locals, which would have
smashed the stack on the first call and, with no MPU on a Cortex-M0+,
corrupted memory below rather than faulting at the bug.

This is also the clearest example of what on-device testing would catch that
the host builds cannot: the host has an 8 MB stack and would run the same
code without complaint forever.
