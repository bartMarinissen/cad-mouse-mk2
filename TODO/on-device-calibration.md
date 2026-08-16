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

## Built: the solver

`bundle_solver.h`, verified end to end by `verify_solver.sh` — 30 synthetic
frames generated from a known parameter vector through the real forward model,
then fitted from nominal. Cost falls from 1423 to 1.1e-7, an RMS residual of
0.00002 mT over 270 observations, in 20 iterations. A second test starts the
fit *at* the answer and checks the pose back-substitution does not drift poses
that were already correct — pass 2 runs every iteration regardless, so a sign
error there would otherwise hide behind the shared fit absorbing it. Two more
cover the ridge term: that a tight prior pulls a weakly-observable parameter
toward nominal rather than away from it, and that the unregularized
`magnet_strength_mean` stays both well-posed and well-determined.

The linear solve is a plain dense `H.ldlt()`. `H_ss`'s own bordered
block-diagonal structure could be Schur-eliminated a second time, reducing the
45x45 to a 12x12, and deliberately is not: the P×P factorization is ~30k MAC
against ~1.09M for the accumulation it sits on, so that optimizes ~3% of an
iteration in exchange for a second differently-shaped elimination to keep
correct. The block structure earns its keep in the *accumulation*, which is
where the cost is. Structure for accumulation, dense solve.

## Decided: priors, and magnet_strength_mean carries none

`nominal_prior_sigma()` in `bundle_param_layout.h` ports
`bundle_params.py`'s `RegularizationSigmas` into the firmware's unit-major
column order. Same beliefs about the same hardware, permuted, not
independently chosen: magnet position 1 mm (3D-printed knob), tilt 0.06 rad,
strength spread 0.1, sensor offset 1.8 mT, gain aniso 0.25, gain sym/rot 0.03.

**`magnet_strength_mean` is deliberately unregularized**, matching `None`
there. The nominal it would be centred on is a round guess; real magnets
plausibly run anywhere from 500 to 1500 mT, with no reason to prefer the
middle, and this has to work for units nobody has measured. A prior there
would drag the fitted field scale toward a number nobody stands behind.

That is safe despite the parameter being near-degenerate with position and
gain (`cross-magnet-interference.md`), for a specific reason worth keeping: a
flat direction only survives if it lies *entirely* within unregularized
coordinates. `magnet_strength_mean` is one column, and alone it is strongly
observable — it scales the field. It is only degenerate in *combination* with
position and gain, and those are strongly and justifiably regularized, so the
prior on the partners supplies curvature along the whole combined direction.
Measured: with the nominal priors, the fit recovers a +6% synthetic field
scale as +5.97%.

**Magnet strength is a dimensionless multiplier, not an mT offset**, matching
`bundle_geometry.py`'s `magnet_strength = 1.0 + offset`. This was got wrong
first: an absolute mT parameterization makes the ported sigmas off by the
nominal strength — a factor of ~1000 — and it needs a chain-rule factor of
`nominal_strength_mT` in the Jacobian column, because
`SharedJacobianBlock::d_strength` is the absolute derivative. Both errors
mis-fit everything else rather than failing outright; the column-wiring test
caught both.

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

*(Magnet tilt's parameterization and the column ordering used to be listed
here. Both are now decided — see the sections above.)*

- **Whether it should be built at all.** The above says it's possible. It
  does not say it's worth the firmware surface area, and that is the actual
  decision.

## The P=45 parameters, and how to order them

The *partition* was never open — `parameterization.py` owns it. The
*ordering* is, and is a real design question; both are below.

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

**The sparsity that matters does not depend on this ordering at all.** Under
`PAIRED_ONLY` sensor *i* sees only magnet *i*, so one sensor's 3 rows touch
all 3 `magnet_pos` columns (the gauge basis moves all three magnets
together), its own magnet's 2 `magnet_tilt` columns, `strength_mean`, 1–2
`strength_diff`, its own 3 `sensor_offset` and its own 8 gain columns —
**about 18–19 of 45.** `H_ss += Jᵀ J` is only nonzero where both columns are
live, so a sparsity-aware accumulation is ~3·19² ≈ 1,083 MAC per sensor
against 3·45² = 6,075 dense: roughly a **6× cut on the term that dominates
the solver**, when everything else considered on this page is worth ~0.3%.

That count is a property of the *partition* — which parameters exist and
which sensor's residuals touch which — not of the column order. Permuting
columns maps H to Π H Πᵀ, which relabels entries without creating or
destroying zeros. An earlier version of this section said the layout had to
be settled before the sparsity could be priced; that was wrong, and it
confused needing `BLOCKS` with needing `GROUP_SLICES`.

## Design: order columns by unit, not by parameter type

Ordering does not change the operation count, but it decides whether the
nonzeros are *consolidated* — and the PC's type-major order is close to
worst-case for that. Sensor 0's 19 live columns land in 8 disjoint runs
(`{0,1,2} {3,4} {9} {10} {12,13,14} {21,22} {27,28,29} {36,37,38}`).

Partitioning by who-sees-what instead gives two runs per sensor:

- **border (6):** `magnet_pos` 3 + `strength_mean` 1 + `strength_diff` 2 —
  every sensor touches these
- **per-unit `U_i` (13 each):** `tilt_i` 2 + `offset_i` 3 + `gain_i` 8 — only
  sensor *i* touches these

`strength_diff` is what fixes the border at 6 rather than 4: magnet 2's
strength is `-d0-d1`, so sensor 2 touches both columns, and splitting them
across `U_0`/`U_1` would make sensor 2 straddle two blocks.

**`H_ss` is then itself bordered block-diagonal** — the same arrowhead as the
outer problem, one level down — because `H_ss[U_i, U_j] = 0` exactly for
i≠j: that entry needs a sensor touching both, and no sensor touches more than
its own. Consequences:

- storage 777 floats against 2025, ~2.6x — and `SharedNormalEquations`
  already exceeds the 4 KB stack, so this is not only about MACs
- each sensor's contribution becomes three fixed-size `H.block<a,b>(i,j) +=`
  with compile-time extents, rather than a scatter through 8 runs with
  runtime index arithmetic — which matters under `EIGEN_NO_MALLOC`
- the reduced system can be Schur-eliminated *again*, folding the three 13x13
  blocks into the border for a final 6x6 solve. Minor on its own (the P×P
  LDLT was ~30k against 1.09M for accumulation) but free once ordered

**Decided: the firmware orders unit-major, and does not have to match the PC
side.** The interop objection that made this a Design is withdrawn — the
on-device layout is free to differ, so the two vectors are related by a fixed
permutation applied at the storage boundary, and nothing else.

### Put `magnet_tilt` in the border, not in the blocks

The partition above holds under today's `PAIRED_ONLY` coupling. Modelling
cross-magnet interference (`cross-magnet-interference.md`, an intended
direction) changes it — but by less than it looks.

**The outer arrowhead is untouched.** Math.md §7.1 requires only that frame
*k*'s residual depends on `x` and `pose_k` alone. Cross-magnet coupling is
entirely *within* a frame — sensor *i* seeing magnet *j* at one instant — and
never links frame *k* to frame *l*. The Schur elimination, the streaming, the
O(P²) memory argument: all unaffected.

**`H_ss`'s own arrowhead survives too**, because what separates `U_i` from
`U_j` is not magnetics. `sensor_offset` and `gain` belong to a specific
physical sensor, and no amount of field cross-talk makes sensor *j*'s reading
depend on sensor *i*'s gain matrix. Only `magnet_tilt` migrates, from
per-unit to border (and `strength_diff` goes from 1–2 live columns per sensor
to 2, since each sensor then sees all three magnets' strengths):

| | border | `U_i` | live/sensor | `H_ss` speedup | storage |
|---|---:|---:|---:|---:|---:|
| `PAIRED_ONLY`, tilt in blocks | 6 | 13 | 19 | 5.6x | 777 |
| `ALL_MAGNETS`, tilt in border | 12 | 11 | 23 | 3.8x | 903 |

So there is a fork. Tilt in the blocks gives the full 5.6x now, but on the
switch those 6 columns must leave the blocks and cannot join a contiguous
border — they would sit at offsets 6, 19 and 32, and the border stops being
one run. Tilt in the border makes the structure **invariant** across the
switch: `ALL_MAGNETS` then changes only which entries happen to be zero, not
the block geometry, so the accumulator does not need restructuring or
re-verifying.

Take the invariant ordering. Paying 47% of an optimization to avoid rebuilding
and re-verifying the accumulator is the right side of that trade when the
switch is intended rather than hypothetical, and 3.8x is still far larger than
anything else available here.

Note `evaluate_bundle_jacobian` is already per-(sensor, magnet), so
`ALL_MAGNETS` means calling it 9 times per frame instead of 3 and
accumulating — no interface change. The dipole correction that TODO proposes
does not go through `VirtualSensor::evaluate` and so needs its own analytic
Jacobian, but the shape it plugs into is already right.

All counts here are derived from basis shapes, not measured — worth
re-deriving in code before relying on exact figures, though the orders of
magnitude are not in doubt.

## Not built

- Cross-magnet coupling itself — the solver calls the per-(sensor, magnet)
  entry point once per sensor, not once per pair. See
  `cross-magnet-interference.md`; the layout is already sized for the switch.
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
