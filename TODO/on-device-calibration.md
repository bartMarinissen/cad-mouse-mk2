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

- **How magnet tilt is parameterized on-device.** Whether the solver needs a
  `so3_left_jacobian` correction for tilt (as `bundle_geometry.py` does)
  depends on whether it keeps `R_mag` as a persistent matrix stepped in the
  tangent space each iteration — matching `solve_pose.cpp`'s convention for
  the pose rotation — or as a rotation vector from nominal. The prototype's
  tilt columns are raw derivatives and do not commit to either.
- **The concrete shared-parameter column layout.** Which of the ~45 columns
  belong to which group. Needed before the per-frame blocks' real sparsity
  can be exploited: most groups touch only one sensor's 3 rows per frame, so
  the dense P×P the prototype forms is far denser than the problem is.
- **Whether it should be built at all.** The above says it's possible. It
  does not say it's worth the firmware surface area, and that is the actual
  decision.

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
