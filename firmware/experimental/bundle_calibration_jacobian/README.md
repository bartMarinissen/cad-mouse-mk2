# Bundle calibration on-device — prototype

Feasibility work for running the joint calibration fit (today:
`magnet_field_model/calibration/`, on a PC) on the knob itself. Two questions:
whether the much larger Jacobian can reuse the firmware's existing forward
model instead of becoming a second derivation to maintain, and whether the
solver's normal equations fit in RAM.

**`TODO/on-device-calibration.md` owns what is and isn't answered, and what is
still open.** Read it first — this file is only about working in this
directory.

Run the checks:

```bash
./verify.sh        # Jacobian, against central finite differences
./verify_schur.sh  # solver shape, against a dense reference solve
./verify_arm.sh    # cross-compiles for cortex-m0plus, reports flash/RAM cost
```

The first two build on the host with `g++` and real Eigen 3.4
(`libeigen3-dev`), no ARM toolchain, and print the measured error for every
check — read the numbers there rather than from any summary, including this
one. The third needs the earlephilhower toolchain and skips itself if it isn't
installed; it is a compile check, not a run.

## Why it is outside the build

`platformio.ini` points `src_dir`/`include_dir`/`test_dir` at `firmware/src`,
`firmware/include` and `firmware/test`. This directory is under none of them,
so PlatformIO cannot see it: nothing here can reach the shipped binary or the
on-device test suite by accident. That is deliberate, and it is the one fact
about this directory that isn't visible from reading its contents — an absence
doesn't show up in a file. Moving any of it into the build is a decision to
make on purpose.

The flip side: nothing here is covered by `pio test`, so the two scripts above
are the only thing standing between this code and a silent wrong derivative.

## Why one check uses finite differences and the other doesn't

They are checking different kinds of claim, and using the wrong instrument for
either would hide real bugs.

The Jacobian is hand-derived algebra with no ground truth to compare against —
only the model itself, differenced. So `verify.sh` finite-differences it, the
same way `firmware/test/README` explains for `test_jacobian.cpp`, and for the
same reason: a wrong derivative doesn't crash, it just makes a solver converge
slowly to a slightly wrong answer.

The Schur elimination is not an approximation — it is a linear-algebra
identity. There *is* a ground-truth answer: assemble the full arrowhead system
densely and solve it in one shot. `verify_schur.sh` does that and demands the
frame-by-frame path agree to float32 precision. Finite differences would be a
strictly weaker check of something exactly checkable.

## Things to know before trusting a result

- **Finite differences are the wrong tool for an exact symmetry, and fail in a
  way that looks like a bug.** Spinning a magnet about its own polarization
  axis changes nothing — the derivative is structurally zero, not small.
  Extracting a zero by differencing two nearly-equal large floats is where
  central differences are worst: roundoff dominates truncation, so *shrinking*
  the step makes the error grow (measured: 0.94 → 12.7 as the step went down).
  That reads exactly like a wrong derivation. The test checks it as a direct
  invariance instead — rotate by real angles, compare outputs, never subtract —
  which resolves to machine precision. If a near-zero target ever fails here,
  suspect the instrument before the algebra.
- **The dead axis is the magnet's own current polarization axis**, not a fixed
  world axis. An earlier version of this test asserted the wrong one and failed
  against a correct implementation.
- **Synthetic test data must be checked for rank, not assumed.** The Schur test
  originally generated matrix entries as `sin()` of a linear combination of
  indices. Every such matrix lives in a 2-D subspace regardless of its size,
  because `sin(x + shift)` is always a fixed linear combination of
  `sin(x)`/`cos(x)` — so every synthetic pose block was singular, and the test
  failed at ~100% error on correct code. It now uses `std::mt19937` and
  explicitly asserts each block is full rank before comparing, because on a
  singular system two solve paths can each produce *an* answer without
  producing the *same* one, which is indistinguishable from a real bug.
- **Pass 2 must linearize where pass 1 did.** Applying the shared update before
  every frame has been back-substituted makes the back-substitution silently
  stop being exact. `SCHUR_SOLVER_DESIGN.md` has the detail.
- **The instruction-count figures in the headers are static host x86 counts**,
  the same method `TODO/Performance.md` uses. They are directionally real and
  are not RP2040 soft-float measurements. `verify_arm.sh` establishes this
  code *builds* for the target and what it costs in flash, which is a
  different and much weaker claim than knowing how fast it runs there. Nothing
  here has executed on hardware.
- **`design documentation/Math.md` owns every derivation here.** §1–6 are the
  forward model and its per-frame Jacobian; §7 is the arrowhead normal
  equations and the Schur elimination `schur_normal_equations.h` implements.
  This directory's comments point at it rather than restating it — where a
  comment and `Math.md` seem to disagree, `Math.md` is right.
