# Migrate firmware math from Eigen to BasicLinearAlgebra

`TODO/Performance.md` found that Eigen's abstraction actively costs this
target: solver operations don't inline on the RP2040 build (real `bl` calls
into flash-resident Eigen internals, paying a linker-veneer hop from the
RAM-pinned solver), and `jacobian.transpose() * jacobian` was dispatching
into Eigen's general blocked-GEMM kernel for what's actually a 9×6 product
— worked around there by hand-rolling 21 `.dot()` calls instead of trusting
the library. Separately, `BicubicField.cpp` noted "Because Eigen matrix
can't be constexpr" as the reason its basis-weight computation can't be
simplified further.

## Decided

Eigen (`ArduinoEigenDense.h`) has been replaced with `tomstewart89/
BasicLinearAlgebra` ("BLA"; MIT license), **vendored** into
`firmware/lib/BasicLinearAlgebra` (the 5.1.0 release) and patched, across
every firmware file that touched Eigen types. `pio run -e seeed_xiao_rp2040`,
`-e seeed_xiao_rp2040_unity`, and
`pio test -e seeed_xiao_rp2040_test --without-uploading --without-testing`
all build clean. `constexpr` for the bicubic table remains out of scope —
real follow-on work, not bundled into this pass, though vendoring (done here
for the reason below) is a prerequisite for it and no longer blocks it.

### Why vendored: the RAM/flash regression was `Printable`, not floats-vs-doubles

The first pass (non-vendored, straight off the registry) shipped with a real
regression: Flash +11.4% (304,048→338,776 B), RAM +27.6% (64,516→82,296 B).
Root-caused, not guessed: `BLA::MatrixBase` inherits Arduino's `Printable`
(`virtual size_t printTo(Print&) const = 0`, purely so `Serial.print(myMatrix)`
works — nothing in this codebase actually calls it). **Any virtual function
makes every instance of the class
carry a hidden vtable pointer** — 4 bytes per `BLA::Matrix`, which Eigen's
plain-aggregate types never had. Confirmed with `nm` on the ELF: every
`BLA::Matrix<...,...,float>` specialization in use had its own emitted
`vtable for BLA::Matrix<...>` symbol, and the 4,641-entry bicubic table
(`BICUBIC_INTERPOLATION_TABLE`, `NZ*NR` `Vec2`s) measured 12 bytes/entry
instead of the 8 (2 floats) it should be — **+18,564 bytes on that one
array alone, bigger than the entire measured RAM regression.** Confirmed
this wasn't fixable with flags: `--gc-sections` (already on, via the
arduino-pico core's default linker flags) can't remove it because every
constructor of a polymorphic type writes `&vtable` into the object — that's
a real, live reference, not dead code. `-fno-rtti` (also already on) strips
`typeinfo`, a different mechanism from vtables/virtual dispatch entirely.
Neither flag, nor LTO/devirtualization, can shrink `sizeof(BLA::Matrix<...>)`,
because the vtable pointer is a physical object member mandated by the
Itanium C++ ABI the instant a class has any virtual function — that's a
data-layout fact, not something an optimizer is allowed to remove.

Fix: vendored BLA 5.1.0 into `firmware/lib/BasicLinearAlgebra` and dropped
`: public Printable` from `MatrixBase` plus the now-illegal `final` on
`printTo()` (`final` requires actually overriding a virtual, which no
longer exists) in `BasicLinearAlgebra.h`. `printTo()` itself is kept as an
ordinary non-virtual member — still callable directly, just not through
`Printable`'s polymorphic interface. Nothing else touched. Re-measured:
**Flash 298,976 B (−1.7% vs. the pre-migration Eigen baseline, not just vs.
the buggy BLA build), RAM 63,348 B (−1.8% vs. baseline)** — confirmed via
`nm` that zero `vtable for BLA::*` symbols remain, and the bicubic table is
back to exactly 8 bytes/entry (37,128 B total). BLA now beats Eigen on both
axes, which is the result the migration was originally trying to get.

### Verification actually performed

No hardware is available in this environment, so nothing here is verified
on-device. What *was* verified, natively on the host, self-contained scripts
in `/tmp` (not committed — not a reusable harness, just what this pass used):

- **`firmware/test/test_jacobian.cpp` compiled and run natively** (not just
  compiled): all 5 test cases pass, including the 384-point forward-model
  grid sweep. This is the project's stated safety net for the whole solver
  chain, and it passed for real, not just "the build succeeded."
- **`solve_knob_pose()` itself** (not exercised by `test_jacobian.cpp`,
  which only covers the forward model/Jacobians) was checked with a
  synthetic ground-truth pose: generate "measured" fields from a known
  (t, R), perturb the starting guess by a realistic hot-start-sized amount
  (~0.03mm / ~0.3°, matching the real 50Hz hot-started operating regime),
  and confirm the LM loop (BLA `CholeskyDecompose`/`CholeskySolve`,
  `dot()`, `exp_so3` rotation update) converges back to the ground truth:
  position error ~2×10⁻⁵ mm, rotation error ~9×10⁻⁶ (axis-angle proxy).
  **Caveat found along the way**: the same setup with a *large* single-shot
  perturbation (~0.7mm / ~3.5°) diverges badly. Not reproduced against the
  pre-migration Eigen code, so this is not confirmed as a regression — it
  plausibly reflects the fixed-damping (`LAMBDA=0.02`) LM formulation's
  actual convergence basin, which the real system never leaves because it's
  hot-started every frame. Flagging rather than asserting, since it wasn't
  cross-checked.
- **Real ARM build size**, `-e seeed_xiao_rp2040`, measured at each stage:
  Eigen baseline 304,048 B flash / 64,516 B RAM → non-vendored BLA (bug)
  338,776 B / 82,296 B → **vendored+patched BLA 298,976 B / 63,348 B**. See
  "Why vendored" above — root-caused and resolved, not just re-measured.

### API surface actually available (discovered by reading the pinned
### 5.1 release, not just the README or upstream `master`)

The design phase researched `master`, which drifted from what `^5.1`
actually ships. Corrections made during implementation:

- **No `DotProduct` in 5.1** (added to `master` after the 5.1 tag, not in
  any published version). Hand-rolled `dot()` in `math3D.h` instead.
- **No named `.x()`/`.y()`/`.z()` accessors, and no `operator[]`** at all —
  only `operator()(i, j=0)`. Every `v.x()`/`v[i]`-style call site in the
  tree (not just files that said "Eigen" — `Animations.cpp`,
  `SensorController.cpp`, and a `MotionController.h` default member
  initializer were all missed by grepping for the word "Eigen" and only
  caught by the compiler) had to become `v(i)`.
- **Scalar operators deduce `DType` from *both* operands**, not just the
  matrix side: `mat * (some_double_or_int_expression)` fails to compile
  ("deduced conflicting types for parameter DType") unless the scalar is
  already exactly `float`. Bit this twice — `M_PI` (a `double`) multiplying
  a `Vec3`, and `Config::ZERO_SAMPLES` (an `int`) dividing one. Both need an
  explicit `float(...)`/`static_cast<float>(...)`.
- **`Submatrix()`/`Row()`/`Column()` return temporaries**, and BLA's `+=`/
  `-=` are free functions requiring a non-const lvalue — they can't bind to
  those temporaries. `residual.Submatrix<3,1>(0,0) -= x;` doesn't compile;
  `residual.Submatrix<3,1>(0,0) = residual.Submatrix<3,1>(0,0) - x;` does
  (plain `=` is a `RefMatrix` *member*, which works on temporaries).
- **`CholeskyDecompose(H)` factorizes `H` in place** and reports
  `positive_definite` explicitly, rather than degrading silently the way
  Eigen's `.ldlt()` did. Added an explicit check (treated the same as the
  existing NaN/Inf guard) rather than trusting `CholeskySolve` on a
  possibly-non-SPD factorization.
- **Zero runtime bounds/shape checking, in any build mode** — confirmed by
  reading the whole library: the only `assert`s anywhere are two
  `static_assert`s (compile-time). There is no `eigen_assert` equivalent to
  toggle with `NDEBUG`. `firmware/test/README` updated to say so; the test
  env's `build_unflags = -DNDEBUG` still restores plain `assert()`, but
  there's nothing library-specific left for it to unlock.
- `BLA::Matrix`'s storage is natively row-major (`storage[i*Cols+j]`),
  which happens to match both `float[3][3]`'s layout and numpy's
  convention — `CalibrationParams.h`'s `toMat3()`/`toVec3()` went from an
  `Eigen::Map<...,RowMajor>` reinterpret to a plain per-element copy loop,
  simpler than before.
- `Zeros<Rows,Cols,DType>`/`Eye<Rows,Cols,DType>` are the `::Zero()`/
  `::Identity()` replacements — free-standing types convertible into a
  concrete `Matrix<>`, not static factory methods.
- No `.eval()` equivalent needed anywhere: `operator*` (and the other
  arithmetic operators) return a concretely materialized `Matrix<>` by
  value via a plain nested loop, not a lazy expression template. The one
  place Eigen needed `.eval()` (`R = (dR * R).eval();`, guarding against
  self-aliasing) has no analogous hazard under BLA and is now just
  `R = exp_so3(dw) * R;`.

### What changed, file by file

- `math3D.h` is now the single BLA-based source of truth for
  `Vec2`/`Vec3`/`Mat3`/`Vector9f`/`Matrix9x6f`/`Matrix3x6f`, plus the
  hand-written gap-fillers (`dot`, `all_finite`, `cwise_product`,
  `cwise_sqrt`, `identity3`) and the canonical `exp_so3()` (ported out of
  `test_jacobian.cpp`, which now calls the shared one instead of its own
  copy; `solve_pose.cpp` uses it in place of `Eigen::AngleAxisf`).
  Duplicate `using Vector9f = ...`/`Matrix9x6f = ...` redeclarations that
  used to sit independently in `solve_pose.cpp`, `MotionController.h`,
  `test_jacobian.cpp`, and `magnet_model_table.h` are gone — one owner.
- `BicubicField.{h,cpp}`, `magnet_local_model.{h,cpp}`, `virtual_sensor.cpp`,
  `forward_model.{h,cpp}`: mechanical port (`.transpose()`→`~`,
  `.block<>()`→`.Submatrix<>()`, `<<` comma-init→variadic constructor,
  `.asDiagonal()`→direct fill at its one call site).
- `CalibrationParams.h`: `toMat3()`/`toVec3()` simplified as above.
- `solve_pose.cpp`: the highest-risk file, see the API-surface notes above.
- `MotionController.{h,cpp}`, `SensorController.cpp`, `Animations.cpp`,
  `CalibrationStorage.cpp`: caught by the *compiler*, not the original
  audit, since none of them said the word "Eigen" anywhere — they just used
  Vec3/Mat3 with Eigen-specific member syntax. `.determinant()` →
  `BLA::Determinant(...)`.
- `platformio.ini`: `ArduinoEigen` dependency dropped, `EIGEN_NO_MALLOC`
  removed (BLA structurally cannot allocate — plain fixed-size array
  storage, no flag needed), `-DNDEBUG`'s comment corrected to not claim
  it's Eigen-specific.

## Open

- ~~Whether BLA actually inlines better than Eigen did, and whether any of
  `TODO/Performance.md`'s hand-unrolled optimizations can come back out now
  that Eigen's gone~~ — done, see `TODO/Performance.md`'s fourth pass:
  BLA's `CholeskyDecompose`/`CholeskySolve`/`dot()` inline completely
  (Eigen's equivalent never did), but all three unrolling candidates
  checked (symmetric `H = JᵀJ`, written-out skew products, the bicubic
  basis-weight hoist) turned out to have independent algorithmic
  justification and stay as-is, backed by measurement not assumption.
- **On-device verification.** Nothing here has run on real hardware. The
  host-native numeric checks (above) are strong evidence of correctness but
  are not a substitute for `pio test -e seeed_xiao_rp2040_test` actually
  uploading and running on the RP2040.
- **The large-perturbation solver divergence noted above** — worth a
  deliberate look (or an explicit "out of scope, hot-start-only" note)
  rather than leaving it as an incidental finding from an ad hoc check.
- `constexpr` for the bicubic table (the thing that originally motivated
  looking at alternatives to Eigen) is still not done. Vendoring -- now
  done, above -- was the blocker for touching BLA's own source to attempt
  it; `Matrix`, `RefMatrix`, and `MatrixTranspose` are all plain-array/
  reference-based with no expression-template indirection, which should
  make this a smaller lift than it ever was against Eigen, but that's an
  argument for why it's tractable, not a statement that it's been attempted.
- **Vendored code needs a way to stay in sync with upstream on purpose.**
  There's no process yet for noticing if upstream BLA fixes a bug this copy
  also has, or for re-applying the `Printable` patch if someone re-vendors
  from a newer release. Worth at least a comment pointing back to this file
  from `firmware/lib/BasicLinearAlgebra/BasicLinearAlgebra.h` (already
  added) and, longer-term, a real process if this needs to move past 5.1.0.
