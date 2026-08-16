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
BasicLinearAlgebra` ("BLA"; MIT license, PlatformIO/Arduino registry name
`BasicLinearAlgebra`, pinned to `^5.1`), as an ordinary (non-vendored)
PlatformIO dependency, across every firmware file that touched Eigen types.
`pio run -e seeed_xiao_rp2040`, `-e seeed_xiao_rp2040_unity`, and
`pio test -e seeed_xiao_rp2040_test --without-uploading --without-testing`
all build clean. **Vendoring BLA and pursuing `constexpr` (for the bicubic
table or anywhere else) remain deliberately out of scope** — real follow-on
work, not bundled into this pass.

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
- **Real ARM build size**, `-e seeed_xiao_rp2040`, both before and after,
  clean rebuild: **Flash 304,048 → 338,776 B (+34,728 B, +11.4%), RAM
  64,516 → 82,296 B (+17,780 B, +27.6%)**. This is a real regression, not
  noise — measured, not estimated, per `TODO/Performance.md`'s own "prefer
  measuring to estimating" rule. Not yet root-caused (candidates: BLA's
  `operator*`/`Cholesky` codegen inlining worse than hoped on this target,
  the migration's own small hand-written helpers, or something else) --
  the disassembly-level investigation `TODO/Performance.md` used for its
  own passes hasn't been repeated here. **Open**, see below.

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

- **Root-cause the +11.4% flash / +27.6% RAM regression.** Not expected,
  not yet investigated at the disassembly level the way
  `TODO/Performance.md`'s own passes were. Worth checking whether BLA's
  `CholeskyDecompose`/`CholeskySolve`/`operator*` inline better or worse
  than Eigen did on this target before concluding anything about whether
  the migration was a net win.
- **On-device verification.** Nothing here has run on real hardware. The
  host-native numeric checks (above) are strong evidence of correctness but
  are not a substitute for `pio test -e seeed_xiao_rp2040_test` actually
  uploading and running on the RP2040.
- **The large-perturbation solver divergence noted above** — worth a
  deliberate look (or an explicit "out of scope, hot-start-only" note)
  rather than leaving it as an incidental finding from an ad hoc check.
- `constexpr` for the bicubic table (the thing that originally motivated
  looking at alternatives to Eigen) is still not done. BLA's `Matrix`,
  `RefMatrix`, and `MatrixTranspose` are all plain-array/reference-based
  with no expression-template indirection, which should make this a much
  smaller lift than it ever was against Eigen -- but that's an argument for
  why it's tractable, not a statement that it's been attempted.
- Vendoring BLA (to allow the `constexpr` work above) is still not done,
  per this pass's explicit scope.
