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
axes, which is the result the migration was originally trying to get. (This
comparison was measured at the project's then-current `-O2`; the project
has since moved to `-O3` project-wide, so current flash/RAM numbers live in
`Performance.md`, not here — the `-O2`-vs-`-O2` Eigen/BLA comparison above
is still valid on its own terms, just not directly comparable to the
current build.)

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

## Extended: porting the cross-magnet refactor onto BLA

`origin/experimental` did a "full refactor of the forward-model call chain"
(every sensor now sees all three magnets by superposition, not just its
paired one) entirely against Eigen, independently of everything above — it
branched before this migration and never merged it. Re-derived onto BLA
rather than mechanically rebased: read the new code, ported it using the
same idioms this file already established, and re-measured every
hand-unrolling decision fresh rather than assuming the old numbers carried
over. Full context in the branch's plan notes; summarized here for the
file-by-file record this document already keeps.

- **New abstractions, ported using the existing idiom set**:
  `magnet_local_model.{h,cpp}` gained `MagnetModel::place()`, the
  `MagnetPlacement` struct (`near_approx_world`/`far_approx_world`), and the
  brand-new free function `dipole_field(m, r, J)` — none of these existed at
  the previous pass, so there was nothing to port *from*, just fresh BLA code
  written against the conventions already in place (`dot()` for
  `.squaredNorm()`/`.dot()`, `.Column()` for `.col()`, `~R` for
  `.transpose()`, direct `Mat3(...)` fill for `<<`-init).
- **`Mat2` promoted to `math3D.h`**: `BicubicField::evaluate()`'s signature
  collapsed to a single `Mat2&` out-param (was three separate `Vec2&`s), and
  `MagnetModel::evaluate()` needs the same type for its local
  `J_cylindrical` — two genuinely independent call sites, unlike
  `Vector6f`/`Matrix6x6f` (still `solve_pose.cpp`-local, still
  single-use-site even after this pass).
- **`.x()`/`.y()`/`.z()` added to vendored BLA** (`BasicLinearAlgebra.h`'s
  `MatrixBase`, next to its `operator()(i,j=0)` pair). This pass brought in
  enough more Eigen-idiom source in one go (`dipole_field`,
  `MagnetPlacement`'s methods, three new `test_jacobian.cpp` tests) that
  hand-translating every `.x()/.y()/.z()` call site to `(0)/(1)/(2)` — the
  approach the original migration bullet above took — was worse than closing
  the accessor gap once, in the vendored copy we already patch. Const/non-
  const pair mirroring `operator()`, `static_assert`-guarded on shape so
  misuse (`.z()` on a `Vec2`) is a compile error, zero runtime cost (inlines
  to the same `operator()` call). Existing `(i)`-style call sites elsewhere
  are untouched; both idioms are fine going forward.
- **`outer()` added to `math3D.h`**, alongside `dot()`/`cwise_product()`/
  `cwise_sqrt()` in the same "small BLA gap-fillers" group — needed once
  `dipole_field`'s hand-rolled Jacobian was un-hand-rolled into the clean
  outer-product form (see `TODO/Performance.md`'s cross-magnet-refactor
  pass for the measurement).
- **`solve_pose.{h,cpp}`**: `measured_fields` collapsed from a 3-element
  `Vec3[3]` to a single `const Vector9f&`, letting the three
  `Submatrix<3,1>()` subtraction lines become one `residual -=
  measured_fields;` — compile-checked rather than assumed: both operands are
  genuine `Vector9f` lvalues here (not `Submatrix()` views), so BLA's `-=`
  binds fine, unlike the case this file's API-surface notes above describe.
  `Jacobian_out` (the optional out-param) was dropped upstream on
  experimental, orthogonal to BLA; ported as dropped.
- **Host test env formalized**: `origin/experimental` added its own
  `firmware/host_shims/Arduino.h` + `[env:native_test]`, built for
  Eigen (`ArduinoEigen` needs less off-target surface than BLA does). Ported
  by re-deriving the actual required stub surface against BLA's *current*
  (post-`Printable`-patch) include graph rather than assuming the old ad hoc
  `/tmp` scratchpad's stub list still applied — turned out to be exactly
  `__not_in_flash_func` plus a minimal `Print` (for
  `MatrixBase::printTo()` to typecheck against, even though nothing calls
  it), confirmed by grepping the vendored headers' own `#include`s. This
  formalizes, as a committed environment, what the original migration pass
  above did by hand in a session scratchpad.
- **Controllers/states**: mechanical carry-forward of the same patterns
  already in this file's "what changed" list above (`Vec3::Zero()` →
  `BLA::Zeros<3,1,float>()`, `Mat3::Identity()` → `identity3()`, etc.) —
  `MotionController`, `TelemetryController`, `IdleState` had already dropped
  their `last_jacobian`/`rcond` plumbing upstream on experimental, orthogonal
  to BLA.

## Extended: `constexpr` for `BLA::Matrix`

The last item this file's "Open" section left standing — closed by patching
`BLA::Matrix`'s own constructor rather than working around it.

**What changed**: `firmware/lib/BasicLinearAlgebra/ElementStorage.h`'s
variadic fill constructor (`Matrix(DType head, TAIL... args)`) and
`operator()` are now `constexpr`; `BasicLinearAlgebra.h`'s
`MatrixBase() = default` is marked explicitly `constexpr` too, since a
derived `Matrix`'s constexpr constructor implicitly default-constructs it.
The fill constructor's body used to walk a recursive `FillRowMajor` helper,
writing each element via `operator()` inside the constructor *body* — not
usable in a constexpr constructor, because the standard requires every
member be initialized through the *mem-initializer list*, not just
assigned to in the body. Replaced with mem-initializer aggregate-list
syntax instead: `storage{head, args...}`. Args are already in row-major
order, matching `storage`'s own layout, and — matching FillRowMajor's old
zero-fill base case — any cells past the last argument are zero-initialized
by ordinary C++ aggregate-init rules, for free. `FillRowMajor` itself is
gone; nothing else called it. The plain `Matrix() = default` (no
mem-initializer, used everywhere a matrix is about to be overwritten
in-place, e.g. `solve_pose.cpp`'s locals) was deliberately left alone —
giving it a default member initializer to make *it* constexpr-eligible too
would mean every default-constructed matrix pays for a zero-fill it
usually doesn't need.

### The actual payoff: `BICUBIC_INTERPOLATION_TABLE`, not the basis matrix

Going in, the assumed motivating case was `BicubicField.cpp`'s small 4×4
Catmull-Rom basis matrix — that's the one the file's own comment named. It
turned out to be a red herring for runtime cost (see below), and the real
win was somewhere this document hadn't been pointing at:
`magnet_model_table.cpp` (generated, not hand-edited —
`generate_bicubic_table.py`) defines `BICUBIC_INTERPOLATION_TABLE` as a
`const Vec2[NZ][NR]` — **4,641 individual `Vec2(x, y)` constructions**, all
literal hex-float arguments, going through the exact constructor just
patched. Before this pass, since that constructor wasn't `constexpr`, the
C++ object model required this to be **dynamically initialized**: a real
function (`_GLOBAL__sub_I_BICUBIC_INTERPOLATION_TABLE`, confirmed with
`nm`) ran before `main()` and called the `Vec2` constructor 4,641 times,
writing the results into a `.bss`-resident (RAM) array — confirmed via
`nm`, symbol type `B` at a `0x2000...` address (RP2040 SRAM). This is
exactly the "4,641-entry bicubic table... 37,128 bytes" this document's own
"Why vendored" section already measured as a RAM cost, back when it was
diagnosing the `Printable` vtable regression — it just hadn't been named as
"paying for dynamic initialization" until now.

Once the constructor is `constexpr`, the same declaration (`const`, not
even `constexpr` itself — this is the C++ standard's ordinary "constant
initialization" upgrade for a `const` global whose initializer happens to
be a constant expression, not a new keyword anywhere in
`magnet_model_table.cpp`, which stays untouched) becomes eligible for
**static, compile-time initialization**: the whole table is placed directly
in `.rodata`, no runtime construction, symbol type `R` at a `0x1000...`
address (RP2040 flash, confirmed via `nm`) — and the
`_GLOBAL__sub_I_BICUBIC_INTERPOLATION_TABLE` static-initializer function is
gone entirely, along with whatever code was needed to run 4,641 constructor
calls.

**Measured (clean rebuild, `rm -rf .pio/build .cache`, real
`-e seeed_xiao_rp2040` build, before vs. after this pass, nothing else
changed): Flash 328,208 B → 234,472 B (−28.6%, −93,736 B), RAM 70,888 B →
33,648 B (−52.5%, −37,240 B)** — the RAM delta lands almost exactly on the
table's own 37,128-byte size (the small remainder is other minor layout
shifts), and the flash delta is the removed constructor-call code, which is
far more verbose per byte than the raw packed float data it replaced. This
is a large, real, structural win, not a rounding artifact — reproduced
identically across three independent from-scratch rebuilds before being
trusted (an initial non-`rm -rf`'d build gave a suspicious result that
turned out to just be evidence of exactly this effect, not a caching bug).

**Open caveat, not verified**: the table used to live in RAM (uniformly
fast access); it now lives in flash, read through the RP2040's XIP cache.
The bicubic lookup pattern (a local 4×4 window that moves smoothly
frame-to-frame under hot-starting) should stay cache-friendly, but this is
reasoning, not a measurement — nothing here confirms `evaluate()`'s
per-call latency didn't regress, and that's exactly the kind of thing this
document's own "on-device verification" bullet (below) already flags as
open. Flagging rather than asserting.

### The basis matrix: not free, but not the reason to want this

`BicubicField.cpp`'s own local Catmull-Rom `basis` matrix — the case this
comment used to name as the motivation — was made `static constexpr` too,
now that the constructor allows it. **Measured, not assumed: no codegen
change.** Isolated ARM disassembly (real project flags, same method as
`TODO/Performance.md`'s passes) of `BicubicField::evaluate` compiled both
ways — `static constexpr Mat4 basis` vs. the previous plain `const Mat4
basis` — produced a **byte-for-byte identical `.text` section**, not just
an equal call count. `-O3` was already constant-folding this matrix before
the language change: every element is a compile-time literal and nothing
mutates it after construction, which is exactly the case GCC's ordinary
constant propagation already covers without needing the C++ abstract
machine to guarantee it. So the ~7.3-7.5% cost this matrix form still
carries over the hand-expanded scalar form it replaced
(`TODO/Performance.md`'s fourth and fifth passes) is confirmed to live in
the generic 4×4×4×1 multiply itself — which doesn't know most of `basis`'s
entries are 0/±1 the way hand-written scalar code does — not in any
per-call construction cost, which turns out never to have been real at
`-O3`. `constexpr` closes a real language gap here (the type is now usable
where a constant expression is required, and its compile-time-ness is a
guarantee rather than an optimizer's discretion) but this specific call
site's runtime cost is unchanged and not expected to be revisited by this
change. The lesson this leaves for the file: a *local* matrix built from
literals was already free under `-O3` regardless of the C++ keyword; a
*global* one wasn't, because dynamic-vs-static initialization is a language
rule the optimizer can't route around no matter how obviously-constant the
values are.

**Verification**: a standalone `static_assert(basis(0,0) == 0.0f, ...)`-style
check confirms `BLA::Matrix`'s fill constructor is now genuinely usable in
a constant expression, not just accepted syntactically. `pio test -e
native_test` (all 8 cases, unchanged pass/fail) and `pio test -e
seeed_xiao_rp2040_test --without-uploading --without-testing` both
pass/build clean (one pre-existing `-Wnarrowing` warning newly surfaced at
`SensorController.cpp`'s `Vec3(0.1, 0.1, 0.1)` call — aggregate-list
initialization checks narrowing more strictly than the old body-assignment
form did; the double-to-float narrowing itself isn't new, just now visible;
not fixed here, out of scope for this pass). Real ARM build numbers above.

## Open

- ~~Whether BLA actually inlines better than Eigen did, and whether any of
  `TODO/Performance.md`'s hand-unrolled optimizations can come back out now
  that Eigen's gone~~ — done, see `TODO/Performance.md`'s fourth pass and
  later ones: BLA's `CholeskyDecompose`/`CholeskySolve`/`dot()` inline
  completely (Eigen's equivalent never did). **All four checked candidates
  (the original three, plus `dipole_field`'s Jacobian, new in the
  cross-magnet refactor) are on their clean `BLA` forms.** `H = JᵀJ` was a
  pure inlining-level artifact of the project's `-O2` and is free at `-O3`
  (which the project now builds at project-wide) — re-verified byte-for-byte
  identical in the cross-magnet-refactor pass, since `jacobian`'s 9x6 shape
  never changed. The written-out skew products needed `-O3` and
  `-ffinite-math-only` together to reach parity — a scoped-pragma attempt
  fell short on the real build and was reverted, but going
  `-ffinite-math-only` project-wide (with a bit-level `is_finite_bits()`
  check in `math3D.h` protecting the NaN/Inf safety net from that flag)
  reached exact parity; re-verified again under the cross-magnet refactor's
  3-way-summed call shape (still exact parity, 60 calls either way — the
  "changed surrounding code might break this" worry raised going in did not
  materialize here, but was checked rather than assumed). The bicubic
  basis-weight computation — separate from the *hoisting* question this
  bullet originally meant, see `TODO/Performance.md`'s correction — was also
  un-hand-rolled into a matrix form, but that one is not free: measured
  ~7.5% more soft-float calls than the scalar version it replaced under the
  original three-out-param signature, and ~7.3% (352 vs. 328) re-measured
  under the cross-magnet refactor's collapsed `Mat2&`-out-param signature —
  consistent, not a new regression from the signature change. Kept anyway
  per the standing "remove all hand-unrolling" decision. `dipole_field`'s
  Jacobian (brand-new in the cross-magnet refactor, never assessed before)
  turned out to be the pleasant surprise of the four: the clean unreduced
  outer-product form (`outer()`, new in `math3D.h`) measured 65 calls
  against the hand-rolled 6-of-9-then-mirror form's 67 — not just free,
  slightly cheaper.
- **On-device verification.** Nothing here has run on real hardware. The
  host-native numeric checks (above) are strong evidence of correctness but
  are not a substitute for `pio test -e seeed_xiao_rp2040_test` actually
  uploading and running on the RP2040.
- **The large-perturbation solver divergence noted above** — worth a
  deliberate look (or an explicit "out of scope, hot-start-only" note)
  rather than leaving it as an incidental finding from an ad hoc check.
- ~~`constexpr` for the bicubic table (the thing that originally motivated
  looking at alternatives to Eigen) is still not done~~ — done, see the
  `constexpr` extended section above. `BLA::Matrix`'s literal-list
  constructor is now `constexpr`; the real payoff wasn't the small local
  basis matrix this bullet used to picture, but `magnet_model_table.cpp`'s
  4,641-entry `BICUBIC_INTERPOLATION_TABLE`, which moved from a dynamically-
  initialized RAM array to true compile-time `.rodata`: Flash 328,208 B →
  234,472 B, RAM 70,888 B → 33,648 B. On-device latency impact of the table
  now living in flash instead of RAM is unverified — see that section.
- **Vendored code needs a way to stay in sync with upstream on purpose.**
  There's no process yet for noticing if upstream BLA fixes a bug this copy
  also has, or for re-applying the `Printable` patch if someone re-vendors
  from a newer release. Worth at least a comment pointing back to this file
  from `firmware/lib/BasicLinearAlgebra/BasicLinearAlgebra.h` (already
  added) and, longer-term, a real process if this needs to move past 5.1.0.
