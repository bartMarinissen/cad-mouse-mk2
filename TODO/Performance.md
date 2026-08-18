# solve_pose performance

**Everything through the third pass below was measured against the
Eigen-based solver.** `TODO/eigen-to-bla-migration.md` has since replaced
Eigen with `BasicLinearAlgebra` — the RAM/flash regression mentioned
below was root-caused and fixed (a `Printable` vtable pointer, not
floats-vs-doubles or anything algebraic), and the fourth pass at the
bottom of this document re-runs this document's own measurement
methodology against BLA specifically: does it inline better than Eigen
did (yes, substantially), and does that mean any of the hand-unrolling
below can come back out (no — measured, not assumed, for all three
candidates). The Eigen-specific findings in the first three passes (the
non-inlining, the blocked-GEMM dispatch for `jacobian.transpose() *
jacobian`) describe why those rewrites happened and remain accurate
history; they're no longer a description of the current library.

The whole loop runs at 50Hz, and `solve_knob_pose()` (`firmware/src/magnet_model/solve_pose.cpp`)
currently eats a little under half that budget — call it 10ms. Goal is to get
it down toward ~4ms if we can. This file is the investigation notes: assembly-
verified hypotheses on where that time actually goes, not yet a fix plan.

Platform context that matters for everything below: Seeed Xiao RP2040, dual
Cortex-M0+ @ 133MHz, **no hardware FPU**. Every `float` operation is a software
subroutine call. `solve_knob_pose`, `ForwardModel::evaluate`,
`VirtualSensor::evaluate`, `MagnetModel::evaluate`, and `BicubicField::evaluate` are
all marked `__not_in_flash_func` (placed in RAM), presumably to dodge flash
XIP wait-states on this hot path.

## How to measure, before you measure anything

Both of these were learned the hard way in the passes below, and both produce
confident wrong answers rather than obvious failures.

- **Static `bl`-counting can give the wrong sign.** Tallying call targets in a
  disassembly counts work inside a loop kernel once regardless of how many
  iterations it runs, so replacing an Eigen loop kernel with straight-line
  scalar code looks like a regression when it is a 20% win. It flipped the
  sign on the third pass's result. Cross-check with
  `valgrind --tool=callgrind` on a host build before believing a static tally.
- **Prefer measuring to estimating.** The hand-counted flop estimates this
  document originally carried were wrong enough to be replaced wholesale. If a
  number goes in here, say how it was obtained — the sections below are
  labelled by method for exactly this reason.

## Ruled out / already fine

- **Bicubic table location**: `BICUBIC_INTERPOLATION_TABLE` is at `0x20003278`
  per `firmware.map` — RAM, not flash. Confirmed, not a lead.
- **r² parameterization** of the bicubic table (to save the `sqrtf` in
  `MagnetModel::evaluate`, `firmware/src/magnet_model/magnet_local_model.cpp:21`):
  rejected. It would concentrate resolution away from the high-curvature
  region near the magnet, which is exactly where accuracy matters most. Only
  worth revisiting alongside a non-uniform grid + Hermite splines, which has
  its own (painful) lookup cost — not pursuing now.
- **Quasi-Newton** (reuse/update the Jacobian across a couple of iterations
  instead of recomputing the full rotated Jacobian every time): plausible
  lever, parked for later, not investigated yet.

## Blocking prerequisite: the Statistics TODO — RESOLVED

Before adding any new solver telemetry (iteration count, per-phase timing),
this doc called out an existing TODO at `solve_pose.cpp:21`: `solve_knob_pose`
always allocated local `residual`/`jacobian` and copied them into
`*residual_out`/`*Jacobian_out` at the end, rather than writing into those
output pointers directly when non-null. `MotionController::read_pose` always
passes them (`Config::statistics` is `true`), so that copy was live on every
call. Fixed as a side effect of the "third optimization pass" below
(`solve_knob_pose` now writes into the caller's buffers directly), so new
solver telemetry is no longer blocked on it — see "Open next steps".

## Method: disassemble the real binary, don't guess

Built with `pio run -e seeed_xiao_rp2040`, then disassembled
`firmware.elf` (`arm-none-eabi-objdump -d -C`) and walked the actual call
graph starting at `solve_knob_pose`. Two things made naive call counting
useless until handled:

1. **Linker veneers**: RAM code calling flash code (or vice versa) goes
   through a synthetic thunk (`push {r0}; ldr r0,[pc,#N]; mov ip,r0; pop {r0};
   bx ip`), whose real target is a `.word` literal embedded right after the
   thunk. Naively following `bl` targets stops at the veneer; had to extract
   the embedded address and resolve it against the symbol table to find out
   what's actually being called.
2. **Dead paths pollute reachability**: `eigen_assert` bounds-checks compile
   to real (conditional, never-taken-in-practice) calls to `assert_func` —
   meaning **`NDEBUG`/`EIGEN_NO_DEBUG` isn't defined in this "release"
   build** — and the `if (!dx.allFinite())` branch in `solve_pose.cpp` pulls
   in `Serial.println`/`delay`. Both are reachable in the static call graph
   but never execute on the success path, and naively expanding them explodes
   into unrelated code (float-formatting routines, etc.). Excluded both from
   the tally below; noted the `NDEBUG` gap as a separate, easy, unrelated fix.

## The actual finding: Eigen isn't inlining here

This is the more important result, not just "soft float is slow." Operations
like `H = jacobian.transpose() * jacobian`, the 6x6 LDLT solve, and even
plain 3x3 rotation products do **not** compile to inline arithmetic — they're
real `bl` calls into separate out-of-line functions
(`Eigen::internal::generic_dense_assignment_kernel<...>::assignCoeff`,
`Eigen::internal::ldlt_inplace<1>::unblocked`, etc.), each with its own
prologue/epilogue.

And critically: those Eigen helper functions live in **flash**
(e.g. `assignCoeff` for the 9x6 Jacobian sits at `0x10030766`), while the
calling functions are pinned to **RAM** via `__not_in_flash_func`. So nearly
every one of these calls pays a veneer hop (register push, indirect load,
`bx`) on top of the soft-float cost itself — the opposite of what
`__not_in_flash_func` was presumably trying to buy. The live `eigen_assert`
calls plausibly contribute too: GCC's inliner penalizes call-containing
function bodies, and every Eigen leaf op already "contains a call" to its
bounds-check.

## Operation tally (per single LM iteration)

Counts are leaf soft-float/memcpy subroutine calls, resolved through the
whole call tree for **one** pass through the loop body in `solve_pose.cpp:26`
(i.e. not yet multiplied by iteration count). Internal loops inside a given
function (the bicubic row loop, the 3-sensor loop) are folded in already since
they're real runtime loops, not unrolled — see caveats below.

| Layer | fmul | fadd | fsub | fdiv | sqrt/sin/cos | cmp | i2f/f2iz | memcpy | total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `cubic()` (1 call) | 20 | 10 | 10 | – | – | – | – | – | 40 |
| `cubic_deriv()` (1 call) | 16 | 9 | 10 | – | – | – | – | – | 35 |
| `point_or_ghost()` (1 call, worst case) | – | 4 | 4 | – | – | – | – | – | 8 |
| **`BicubicField::evaluate`** (own + 4×`point_or_ghost` + 3×`cubic` + 2×`cubic_deriv`) | 98 | 64 | 70 | – | – | 2 | 6 | 20 | **260** |
| **`MagnetModel::evaluate`** (+ 1× Bicubic) | 119 | 67 | 71 | 1 | 1 sqrt | 3 | 6 | 20 | **288** |
| **`VirtualSensor::evaluate`** (+ 1× MagnetModel) | 353 | 223 | 89 | 1 | 1 sqrt | 3 | 6 | 29 | **705** |
| **`ForwardModel::evaluate`** (+ 1× VirtualSensor; sensor-loop body counted once, real loop runs 3×) | 353 | 223 | 89 | 1 | 1 sqrt | 3 | 6 | 29 | **705** |
| **`solve_knob_pose`** (+ 1× ForwardModel; LM-loop body counted once) | **504** | **344** | **140** | **6** | 3 sqrt, 1 sin, 1 cos | 25 | 6 | 34 | **≈1064** |

So **one LM iteration ≈ 1,064 soft-float/library calls**. `MAX_ITER = 10`
(`solve_pose.cpp:18`), so a single `solve_knob_pose()` call is issuing on the
order of **10,000 individually-dispatched soft-float calls** in the worst
case — fewer if it converges early, which is exactly why iteration-count
telemetry (blocked on the Statistics TODO above) is the next thing worth
having real numbers on.

**Caveats:**
- `point_or_ghost()`'s 8-op figure is the worst case (both `i_out` and
  `j_out` true, i.e. a grid-corner ghost point); an interior stencil point
  costs 0 there. Real cost is state-dependent on how close the current pose
  is to the bicubic grid's boundary.
- `ForwardModel::evaluate` shows identical totals to `VirtualSensor::evaluate`
  because `ForwardModel::evaluate` itself does no arithmetic (pure block
  assembly) — all its cost is the nested `VirtualSensor::evaluate`, counted once
  per the static loop body even though it runs 3× per call.
- These are static-reachability counts through the compiled binary, not a
  cycle-accurate profile. They tell us *what* is being called and how often
  structurally, not wall-clock time per call — useful for ranking where the
  work is, not for predicting the final ms number directly.

## Results after the first optimization pass

Three changes went in together, so the win below can't be cleanly
attributed to any single one from static analysis alone — but the measured
result: **solve now sustains up to 134Hz standalone (~7.5ms/call), and the
whole loop is at 80Hz (12.5ms)**, up from the ~50Hz/~10ms-per-solve baseline
this doc started from. Good progress toward the ~4ms (250Hz) goal, not there
yet.

What changed:
- `-DNDEBUG` added to `build_flags` in `platformio.ini` — strips the live
  `eigen_assert` calls flagged above.
- `build_unflags = -Os` added to `platformio.ini`. This one was a surprise
  find: something (framework defaults, most likely) was silently appending
  `-Os` after our explicit `-O2`, and since GCC honors the *last* `-O` flag
  on the command line, the build had been compiling this whole hot path for
  size, not speed, the entire time — `-Os` also disables/discourages several
  of the inlining paths `-O2` enables. This plausibly explains a lot of the
  "Eigen isn't inlining" finding above on its own.
- Sensor gain moved out of the solver's hot path: `VirtualSensor` no longer carries
  a `sensor_gain` member, and `VirtualSensor::evaluate` (`virtual_sensor.cpp`) dropped the
  three gain multiplies (`B_field_global = sensor_gain * ...`, two
  `J.block<3,3>(...) = sensor_gain * ...`) it used to do on every call — i.e.
  every sensor, every LM iteration. Gain correction now happens once per raw
  sensor read, in the new `SensorController::read_mT()`
  (`firmware/src/controllers/SensorController.cpp`), before the solver ever
  sees the data.

Rebuilt and re-disassembled to sanity-check the inlining hypothesis
specifically. The hot functions did change shape in a way consistent with
more code getting pulled inline rather than shelled out to separate Eigen
helper functions — e.g. `ForwardModel::evaluate`'s own call count collapsed
from 14 `bl`s to 1 (down to just the `VirtualSensor::evaluate` call itself), and
`MagnetModel::evaluate` shrank (206→175 instructions) with fewer calls,
consistent with the `assert_func` removal. `VirtualSensor::evaluate` and
`solve_knob_pose` actually grew in instruction count and in-body call count —
not a contradiction: that's what it looks like when a helper that used to be
a separate out-of-line function (with its own prologue/epilogue and, per the
finding above, its own RAM→flash veneer hop) gets inlined directly into the
caller instead — the caller's body gets bigger and shows more direct
`__wrap_fmul`/`fadd` calls, but there's one fewer function-call hop (and one
fewer veneer) in the chain per operation. Because `virtual_sensor.cpp` and
`solve_pose.cpp` also changed in this same span, this is a supporting
signal, not a clean isolated proof — the timing measurement above is the
number to trust.

## BicubicField::evaluate rewrite (second optimization pass)

The `BicubicField::evaluate` row in the tally above (260 total, 20 `memcpy`,
`point_or_ghost`/`cubic`/`cubic_deriv` as separate out-of-line calls) was
exactly an instance of the "Eigen isn't inlining here" problem this doc
already identified, plus per-fetch bounds-checking branches on top. Rewrote
it in three commits (`79601e0`, `6a0495e`, `2d4ab99` on
`claude/bicubic-field-rp2040-optimize-tmcqof`, based on `experimental`; since
merged into `experimental` — see "Open next steps" below):

1. **Restrict the stencil** to `i0 ∈ [1, NR-3]`, `j0 ∈ [1, NZ-3]` so the
   4-point stencil always lands inside the real grid — deletes
   `point_or_ghost`/`raw` and their bounds-check branches entirely, replacing
   16 branchy ghost-node fetches/call with 16 unconditional indexed loads.
2. **Hoist the Catmull-Rom basis** into 4 scalar weights per axis computed
   once (instead of recomputed per row via `cubic()`/`cubic_deriv()`), and
   replace cubic ghost-extrapolation outside the grid with linear
   extrapolation (clamp `t`/`u` to `[0,1]`, add `overshoot × weight'(tc)`).
   Deletes `cubic`/`cubic_deriv`.
3. **Exploit r=0 axis symmetry**: `i0` was clamped to a minimum of 1 by step
   2, which pushed the patch touching the physical symmetry axis (r in
   `[0, dr)` — a commonly-hit region, since the tracked object is often near
   a magnet's center) into the rough linear-extrapolation fallback. Since
   `Br` is odd and `Bz` is even in r for this axisymmetric field, the
   stencil's missing virtual node at column -1 is *exactly* the mirror of
   column 1, not an approximation — allowing `i0=0` with that mirrored node
   recovers full bicubic accuracy at the axis and yields `Br(0,z)=0` /
   `dBz/dr(0,z)=0` automatically, with one branch (once per call, not per
   fetch) to select the mirror path.

**Verification method**: a host-side instrumented harness (scratch-only, not
committed) that tallies dynamic mul/add/sub/branch/call counts per
`evaluate()` invocation, cross-checked against the actual
`earlephilhower/pico-quick-toolchain` GCC + real `ArduinoEigen` (not just an
`apt`/mainline-Eigen stand-in) — this sandbox's egress policy blocks
PlatformIO's own package/library registry, so libraries were pulled directly
from GitHub and the translation unit compiled in isolation rather than
linked into a full `firmware.elf`.

Dynamic op count per `evaluate()` call (host harness, interior point):

| | mul | add | sub | branch | `point_or_ghost`/`raw` calls | `cubic`/`cubic_deriv` calls |
|---|---:|---:|---:|---:|---:|---:|
| before | 215 | 100 | 110 | 70–134 | 16–32 | 11 |
| after step 1 | 215 | 100 | 110 | **6** | **0** | 11 |
| after step 2 | **140** | 90 | **10** | 10 | 0 | **0** |
| after step 3 (non-axis patch) | 140 | 90 | 10 | 10 | 0 | 0 |

Real-toolchain static `bl`-tally (whole compiled object, before/after each
commit — matches the apt/mainline-Eigen stand-in almost exactly once Eigen's
abstraction is fully gone; see caveat below):

| | fmul | fadd | fsub | `memcpy` | branches | object size |
|---|---:|---:|---:|---:|---:|---:|
| original | 42 | 23 | 28 | 20 | 15 | 2164 B |
| step 1 | 42 | 19 | 24 | 20 | 5 | 1816 B |
| step 2 | 78 | 54 | 24 | **0** | 13 | 1996 B |
| step 3 | 92 | 66 | 24 | 0 | 14 | 2408 B |

(Static counts aren't directly comparable to the dynamic table above — a
function inlined into a loop shows its cost once here regardless of
iteration count, which is why fmul/branches appear to rise at steps 2–3 even
though the dynamic per-call cost dropped. See the branch commit messages /
session notes for the full explanation.)

Near the axis specifically, step 3 vs. step 2's linear-extrapolation
fallback: error at r=0 drops from 9.4e-4 to 3.2e-5 (~29x), and `Br(0,z)`,
`dBz/dr(0,z)` come out exactly 0 at every z tested, as an emergent property
of the mirrored stencil rather than a special case.

**Answering the "coax Eigen into inlining further" question above, for this
function specifically: no, and here's the concrete result.** Tried both
`__attribute__((always_inline))` on `cubic`/`cubic_deriv` and
`__attribute__((flatten))` on `evaluate()` against the step-1 code (before
the manual weight-hoisting rewrite):

| | `cubic`/`cubic_deriv` calls | Eigen glue calls | `memcpy` | fmul | size |
|---|---:|---:|---:|---:|---:|
| step 1 baseline | 3+2 | 2 | 20 | 42 | 656 B |
| `always_inline` on the callees | 0 (inlined) | **5** (duplicated) | 20 | 44 | 1168 B |
| `flatten` on `evaluate()` | 0 (inlined) | 2 | 0 | **100** | 1712 B |
| step 2 (manual rewrite) | — (deleted) | 0 | 0 | 78 | 1996 B |

Neither directive gets close: `always_inline` just re-expands the same
unreassociated polynomial at each call site (Eigen glue calls go from 2 to
5, not eliminated); `flatten` does eliminate `memcpy` but appears to trigger
more aggressive unrolling of the 4-iteration row loop, which duplicates real
(non-redundant) work and nearly triples the multiply count. Root cause:
what step 2 needed was recognizing that the same 4 scalar weights can be
factored out and reused across all 4 rows — an algebraic reassociation of a
bilinear form, not a redundancy-elimination or call-graph optimization, so
no inlining directive reaches it. There's a second factor visible in the
Eigen symbol names themselves: Eigen's own assignment mechanism decides
whether to fully unroll an expression into scalar code
(`copy_using_evaluator_LinearTraversal_CompleteUnrolling`) or fall back to a
generic runtime-loop kernel (`call_dense_assignment_loop`), independent of
GCC's inlining decision — the old `cubic()`/`cubic_deriv()` polynomial is
complex enough that Eigen declines to fully unroll it regardless of whether
GCC inlines the enclosing function; the flatter `p0*a0 + p1*a1 + ...` form
from step 2 is simple enough that Eigen's own unroller does collapse it,
which is why `memcpy` and the glue calls vanish only there.

Not yet done: an end-to-end `solve_knob_pose` retiming with this change —
only `BicubicField.cpp` was isolated-compiled/verified, not the full linked
binary (blocked by the same registry access issue noted above). Worth
re-running the wall-clock measurement from the "first optimization pass"
above once this lands, and updating the `MagnetModel`/`VirtualSensor`/`ForwardModel`/
`solve_knob_pose` rows in the operation tally, which all currently still
reflect the old `BicubicField::evaluate` cost.

## Third optimization pass: solver algebra, hot start, instrumentation

Non-interpolation work. Every change below is verified equivalent, not just
"passing": a host harness built Eigen 3.4 against both the pre- and post-change
trees and compared the forward model and end-to-end solves at five poses. Max
differences were **1.2e-6 (field), 3.1e-6 (Jacobian), 2.9e-6 mm (solved
translation), 2.4e-7 (solved rotation)** — float32 round-off. `test_jacobian.cpp`
also passes on the host, 384/384 grid points.

- **`micros()` instead of `millis()`** for the solve timer. At ~7.5ms per call
  the millisecond clock quantised at ±13%, coarser than most changes worth
  measuring. `Statistics::time_tot`/`n_time` are `uint32_t` (a signed int of
  microseconds overflows in about an hour at 80Hz), and `reset()` now clears
  them, which it previously did not.
- **Rotation is hot-started.** `read_pose` took `Mat3 R = Mat3::Identity()`
  every frame under a TODO saying not to, so the solver re-converged the
  rotation from scratch each frame while translation was hot-started. The
  rotation now lives in `MotionController::last_R`. Payoff is data-dependent:
  near-free at rest, largest while the knob is moving; not captured by the
  fixed-iteration benchmark below, which hot-starts every run identically.
- **`R_mag` folded into `R_total`** (`virtual_sensor.cpp`). `R·(R_mag·J·R_magᵀ)·Rᵀ`
  is `R_total·J·R_totalᵀ`; four 3×3 products become two, and the field rotation
  collapses the same way.
- **`R_magᵀ·m` precomputed** into `MagnetModel::magnet_offset_local`, per
  Math.md §3.2 — it is a frozen calibration constant.
- **Skew products written out.** `[a]_x` has a zero diagonal, so the general
  3×3 product spent a third of its multiplies on structural zeros, and
  `−[B]_x` touches six entries rather than nine.
- **`H = JᵀJ` exploits symmetry**: 21 dot products for the lower triangle,
  mirrored, instead of all 36 entries.
- **Dead work deleted.** `Statistics::avg_jacobian` ran a 54-element EMA every
  frame and nothing ever read it; `raw_full` in `read_pose` was assigned every
  frame for a commented-out debug block; `forward_model.cpp` computed an unused
  `R_T`. `solve_knob_pose` also copied 63 floats into its out-params on every
  call — with `Config::statistics` true that copy was live, so it now writes
  into the caller's buffers directly (the TODO at the top of that function).

### Measured, not estimated

The first two optimization passes above stopped at static/host verification
with hand-counted flop estimates. Those estimates turned out to be wrong enough
in places to be worth redoing properly, so this pass was measured two more ways
before landing: a real ARM build through `pio run`, and dynamic instruction
counts via `valgrind --tool=callgrind` on the host build.

**The toolchain is available in a sandbox like this one — an earlier attempt in
this same investigation wrongly concluded otherwise.** `pip install
platformio` then `pio run -e seeed_xiao_rp2040` succeeds end to end, packages
and all. The false negative came from probing `api.github.com` (403, repo-scoped)
and treating that as proof the toolchain was unreachable — but
`toolchain-rp2040-earlephilhower` ships from `github.com/.../releases/download/`,
which is not the API host, and PlatformIO's package manager falls back across
mirrors for the pieces that do go through the registry API. Worth checking this
directly (`pio pkg install`) before assuming a sandboxed environment can't build
the real firmware.

**Dynamic instruction counts, static-linked host build, 4000 hot-started solves
(so process-startup cost is negligible and the count is dominated by the
algorithm), pre-change tree vs this commit:**

| | per solve | vs pre-change |
|---|---:|---:|
| pre-change, `-O2` | 23,139 | — |
| **post-change, `-O2`** | **18,377** | **−20.6%** |

`BicubicField::evaluate`, `MagnetModel::evaluate`, `ForwardModel::evaluate`, and
Eigen's LDLT solve came out **bit-for-bit identical in executed instructions**
between the two trees — proof the solver still takes the same number of LM
iterations on this workload, so the −20.6% is genuinely per-iteration work, not
an accidental change in convergence.

Attribution (same 4000-solve run, instructions by function):

| | pre-change | post-change | delta |
|---|---:|---:|---:|
| Eigen `gebp_kernel` + `gemm_pack` (general blocked matmul) | 16,758,522 | **0** | −16.8M |
| `VirtualSensor::evaluate` | 17,525,448 | 12,260,127 | −5.3M |
| `solve_knob_pose` | 4,877,296 | 9,251,607 | +4.4M |

The symmetric-`JᵀJ` change was the single biggest win, bigger than estimated:
`jacobian.transpose() * jacobian` was dispatching into Eigen's general blocked
matrix-multiply kernel (`gebp_kernel`/`gemm_pack_lhs`, meant for large matrices,
not a 9×6), and replacing it with 21 explicit dot products deletes that whole
code path — hence 16.8M instructions to zero, not a partial reduction. The rise
in `solve_knob_pose`'s own count is that deleted work moving *into* the function
as inlined scalar code rather than a remote call, which is why it shows up
there instead of vanishing outright. `virtual_sensor.cpp`'s own algebra changes landed
smaller than estimated (~5.7% of the pre-change total vs an early ~9.3% guess).

**Real ARM build** (`toolchain-rp2040-earlephilhower`, `pio run -e
seeed_xiao_rp2040`): flash **4,960 B smaller** than pre-change (266,668 vs
271,628 B used); RAM +48 B (61,952 vs 61,904 B, of 262,144 total).

**The static `bl`-tally method this document uses elsewhere gives the wrong
answer for this class of change, and should not be trusted here without the
dynamic cross-check above.** Tallying `bl` targets in the real ARM disassembly
the same way as the passes above shows the per-iteration soft-float count
*rising* — 972 calls pre-change vs 1078 post-change, i.e. apparently +10.9%,
the opposite of what actually happens. This isn't a mistake in that count, it's
what the method structurally cannot see: the deleted work was inside an Eigen
loop kernel, counted once in the static tally no matter how many times its
internal loop actually executes, whereas the replacement is straight-line
scalar code that the tally counts in full. The BicubicField rewrite earlier in
this document flagged the same trap for static-vs-dynamic counting; this is
another instance of it, this time bad enough to flip the sign.

**Unity build** (`env:seeed_xiao_rp2040_unity`, `platformio.ini`): confirmed via
`pio run` that it compiles exactly one object,
`magnet_model_unity.cpp.o`, in place of the five separate ones — so
`build_src_filter` behaves as intended here and the PlatformIO LDF concern
raised when this environment was added does not bite in practice. Measured
effect on host dynamic instruction count: **−1.3% at `-O2`, ~0% at `-O3`**
(the two optimization levels converge to similar codegen once GCC has whole-TU
visibility either way it gets there). Kept as opt-in given the small win; worth
dropping instead if a second build environment isn't worth carrying for ~1%.

**`-O3`**: **−14.8%** executed instructions on top of the `-O2` post-change
number (host measurement), but real cost on target: **+6,576 B RAM, +14,688 B
flash** (RAM 61,952 → 68,528 of 262,144 total). Not switched by default —
RAM is the binding constraint here since the interpolation table must stay
resident, and 6.6 KB is meaningful headroom to give up pre-emptively. Worth a
one-line `PLATFORMIO_BUILD_FLAGS="-O3" pio run` experiment against the new
`micros()` timing on real hardware before deciding either way.

Wall-clock on the actual device is still unmeasured — that needs hardware, and
is what the new `micros()` instrumentation is for. Executed-instruction counts
on a pure-software-float target should track closely (nearly every float op is
a call there), but that is reasoning, not a measurement in hand.

### Found while doing this — since fixed elsewhere

`Positions::approx_rest_pos` was `{0, 0, magnet_z_pos_from_pivot}` = `{0,0,14}`,
which put the magnet-local query at exactly **(r=0, z=0)** — the magnet's
bottom face centre, outside the bicubic table's `z ∈ [-20,-0.5]` domain and at
the field's singular point. `magnet_rest_distance_sensor` was defined in
`positions.h` and referenced nowhere else, consistent with having been dropped
from this expression by accident.

This mattered beyond the first frame: it is the boot value of `last_pos`, the
reset value of the hot start, and the starting guess
`SensorController::updateCalibration()` perturbs by 0.1mm, so it fed the
calibration baseline too. It was left alone *in this pass* because correcting
it shifts that baseline and therefore the tuned `Config::GAIN_T`/`GAIN_R` — a
behavioural change, not a performance one.

**It has since been fixed**, in `c08329e` ("Fix constants and led
positioning") on a parallel branch that merged within a minute of this
section being written — cleanly, since the two touch different files, which
is why this note survived describing it as open. `positions.h` now reads
`approx_rest_pos = {0, 0, magnet_z_pos_from_pivot + magnet_rest_distance_sensor}`,
and `magnet_z_pos_from_pivot` is 15 rather than 14, so the rest guess is
`{0,0,21}` and the magnet-local query lands at (0, -6) — inside the table,
matching the 6mm standoff. Note the numbers above (14, and 20 as the intended
value) are the pre-fix ones and no longer describe the code.

## Fourth pass: does BLA inline better than Eigen, and can the manual unrolling come back out?

`TODO/eigen-to-bla-migration.md` replaced Eigen with a vendored, patched
`BasicLinearAlgebra` (patched to drop an `Arduino::Printable` base that was
adding a vtable pointer to every matrix instance). This pass asks the two
follow-on questions that migration left open: does BLA's own arithmetic
actually inline better than Eigen's did on this target, and if so, is any of
the hand-unrolled code above — the symmetric `H = JᵀJ`, the written-out skew
products, `BicubicField::evaluate`'s hoisted basis weights — now unnecessary?

**Method**: real ARM disassembly (`arm-none-eabi-objdump -d -C` +
`arm-none-eabi-gcc-nm -S`, same tools as the passes above, same compiler
invocation `pio run -v` actually uses — same flags, same
`arm-none-eabi-g++`, same target) on the current vendored-BLA build for
`nm`-confirmed inlining/call-count. **First pass at the two candidate
reverts below used host x86 `valgrind --tool=callgrind` counts instead —
wrong target, caught in review, redone.** This target has no hardware FPU
and no ARM emulator is available in this sandbox, so getting a true dynamic
count without real hardware means tracing the actual ARM-compiled loop
structure by hand: isolate the differing computation into a
`__attribute__((noinline))` function compiled with the project's exact
flags, identify each loop's real trip count from its address-stride/compare
codegen (all loops in both candidates have compile-time-fixed trip counts —
matrix dimensions, not data-dependent — so this gives an exact per-call
dynamic count, not an estimate), and multiply out. Slower than running a
host binary, but it's the number that's actually true on the RP2040, which
a host x86 count is not: x86 has a hardware FPU and a different
loop-unrolling cost model, so a host dynamic count reflects loop
structure/vectorization decisions, not the soft-float call count that
actually dominates cost on a target where every `float` op is a real
subroutine call.

### Does BLA inline better? Yes, substantially — with one nuance.

- **`BLA::CholeskyDecompose`, `BLA::CholeskySolve`, and the hand-written
  `dot()` helper (`math3D.h`) have zero standalone symbols anywhere in the
  linked binary** — confirmed with `nm`. They're fully inlined at every call
  site. This is the direct fix for the specific problem this document
  documented against Eigen: `.ldlt().solve()` used to stay as real
  out-of-line calls into `ldlt_inplace<1>::unblocked`/`assignCoeff`; under
  BLA there is no separate function there at all to call.
- **`BLA::operator*` for `Matrix<3,3,float> * Matrix<3,3,float>` inlines at
  most call sites but not all.** `VirtualSensor::evaluate` fully inlines
  both of its chained 3×3 products (`R * magnet.magnet_rotation`,
  `R_total * J_local * R_total_T`) — no separate `operator*` call anywhere
  in its disassembly. But `solve_pose.cpp`'s `orthonormalize_approx()` (two
  more 3×3 products) and `exp_so3(dw) * R` call the *same* `operator*`
  specialization out-of-line — confirmed via `nm`: exactly one
  `BLA::operator*<Matrix<3,3,float>,...>` symbol exists in the binary, and
  `solve_knob_pose`'s disassembly `bl`s into it twice. This is GCC's
  per-call-site inlining heuristic (cost budget already spent inlining
  other things in that function), not a BLA limitation — the same
  operation inlines cleanly elsewhere. **Per explicit direction this pass
  didn't chase where that leftover call lands** (RAM vs. flash placement is
  a separately-easy fix, not what this investigation was scoped to judge
  inlining by).
- `ForwardModel::evaluate` (1 real call, to `VirtualSensor::evaluate`, same
  structure this doc measured for Eigen post-optimization: "own call count
  collapsed from 14 `bl`s to 1"), `MagnetModel::evaluate` (1 real call, to
  `BicubicField::evaluate`, 202 instructions — comparable to the 175
  post-rewrite Eigen figure above, not the 700+-instruction bloat pattern
  poor inlining would produce), and `VirtualSensor::evaluate` (1 real call,
  to `MagnetModel::evaluate`) all show exactly the call structure this
  document's own tally expects — no unexpected out-of-line glue anywhere
  in that chain.

### Can any manual unrolling come back out? No — checked three candidates, all three cost real, measured work if reverted.

**`H = JᵀJ` (`solve_pose.cpp`), 21 symmetric dot products vs.
`~jacobian * jacobian`:** not revertible. Traced exact loop trip counts on
the real ARM-compiled object (`arm-none-eabi-g++`, real project flags):
the current form's `dot()` helper (`math3D.h`) compiles to a 9-iteration
loop of `fmul`+`fadd` pairs per call — 18 soft-float calls per dot product,
not the theoretical-minimum 17, because `dot()` initializes its accumulator
to `0.0f` and adds every term rather than special-casing the first one the
way `BLA::operator*` itself does internally (a real, if minor, one-line
inefficiency in `dot()`, noted separately below). 21 calls × 18 = **378
soft-float calls, real ARM dynamic count, exact** (all loop trip counts
here are compile-time-fixed matrix dimensions, not data-dependent, so this
is an exact per-call count, not an estimate). The full-product form's
`BLA::operator*` **does** special-case the first term (`ret(i,j) =
matA(i,0)*matB(0,j)` once, then 8 more `fmul`+`fadd` pairs) — 17 soft-float
calls per output entry, correctly optimal — but computes all 36 entries
where only 21 are needed: 36 × 17 + 1 `memcpy` (copying the 6×6 result) =
**613 total, real ARM dynamic count, exact**. **613 vs. 378 — the full
form costs 62% more.** (An earlier pass at this used a host x86
`valgrind --tool=callgrind` count instead, which came out *favoring* the
full-multiply form — 460M vs. 472M instructions over 200k calls. That
number was wrong for this target and has been replaced; see the method
note above.)

**Skew-matrix products (`virtual_sensor.cpp`), written-out entries vs.
`M * skew_matrix(v) - skew_matrix(B_field_global)`, specifically checked
because `skew_matrix()`'s zero entries are compile-time literals and this
project already builds with `-fno-signed-zeros`/`-fassociative-math`,
which make `x * 0.0f → 0.0f` a legal fold:** not revertible, and by a wider
margin than first measured. Same trip-count-tracing method, real ARM
object: the current hand-unrolled form is a 3-iteration loop of 9
`fmul`/`fsub` calls each (matches the source's 6 multiplies + 3 subtracts
per row) plus 6 more calls in a straight-line tail (the `−[B]ₓ` six-entry
touch) — **33 soft-float calls, 0 `memcpy`, exact.** The "clean" form's
`M * skew_matrix(v)` goes through `BLA::operator*`'s generic 3×3 loop,
which turned out **not to be unrolled** at `-O2` in this context — it's a
real nested loop (3 outer × 3 inner) computing a full, non-reduced
3-term dot product for *every* output entry, including the ones landing on
`skew_matrix()`'s zero literals. That's the actual reason the constant-fold
hypothesis fails: folding `x * 0.0f → 0.0f` requires the compiler to see
the literal at the specific multiply site, which only happens if the loop
is unrolled enough to separate that site out — it isn't, so the zero
never gets exploited at all. 9 entries × 5 ops (3 `fmul` + 2 `fadd`,
un-reduced) = 45, plus the elementwise subtraction against
`skew_matrix(B_field_global)` (9 more `fsub`, no equivalent zero-skip
either) = 54 soft-float calls, plus 3 `memcpy` calls (108 bytes total) for
materializing the temporaries and the final `Submatrix<3,3>` write — **57
total, exact.** **57 vs. 33 — the clean form costs 73% more**, a
substantially bigger gap than the host x86 number this document first
reported (which had actually gotten the *direction* right — clean costs
more — but by an order of magnitude less than the real target shows: the
host build's own loop-unrolling decisions happened to differ enough from
the ARM `-O2` build's that even the "which one has fewer soft-float calls"
comparison it made isn't the number to cite here). Kept as-is.

**`BicubicField::evaluate`'s hoisted-once basis weights vs. the original
per-row `cubic()`/`cubic_deriv()` calls:** not a candidate at all, on
re-reading this document's own second-pass section above — it already
concluded this was "an algebraic reassociation of a bilinear form... not a
redundancy-elimination or call-graph optimization, so no inlining directive
reaches it." That conclusion doesn't depend on which matrix library sits
underneath (the basis-weight formulas are plain scalar `float` arithmetic,
no `Mat3`/`Vec2` operations involved), and re-reading the current source
confirms nothing changed there. Nothing to test.

### Bottom line

BLA inlines its own glue code dramatically better than Eigen did — the
`ldlt_inplace`/`assignCoeff` problem this document spent two passes working
around is gone outright for Cholesky and dot products, not just mitigated.
But none of the three hand-unrolled optimizations in this document were
ever *purely* inlining workarounds — each has an independent algorithmic
justification (fewer redundant entries computed, fewer structural-zero
multiplies attempted, weights hoisted out of a loop) that holds regardless
of how well the underlying library inlines, and on the real target the
margins are larger than a first (host-x86-based) pass at this suggested:
62% more soft-float calls for the `H = JᵀJ` revert, 73% more for the skew
form. Measured on the actual RP2040 target, not assumed and not proxied
through a host build: all three stay **at this project's actual `-O2`
build**. That qualifier turns out to matter — see the `-O3` check below,
which changes the answer for one of the two.

### -O3: does more aggressive optimization change the answer? Yes for one, no for the other.

Re-ran both isolated ARM comparisons above at `-O3` and `-O3 -funroll-loops`
(same real project flags otherwise, same trip-count/unroll-tracing method
— checked branch *direction* this time, not just presence, after an
earlier miscount here read four short forward `bne.n` hops as loop
back-edges when they weren't).

- **`H = JᵀJ`: the gap closes completely.** At `-O3` both forms fully
  unroll (zero backward branches, confirmed by address comparison), and
  both land on **exactly 102 soft-float calls (54 `fmul` + 48 `fadd`)** —
  identical. `~jacobian * jacobian`'s output is symmetric (`H(i,j)` and
  `H(j,i)` are literally the same expression over the same inputs), and at
  `-O3` GCC's value-numbering apparently recognizes that once the whole
  36-entry computation is unrolled into straight-line code, and stops
  redoing the duplicate half — the same saving the hand-written 21-dot
  version gets by construction. **This is not the zero-multiply constant
  fold the skew-matrix hypothesis was about** — `jacobian` has no zero
  entries — it's cross-entry redundancy elimination on a symmetric result,
  a different mechanism, and it only fires once the loop is fully unrolled
  (still real at `-O2`, where the loop doesn't unroll and the gap is the
  62% measured above).
- **Skew-matrix products: `-O3` alone narrows the gap but doesn't close
  it — `-O3 -ffinite-math-only` together close it exactly.** See the
  dedicated section right below; this was worth chasing further rather
  than stopping at "doesn't close."

**So: `-O3` would make dropping the `H = JᵀJ` hand-optimization free, but
not the skew-matrix one on its own.** Caveat that matters before acting on
either: this was measured on isolated single-function objects, not a real
build, and `-O3` project-wide has a documented cost from this same
document's earlier passes — **+14,688 B flash, +6,576 B RAM** — with RAM
called out there as the binding constraint (the bicubic table has to stay
resident). Getting either win for free requires *building at `-O3`*, which
is not currently how this project builds and has its own real tradeoff
already evaluated and left off by default. A narrower option: `-O3` scoped
to just `solve_pose.cpp`/`virtual_sensor.cpp` via
`__attribute__((optimize(...)))` or `#pragma GCC optimize`. For the
skew-matrix case this has now been tested — see the section below — and it
does **not** reach parity, capping at 51 calls instead of 33, because
per-function/per-region scoping doesn't reliably combine `-O3` with
`-ffinite-math-only` the way real command-line flags do. Still untested for
`H = JᵀJ` specifically, which only needs `-O3` itself (no `-f` flag to
combine it with), so it may not hit the same limitation — and would need
the same numerical-equivalence verification (`test_jacobian.cpp`, the
solver convergence check) any of this document's other changes get before
landing.

### Pushed further on the skew-matrix gap specifically: it closes exactly, with a real catch

Asked directly to try harder on this one rather than accept "narrows but
doesn't close." Swept the individual components of `-ffast-math` on top of
`-O3` (full `-ffast-math` alone already produced an exact match — worth
isolating *which piece* of it did that, since blanket `-ffast-math` is
usually too broad a hammer to reach for on a physical-position solver):

| flags (on top of `-O3` + this project's existing relaxed-math flags) | `clean_skew` result |
|---|---|
| (none — `-O3` alone) | 51 calls, +55% vs. current's 33 |
| `-ffinite-math-only` | **33 calls — exact match** |
| `-funsafe-math-optimizations` | 51 calls, no change |
| `-fexcess-precision=fast` | 51 calls, no change |
| full `-ffast-math` | 33 calls — exact match (subsumes the above) |

**`-ffinite-math-only` is the specific, isolated piece that does it** — and
it makes sense why: `x * 0.0f → 0.0f` is only a valid fold if `x` can't be
NaN or Inf (`NaN * 0 = NaN`, `Inf * 0 = NaN`, not `0`). This project's
existing relaxed-math flags (`-fno-signed-zeros`, `-fassociative-math`,
etc.) waive IEEE *signed-zero* and *reassociation* guarantees, but none of
them waive the NaN/Inf guarantee `-ffinite-math-only` waives — that was the
actual missing piece the whole time, not a matter of trying harder on
inlining. Confirmed it's specifically the `-O3`+`-ffinite-math-only`
*combination* that's needed: `-ffinite-math-only` alone at this project's
actual `-O2` gets partway (39 calls, +18%, the `memcpy`s survive and the
subtraction against `skew_matrix(B)` stays unreduced) but not exactly —
full parity needs both together.

**The catch, and it's a real one, not a formality**: `-ffinite-math-only`
doesn't just delete dead checks — it changes what gets *computed* when a
NaN or Inf actually occurs, because folds like this one are only
equivalence-preserving under the finite assumption. `solve_pose.cpp` has
an explicit safety net built around exactly the case this flag assumes
away:

```cpp
if (!all_finite(dx)) {
    // Math collapsed (NaN or Inf). Reject update and abort solver.
    ...
}
```

If `-ffinite-math-only` were applied anywhere upstream of that check —
even scoped to just `VirtualSensor::evaluate`, not the whole project — a
genuine NaN arising from a numerically degenerate pose could get folded
into something that *looks* finite by the time it reaches `all_finite()`,
instead of propagating as the NaN that check exists to catch. That's not
hypothetical: it's precisely the class of transformation this flag
licenses. Scoping the flag narrowly (a `#pragma GCC optimize` /
`__attribute__((optimize(...)))` bracket around just the skew computation,
not the whole solve loop) would reduce the blast radius but doesn't remove
this risk in principle for values that flow from there into `dx`.

**Superseded**: at the time this was written, not this document's call to
make unilaterally — flagged with the numbers rather than applied. The user
subsequently decided to accept this tradeoff and go project-wide; see "All
hand-unrolling removed" below for what was actually applied, including the
bit-level check that mitigates (but does not eliminate) the safety-net
risk described here.

### Scoping the flag instead of applying it everywhere: mechanically safe, but doesn't reach parity

Asked directly whether the fold above can be had without the flag
"everywhere" — scoped to just the function that needs it, leaving
`solve_pose.cpp`'s `all_finite(dx)` check (and everything else) at the
project's normal flags. Two separate questions:

1. **Does scoping avoid the safety-net risk?** Yes, for a boring reason:
   `virtual_sensor.cpp` and `solve_pose.cpp` are separate translation units
   and this project doesn't build with LTO, so nothing in
   `virtual_sensor.cpp` — flagged or not — can change what GCC assumes when
   it compiles `solve_pose.cpp`'s `all_finite()` check. `virtual_sensor.cpp`
   also has no NaN/Inf-sensitive logic of its own to worry about (checked
   directly — no `isnan`/`isinf`/`all_finite` in that file). The only
   residual risk is local to values computed *inside* the flagged function
   itself, exactly as the previous section already noted.

2. **Does scoping actually reach the 33-call parity found above?** Tested
   both mechanisms GCC offers — `__attribute__((optimize(...)))` on the
   function, and `#pragma GCC push_options` / `optimize(...)` /
   `pop_options` around it — against an isolated copy of the clean
   `skew_matrix()`-based rewrite, compiled with the project's real `-O2`
   base flags otherwise unchanged:

   | scoping mechanism | flags requested | result |
   |---|---|---|
   | *(whole-TU, for reference)* | `-O2` | 57 calls |
   | `__attribute__((optimize("finite-math-only")))` | finite-math only | **54 calls — exact parity with the whole-TU `-O2 -ffinite-math-only` measurement above** |
   | *(whole-TU, for reference)* | `-O3` alone | 51 calls |
   | `__attribute__((optimize("O3,finite-math-only")))` | O3 + finite-math | 51 calls — **matches `-O3` alone; the finite-math fold contributes nothing extra** |
   | `__attribute__((optimize("finite-math-only,O3")))` (order reversed) | same | 51 calls — same result |
   | `#pragma GCC optimize("O3")` then a second `optimize("finite-math-only")` (stacked) | same | 51 calls — same |
   | `#pragma GCC optimize("O3,finite-math-only")` (one line) | same | 51 calls — same |
   | two stacked `__attribute__((optimize(...)))` on one function | O3 then finite-math | anomalous — collapsed to a 6-call *looped* result, evidence the optimization level itself regressed rather than combined; not a usable path |
   | *(whole-TU, for reference)* | `-O3 -ffinite-math-only` | 33 calls — full parity |

   **Scoping an `-O` level change together with `-ffinite-math-only`,
   through every mechanism GCC offers, consistently reproduces `-O3`
   alone's number (51) — never the whole-TU combination's 33.** Scoping
   *just* `-ffinite-math-only` at the project's real `-O2` works exactly as
   well as the whole-TU flag would (54, matching parity) — the mechanism
   itself isn't broken for a single flag. It's specifically the
   *combination* of an optimization-level bump with an individual `-f` flag
   that doesn't survive the per-function `optimize` attribute/pragma path,
   even though the identical combination works as real command-line flags.
   This reads as a genuine GCC limitation rather than anything about this
   codebase — the `optimize` attribute has documented gaps versus the same
   flags on the command line, and this is a concrete instance of one.

**Net effect: scoping doesn't reach the same place as the flag "everywhere."**
The only measured way to the full 33-call parity is real
`-O3 -ffinite-math-only` on the whole translation unit (or the whole
project) — not a per-function override. A per-*file* flag override (a
custom PlatformIO/SCons rule limited to `virtual_sensor.cpp`) would very
likely reach 33, since that's the same whole-TU mechanism the 33-call
measurement used, and it would leave `solve_pose.cpp` untouched — but
that's a real build-system change, not a one-line attribute, for a result
that only *matches* the hand-unrolled code's performance rather than
beating it. The mechanism that's actually cheap to apply (a function
attribute) tops out at 51 calls — 55% more than today's 33 — so it's a
real loss, not a wash. Given that, **the hand-unrolled skew-matrix code
stays.** Not applied; recorded here so the next person doesn't re-try
attribute scoping expecting it to reach parity.

### Went to `-O3` project-wide; `H = JᵀJ` reverted for free, the skew fix via pragma did not extrapolate

Decided to actually pay `-O3`'s project-wide cost (checked against the real
XIAO RP2040 budget: 2 MB flash / 264 KB RAM, comfortably affordable) and
contain `-ffinite-math-only`'s NaN-safety-net risk by scoping it to just
`virtual_sensor.cpp` via `#pragma GCC push_options` /
`optimize("finite-math-only")` / `pop_options` — the idea being that with
`-O3` now a real project-wide command-line flag, only the single
`finite-math-only` flag needs scoping, which earlier testing (the section
above) showed reaches exact parity for a *single* flag scoped alone.

`platformio.ini`'s `-O3` change and `solve_pose.cpp`'s `H = JᵀJ` revert
(`Matrix6x6f H = ~jacobian * jacobian;`) are applied and verified: full
build succeeds, **Flash 327,040 B (16.1%) / RAM 70,900 B (27.0%)** — up from
the pre-`-O3` baseline of 298,976 B / 63,348 B, and well inside budget.
`H = JᵀJ` doesn't depend on `finite-math-only` at all, only on `-O3` itself
being real (not scoped), so it isn't affected by what follows.

**The skew-matrix pragma did not extrapolate to the real build.** Compiling
the actual `virtual_sensor.cpp` object from the real build (`-O3`
project-wide + the file-scoped pragma) and disassembling it directly:
the skew computation lands on **51 soft-float calls, not the target 33** —
the same number as `-O3` alone, no finite-math benefit at all. This is a
genuinely new combination the prior section didn't test: it only measured
"ambient `-O2` + scoped `finite-math-only` alone" (54, parity) and "ambient
`-O2`-ish + scoped `O3,finite-math-only` combined in one string" (51, no
benefit) — never "ambient *real* command-line `-O3` + scoped
`finite-math-only` alone on top of it," which is exactly this case, and it
also caps at 51. Whatever GCC does differently between a `-O` level set via
the actual command line versus reconstructed through `optimize`
attribute/pragma machinery, it evidently affects this fold even when the
pragma itself only ever asks for the one flag.

**Reverted just the skew part back to the hand-unrolled form** rather than
ship a real 55% regression (51 vs. 33 calls) under the mistaken belief that
parity was reached — the file no longer carries the pragma. `-O3`
project-wide and the `H = JᵀJ` revert stand on their own regardless of how
the skew question resolves.

**Superseded by the next section**: the "still open" per-file build-flag
override this section originally proposed as the untried next step was not
needed — the project went `-ffinite-math-only` project-wide instead (see
below), which is the same real-command-line-flags mechanism and reaches the
same 33-call parity without a custom SCons rule.

### All hand-unrolling removed: `-ffinite-math-only` project-wide, with a bit-level finiteness check to protect the safety net

Decided to go all the way: `-ffinite-math-only` project-wide (a real
command-line flag — the only mechanism actually measured to reach 33-call
parity, since every scoping attempt above topped out at 51), and fix the
resulting NaN/Inf safety-net risk at its root with a check immune to the
flag, rather than trying to contain the flag's scope.

**Why a bit-level check closes the check-folding risk (but not the
upstream-fold risk).** `-ffinite-math-only` licenses two distinct things:
(a) folds like `x * 0.0f -> 0.0f` even when `x` is actually NaN/Inf at
runtime — the mechanism this whole investigation has been exploiting — and
(b) treating `isnan`/`isinf`/`std::isfinite`-style checks as always coming
out "finite," since the compiler assumes the other branch is unreachable,
which can fold the check itself into a constant `true`. A check that reads
the raw IEEE-754 bit pattern via `memcpy` and tests the exponent field with
plain integer ops isn't a floating-point *operation* in the sense this flag
governs, so it can't be folded the same way — `math3D.h` now has:

```cpp
inline bool is_finite_bits(float x) {
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}
```

(a float is NaN or Inf iff its exponent bits are all one), and `all_finite()`
calls it instead of `std::isfinite()`. This closes risk (b) completely. It
does **not** close risk (a) for values computed *upstream* of the check —
if a NaN feeding into `M * skew_matrix(v)` lined up with one of
`skew_matrix()`'s literal-zero entries, the fold could still discard that
NaN before it ever reaches a downstream check. That residual risk is
inherent to the flag, not something any check design can close; recorded
honestly rather than claimed away.

**Two sites needed the fix, a third didn't.** Grepped the whole tree for
`isnan`/`isinf`/`isfinite`/`all_finite`:
- `math3D.h`'s `all_finite()` (`solve_pose.cpp`'s `if (!all_finite(dx))`,
  the solver's core safety net) — fixed, as above.
- `CalibrationStorage.cpp`'s `isPlausible()` (rejects corrupted/erased
  flash — every byte reads `0xFF`, every float decodes to NaN) — also
  fixed, same `is_finite_bits()`. Its old comment plainly stated this build
  didn't set `-ffinite-math-only`; that's now false, so the comment was
  rewritten. (It also used to note "this used to be a bit test during
  unpacking" — this change is a return to a pattern the file already used
  once, not a new one.)
- BLA's `CholeskyDecompose` (`firmware/lib/BasicLinearAlgebra/impl/NotSoBasicLinearAlgebra.h`,
  the `if (sum <= 0.0)` positive-definite check) — **left alone.** Read the
  implementation: `sum -= A(i,k) * A(j,k)` multiplies two runtime matrix
  entries, never a compile-time literal zero, so there's no fold
  opportunity here regardless of the flag. `NaN <= 0.0` already evaluates
  `false` under plain IEEE rules — this was never the real NaN-catcher —
  and `sqrt()`/division are genuine runtime library calls that correctly
  propagate an actual runtime NaN either way, since the flag changes what
  the *compiler* may assume and fold, not what these runtime operations
  compute. Real NaN detection already happens downstream at `all_finite(dx)`,
  the site that was actually fixed.

**Verified on the real build, not assumed:**
- The skew computation in `virtual_sensor.cpp` (now the clean
  `J.Submatrix<3, 3>(0, 3) = M * skew_matrix(v) - skew_matrix(B_field_global);`,
  pragma removed) now costs **exactly 33 soft-float calls** — disassembled
  the real compiled object, located the tail after the `MagnetModel::evaluate`
  call, subtracted the (separately verified) 15 calls for `B_field_global`
  and 90 for `M`'s two chained 3×3 products from the 138-call remainder.
  Full parity with the hand-unrolled form, reached this time because
  `-O3`/`-ffinite-math-only` are both real project-wide flags rather than a
  pragma reconstruction.
- `is_finite_bits()` was **not** folded away: disassembled both
  `solve_pose.cpp.o` (`all_finite(dx)`) and `CalibrationStorage.cpp.o`
  (`isPlausible()`) and found the real mask-and-compare sequence intact in
  both — `movs r3, #255; lsls r3, r3, #23` (building `0x7F800000`),
  `ands`/`cmp` against it, branching to the NaN-error path
  (`solve_pose.cpp`) or `return false` (`CalibrationStorage.cpp`). Confirmed
  directly, not inferred from the source alone.
- `H = JᵀJ` remains free at `-O3` (unaffected by any of this, doesn't need
  `finite-math-only`).
- Full build: **Flash 326,712 B (16.1%) / RAM 70,644 B (26.9%)** — in the
  same range as the earlier `-O3`-only checkpoint, comfortably inside the
  2 MB / 264 KB budget.
- `pio run -e seeed_xiao_rp2040_unity` still builds. The pragma-leak concern
  the previous attempt's unity check existed for no longer applies — the
  flag is a real project-wide command-line flag now, not a per-file pragma,
  so there's nothing for `magnet_model_unity.cpp`'s textual concatenation to
  leak across.
- `firmware/test/test_jacobian.cpp`'s finite-difference suite: `pio test
  -e seeed_xiao_rp2040_test` builds clean (upload fails in this sandbox —
  no hardware — as expected); ran the equivalent host-native harness this
  session already built and it's 5/5 passing, including the full 384-point
  `ForwardModel` grid sweep. This is also the correctness check for the
  `BicubicField.cpp` change below, per this doc's own evaluate()-touching
  rule.
- Host-native solver checks: the realistic hot-start perturbation
  (~0.03mm/~0.3°) still converges to the same numbers
  `TODO/eigen-to-bla-migration.md` already documented (`t_err≈2×10⁻⁵mm`,
  `R_err≈9×10⁻⁶`). The known large-perturbation divergence
  (`TODO/eigen-to-bla-migration.md`'s caveat, ~0.7mm/~3.5°) still reproduces
  — checked directly against a true baseline build of the pre-this-session
  code via `git stash`, which diverges on the identical case too (different
  specific numbers, same failure), confirming this is the pre-existing,
  already-documented issue, not a new regression.

### `BicubicField.cpp`'s hand-rolled Catmull-Rom weights: also removed, but this one has a real, measured cost

Corrected an earlier scoping mistake in this document: the second pass's
"hoisted-once basis weights" conclusion is about **when** the weights are
computed (once per `evaluate()` call vs. once per row via
`cubic()`/`cubic_deriv()`), and that hoisting is real and unaffected by any
of this. It says nothing about **how** they're computed. The actual code
wrote `a0..a3`/`b0..b3` and their derivatives as 16 individually-expanded
scalar polynomial lines, and the comment directly above them gave the real
reason: `BLA::Matrix` has no `constexpr` constructor, so the clean matrix
form the function's own header comment already documents (a fixed 4×4
Catmull-Rom basis matrix times a power vector `[1,t,t²,t³]`) couldn't be a
compile-time constant, and was hand-expanded into scalar arithmetic
instead. That's a separate axis from the hoisting question, and it *is*
more hand-unrolling.

Verified the matrix form algebraically against all 16 original scalar
lines before touching the code (not just spot-checked): `a_i(t) =
0.5 * Σⱼ BasisMatrix(i,j) * t^j`, e.g. `a0 = -0.5t + t² - 0.5t³` is exactly
row 0 of `0.5 * BasisMatrix * [1,t,t²,t³]ᵀ`, and `da0_dt = -0.5 + 2t -
1.5t²` is the same row against `[0,1,2t,3t²]ᵀ`. Replaced with a file-local
`Mat4`/`Vec4` and four matrix-vector products
(`a4 = 0.5f*(basis*Vec4(1,t,t2,t3))`, etc.), the overshoot step collapsed
from 8 scalar `+=` lines to 2 vector ones, and the contraction loops index
`a4(0)`/`a4(1)`/etc. in place of the old named scalars.

**Fully inlines** — `nm` on the compiled object shows one
`BicubicField::evaluate` symbol either way, old and new, no separate
`BLA::operator*` call left out-of-line.

**But it is not free, unlike the skew-matrix case.** Compiled both the old
scalar version and the new matrix version with the *identical* real project
compile line (`-O3 -ffinite-math-only`, captured via `pio run -v`) and
disassembled both:

| version | soft-float calls |
|---|---|
| old (hand-unrolled scalar) | 320 (171 `fmul` + 124 `fadd` + 25 `fsub`) |
| new (matrix form) | 344 (191 `fmul` + 131 `fadd` + 22 `fsub`) |

**24 more calls, ~7.5% more**, even with the same flags that gave the
skew-matrix case exact parity. The basis matrix's structural zeros (5 of 16
entries) and the derivative power vector's leading zero give GCC *some*
fold opportunity, but not enough to fully offset computing all 16 entries
of two naive `4×4 · 4×1` products per call (`a`, `da/dt`, `b`, `db/du` — 4
matrix-vector products total) versus the hand-written form, which never
computed the zero terms in the first place. This is a real, measured
tradeoff, not a wash — recorded plainly rather than assumed free by
analogy to the skew case. Kept anyway, per direct instruction to remove all
remaining hand-unrolling; the small extra cost buys a form that matches the
function's own documented derivation instead of a second, silently
-hand-optimized copy of it.

**Small thing found along the way, not acted on**: `dot()` (`math3D.h`)
costs 18 soft-float calls per length-9 call rather than the achievable 17
— it initializes its accumulator to `0.0f` and adds every term, instead of
special-casing the first term the way `BLA::operator*` does internally.
Real but marginal (1 extra `fadd` per call, 21 calls in the hot `H = JᵀJ`
loop = 21 extra calls per LM iteration); worth a one-line fix sometime, not
urgent enough to bundle into this pass.

## Open next steps (not yet acted on)

- Add iteration-count + per-phase (`micros()`) instrumentation to get real
  convergence and timing data instead of static worst-case counts — no longer
  blocked on the Statistics TODO, which is resolved (see above).
- ~~Investigate whether Eigen's small fixed-size helpers can be coaxed into
  inlining even further~~ — done for `BicubicField::evaluate`, see above:
  no, inlining directives alone don't reach it, manual algebraic
  reassociation was required. ~~Worth checking whether the same applies to
  `MagnetModel`/`VirtualSensor`/`ForwardModel`'s Eigen usage~~ — moot, Eigen
  is gone (`TODO/eigen-to-bla-migration.md`); see the fourth pass above for
  the equivalent question against BLA. `CholeskyDecompose`/`CholeskySolve`/
  `dot()` now inline completely (zero standalone symbols); one leftover
  `BLA::operator*` call in `solve_pose.cpp` doesn't inline at two call
  sites (it does elsewhere) — still worth a closer look at whether that's
  RAM- or flash-resident, per the fourth pass's own note that this pass
  deliberately didn't chase it.
- Quasi-Newton Jacobian reuse (parked above) as a way to cut the iteration
  count's multiplier on the expensive Jacobian-assembly path specifically.
- Re-run the operation tally above against the current binary once iteration
  telemetry lands, to get real (not worst-case) numbers now that the gain
  multiplies are gone, `BicubicField::evaluate` has been rewritten, and
  inlining looks healthier.
- `claude/bicubic-field-rp2040-optimize-tmcqof` is merged into `experimental`
  and its code is what the third pass's own baseline was measured against
  (instruction-count/flash/RAM, not wall-clock). What's still missing is a
  real on-device wall-clock/Hz number for the combined
  `BicubicField::evaluate` rewrite + third-pass changes together, the way the
  first optimization pass measured 134Hz/80Hz — the `micros()` instrumentation
  from the third pass is what that measurement would read.
