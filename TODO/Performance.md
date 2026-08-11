# solve_pose performance

The whole loop runs at 50Hz, and `solve_knob_pose()` (`firmware/src/magnet_model/solve_pose.cpp`)
currently eats a little under half that budget — call it 10ms. Goal is to get
it down toward ~4ms if we can. This file is the investigation notes: assembly-
verified hypotheses on where that time actually goes, not yet a fix plan.

Platform context that matters for everything below: Seeed Xiao RP2040, dual
Cortex-M0+ @ 133MHz, **no hardware FPU**. Every `float` operation is a software
subroutine call. `solve_knob_pose`, `ForwardModel::evaluate`,
`Sensor::evaluate`, `MagnetModel::evaluate`, and `BicubicField::evaluate` are
all marked `__not_in_flash_func` (placed in RAM), presumably to dodge flash
XIP wait-states on this hot path.

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
| **`Sensor::evaluate`** (+ 1× MagnetModel) | 353 | 223 | 89 | 1 | 1 sqrt | 3 | 6 | 29 | **705** |
| **`ForwardModel::evaluate`** (+ 1× Sensor; sensor-loop body counted once, real loop runs 3×) | 353 | 223 | 89 | 1 | 1 sqrt | 3 | 6 | 29 | **705** |
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
- `ForwardModel::evaluate` shows identical totals to `Sensor::evaluate`
  because `ForwardModel::evaluate` itself does no arithmetic (pure block
  assembly) — all its cost is the nested `Sensor::evaluate`, counted once
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
- Sensor gain moved out of the solver's hot path: `Sensor` no longer carries
  a `sensor_gain` member, and `Sensor::evaluate` (`sensor.cpp`) dropped the
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
from 14 `bl`s to 1 (down to just the `Sensor::evaluate` call itself), and
`MagnetModel::evaluate` shrank (206→175 instructions) with fewer calls,
consistent with the `assert_func` removal. `Sensor::evaluate` and
`solve_knob_pose` actually grew in instruction count and in-body call count —
not a contradiction: that's what it looks like when a helper that used to be
a separate out-of-line function (with its own prologue/epilogue and, per the
finding above, its own RAM→flash veneer hop) gets inlined directly into the
caller instead — the caller's body gets bigger and shows more direct
`__wrap_fmul`/`fadd` calls, but there's one fewer function-call hop (and one
fewer veneer) in the chain per operation. Because `sensor.cpp` and
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
above once this lands, and updating the `MagnetModel`/`Sensor`/`ForwardModel`/
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
- **`R_mag` folded into `R_total`** (`sensor.cpp`). `R·(R_mag·J·R_magᵀ)·Rᵀ`
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
| `Sensor::evaluate` | 17,525,448 | 12,260,127 | −5.3M |
| `solve_knob_pose` | 4,877,296 | 9,251,607 | +4.4M |

The symmetric-`JᵀJ` change was the single biggest win, bigger than estimated:
`jacobian.transpose() * jacobian` was dispatching into Eigen's general blocked
matrix-multiply kernel (`gebp_kernel`/`gemm_pack_lhs`, meant for large matrices,
not a 9×6), and replacing it with 21 explicit dot products deletes that whole
code path — hence 16.8M instructions to zero, not a partial reduction. The rise
in `solve_knob_pose`'s own count is that deleted work moving *into* the function
as inlined scalar code rather than a remote call, which is why it shows up
there instead of vanishing outright. `sensor.cpp`'s own algebra changes landed
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

### Found while doing this, not acted on

`Positions::approx_rest_pos` is `{0, 0, magnet_z_pos_from_pivot}` = `{0,0,14}`,
which puts the magnet-local query at exactly **(r=0, z=0)** — the magnet's
bottom face centre, outside the bicubic table's `z ∈ [-20,-0.5]` domain and at
the field's singular point. `magnet_z_pos_from_pivot + magnet_rest_distance_sensor`
= 20 lands at (0, -6), comfortably inside, and matches the 6mm standoff the
geometry comment describes. `magnet_rest_distance_sensor` is defined in
`positions.h` and referenced nowhere else in the repo, which is consistent with
it having been dropped from this expression by accident.

This is the boot value of `last_pos`, the reset value of the hot start, and the
starting guess `SensorController::updateCalibration()` perturbs by 0.1mm — so
it affects the calibration baseline, not just the first frame. Left alone here
because correcting it shifts the calibration baseline and therefore the tuned
`Config::GAIN_T`/`GAIN_R`, which is a behavioural change rather than a
performance one. Probably belongs with `TODO/tare-and-calibration.md`.

## Open next steps (not yet acted on)

- Add iteration-count + per-phase (`micros()`) instrumentation to get real
  convergence and timing data instead of static worst-case counts — no longer
  blocked on the Statistics TODO, which is resolved (see above).
- ~~Investigate whether Eigen's small fixed-size helpers can be coaxed into
  inlining even further~~ — done for `BicubicField::evaluate`, see above:
  no, inlining directives alone don't reach it, manual algebraic
  reassociation was required. Worth checking whether the same applies to
  `MagnetModel`/`Sensor`/`ForwardModel`'s Eigen usage, or whether it's worth
  dropping `__not_in_flash_func` from some of these functions given how much
  of their work already ends up in flash-resident Eigen internals anyway.
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
