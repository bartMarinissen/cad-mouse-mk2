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

## Blocking prerequisite: the Statistics TODO

Before adding any new solver telemetry (iteration count, per-phase timing),
we need to resolve the existing TODO at `solve_pose.cpp:21`: `solve_knob_pose`
always allocates local `residual`/`jacobian` and copies them into
`*residual_out`/`*Jacobian_out` at the end, rather than writing into those
output pointers directly when they're non-null. `MotionController::read_pose`
always passes them (`Config::statistics` is `true`), so this copy is live on
every call today. Fix this first so new instrumentation doesn't get bolted
onto a code path that's about to be restructured.

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

## Open next steps (not yet acted on)

- Fix the Statistics TODO, then add iteration-count + per-phase (`micros()`)
  instrumentation to get real convergence and timing data instead of static
  worst-case counts.
- Investigate whether Eigen's small fixed-size helpers can be coaxed into
  inlining even further, or whether it's worth dropping `__not_in_flash_func`
  from some of these functions given how much of their work already ends up
  in flash-resident Eigen internals anyway.
- Quasi-Newton Jacobian reuse (parked above) as a way to cut the iteration
  count's multiplier on the expensive Jacobian-assembly path specifically.
- Re-run the operation tally above against the current binary once the
  Statistics TODO + iteration telemetry land, to get real (not worst-case)
  numbers now that the gain multiplies are gone and inlining looks healthier.
