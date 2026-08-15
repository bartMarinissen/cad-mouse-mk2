# Schur-complement solver: the shape, not the solver

Scope note up front: this is about the *data structure* the bundle-calibration
solver would work on — what has to exist in memory, when, and how big it is —
not the LM/trust-region outer loop (damping, step acceptance, convergence),
which is separate, later work.

**The derivation lives in `design documentation/Math.md` §7** — why the
normal equations are arrowhead-shaped, why the elimination is an exact
identity rather than an approximation, and why it streams one frame at a
time. That's the one authoritative copy; this file is what's specific to
turning it into working code: the store-vs-recompute memory tradeoff, the
concrete data structures, and what precision the implementation actually
hits.

Names here follow the code (`H_pp`, `H_sp`, `H_ss`, `rhs_p`, `rhs_s`), which
map onto Math.md §7's $D_k$, $B_k$, $A$, $\mathbf b_k$, $\mathbf a$
respectively — one frame's terms, not summed, except `H_ss`/`rhs_s` which
are each frame's *additive contribution* to the shared corner ($A$,
$\mathbf a$ are sums over all frames).

Verified against a dense reference (assemble the *full* arrowhead system,
solve it directly, check the frame-by-frame path in
`schur_normal_equations.h` agrees) rather than finite differences — Math.md
§7.4 is a linear-algebra identity, so there's a ground-truth answer to check
against, not just "plausible". Measured (host build, real Eigen 3.4,
`mt19937`-seeded random synthetic frames): both `dx_shared` and every
frame's `dx_pose` match the dense reference to ~7-9e-7 relative error —
float32 machine precision, not an approximation. (The first version of this
test used a hand-rolled deterministic generator, `sin()` of a linear index
combination, instead of a real PRNG — every matrix it produced was silently
rank-deficient, since `sin(x + shift)` is always a linear combination of
`sin(x)`/`cos(x)` and so spans only a 2-D subspace regardless of matrix
size. That made the test fail at ~100% relative error on a correct
implementation; fixed by switching to `std::mt19937`, and the test now
explicitly checks each synthetic frame's `H_pp` is actually full rank
before trusting the comparison, so this failure mode can't silently
reappear.)

Note what's *not* new here: `H_pp` is a 6×6, and `H_pp.ldlt().solve(...)` is
the same fixed-size LDLT call `solve_pose.cpp` already does, at the same
size, once per frame. The only new primitive is a P×P dense LDLT (bigger,
but still fixed-size, no-malloc, one call per solver iteration, not per
frame).

## The actual question: store per-frame data across passes, or recompute it?

The reduction above needs each frame's local system exactly once to fold
into `H_reduced`/`rhs_reduced` (call this **pass 1**), and then needs it
*again* to back-substitute that frame's pose update once `dx_shared` is known
(**pass 2**, which can only happen after pass 1 has finished for every frame
and the P×P system has been solved). Between those two passes, something has
to give: either keep every frame's `H_sp`/`H_pp`/`rhs_p` in memory, or
throw them away and rebuild them.

**Storing per frame:** `H_sp` alone is P×6 = 45×6 = 270 floats = 1080 bytes;
× 60 frames = **~65 KB**. Measured against what's actually left: a current
`pio run -e seeed_xiao_rp2040` reports 64,516 bytes of static RAM used of
262,144, so ~193 KB remains for stack and heap combined. Storing every frame's
coupling block would claim a third of that, for data needed only for a few
microseconds during pass 2's back-substitution.

**Recomputing in pass 2:** rebuild each frame's local system from scratch —
call `evaluate_bundle_jacobian` for that frame's 3 sensors again, using the
frame's current pose estimate (which persists across LM iterations regardless
— that's ~60×6 = 360 floats = 1.4 KB, trivial, and not new: *some* per-frame
pose state has to persist across iterations no matter which design is
chosen). Cost: the forward-model evaluation — the actual expensive part —
runs twice per iteration instead of once.

Pass 2's rebuild is, however, materially cheaper than pass 1's, and the data
structures are shaped to make that automatic rather than optional.
Back-substitution reads only `H_pp`, `H_sp`, `rhs_p` — never `H_ss`/`rhs_s`
— and `H_ss` is the single most expensive term in the accumulation
(`3P²` MACs per sensor, ≈18k per frame at P=45, against ≈2.4k for `H_sp`).
So pass 2 rebuilds a `FramePoseBlock<P>`, which has no `H_ss` to accumulate
and no P×P stack temporary to hold (~1.2 KB against ~8.4 KB). The
linear-algebra half of a pass-2 frame is roughly an order of magnitude
cheaper than a pass-1 frame; the forward-model half is identical, so the
end-to-end ratio depends on which dominates — not yet measured on target.

**One ordering constraint the rebuild imposes:** pass 2 must linearize at the
*same point* as pass 1 — same `x_shared`, same `pose_f`. So `dx_shared` and
the `dx_pose` may only be applied to the iterate after pass 2 has finished
for every frame. Update early and each frame's `H_sp`/`rhs_p` come from a
different linearization than the reduced system that produced `dx_shared`,
and the back-substitution silently stops being the exact identity it is
derived as. This is a property of recompute specifically: the store design
cannot get it wrong, because stored blocks carry their linearization point
with them.

**Recommendation: recompute.** This device is RAM-constrained (a fixed,
unforgiving 256 KB) and comparatively compute-rich for this specific
workload: calibration is a rare, one-time-per-unit, patient operation, not
the 20 Hz motion-tracking loop `TODO/Performance.md` is fighting for
milliseconds on. Doubling the Jacobian-evaluation cost of a calibration that
already isn't latency-sensitive is a good trade for keeping peak memory at
O(P²) + O(N) instead of O(P·N). This also sidesteps ever allocating (or
statically declaring) an array of N per-frame structs at all — "run frame by
frame" in the literal sense: one frame's data exists on the stack, gets
folded into the P×P accumulator or back-substituted, and is gone.

## The data structures (implemented, verified)

- **`FramePoseBlock<P>`** — exactly one frame's *row* of the arrowhead:
  `H_pp` (6×6), `H_sp` (P×6), `rhs_p` (6). Everything that touches a frame's
  pose — eliminating it in pass 1, recovering it in pass 2 — needs these
  three and nothing else. Built via 3 calls to `add_sensor()` (one per
  sensor, taking that sensor's own already-weighted 3-row residual/Jacobian
  contribution — matching `evaluate_bundle_jacobian`'s per-sensor output
  shape). This is what pass 2 rebuilds.
- **`FrameNormalEquations<P>`** — a `FramePoseBlock<P>` plus the frame's
  additive contribution to the *shared* corner: `H_ss` (P×P), `rhs_s` (P).
  Its `add_sensor()` forwards to the inner one rather than restating the
  three pose-row formulas, so no formula is written twice. Worth noting that
  `H_ss`/`rhs_s` are not "this frame's block" the way `H_pp` is — no frame
  owns the shared corner, each just adds into it, which is exactly why they
  can be dropped when only the frame's own row is wanted. This is what pass
  1 builds. Either type lives on the stack for the duration of processing
  one frame; neither is ever stored in an array across frames.
- **`SharedNormalEquations<P>`** — the ONLY state that persists across the
  whole frame set during one solver iteration: `H` (P×P), `rhs` (P). Built
  via `absorb_frame()` (folds one `FrameNormalEquations<P>` in via Schur
  elimination, per pass-1 frame) and `add_prior()` (ridge regularization,
  mirroring `RegularizationSigmas`). `solve()` gives `dx_shared`.
- **`solve_frame_pose_update<P>()`** — free function, takes a freshly-rebuilt
  `FramePoseBlock<P>` (pass 2) plus the solved `dx_shared`, returns that
  frame's own 6-DOF pose update. Taking the pose block rather than the full
  system is what makes the cheap rebuild expressible: a caller *cannot* hand
  it an `H_ss`, so it has no reason to have spent anything computing one.

Peak memory for one solver iteration: one `SharedNormalEquations<P>`
(P² + P floats, 8,280 bytes at P=45) + one frame at a time — 9,528 bytes
during pass 1 (`FrameNormalEquations<P>`, P² + 7P + 6 floats), 1,248 bytes
during pass 2 (`FramePoseBlock<P>`, 7P + 42 floats). Independent of N in
both passes.

**These cannot be stack locals on this device, and an earlier version of
this document said they were.** The RP2040's stack is 4 KB *per core* --
`memmap_default.ld` puts core0's in SCRATCH_Y and core1's in SCRATCH_X, both
`LENGTH = 4k` -- so a `FrameNormalEquations<45>` at 9,528 bytes overflows it
2.3x over and `SharedNormalEquations<45>` at 8,280 bytes 2x over. Declaring
either as an ordinary local would smash the stack on the first call, and
with no MPU on a Cortex-M0+ it would corrupt whatever sits below rather than
fault cleanly at the point of the bug.

Both therefore have to be statically allocated (file-scope, or members of a
long-lived solver object) rather than automatic. That does not change the
O(P²)-not-O(P·N) argument this design turns on -- the whole point is still
that only one frame's worth exists at a time -- it changes *where* those
bytes live. Note the pass-1/pass-2 split helps here for a second, unplanned
reason: `FramePoseBlock<45>` at 1,248 bytes is the one structure that would
comfortably fit on the stack.

The 193 KB free-RAM figure above is the budget these static allocations come
out of, so ~18 KB of solver state against it remains comfortable. The
constraint is the stack specifically, not total RAM.

## Scope of this document

This is the accumulator shape and the algebra it implements. The outer
solver — damping, step acceptance, convergence, and the wiring that turns raw
per-sensor derivatives into the P-wide columns `add_sensor` expects — is not
here. `TODO/on-device-calibration.md` owns the full list of what is and isn't
built, and the open questions.

One design point belongs here rather than there, because it is about this
shape specifically: **the dense P×P is deliberate, not an oversight.** Most
shared-parameter groups touch only one sensor's 3 rows per frame — only
magnet position and the strength mean are genuinely dense across all three —
so a frame's real contribution is far sparser than the P×P this forms.
Exploiting that needs the concrete column layout settled first, and it is a
strict optimization on top of this: same algorithm, same answer, less
arithmetic. Getting the exact general version right first is what makes a
sparse version checkable against something.
