# On-Device Bundle Calibration — How the Full Model Relates to the Pose Solver

*Written this session, from reading the current forward-model code
(`firmware/src/magnet_model/`) against `design documentation/Math.md`.
Supersedes the older session's notes below wherever the two disagree — the
disclaimer that session's summary carries is exactly why.*

## 1. The reuse point is one level lower than the prior session assumed

The prior session's plan (§2.1 below) recovers a magnet's field gradient $M$
by reading it off `ForwardModel::evaluate()`'s returned pose Jacobian (its
translation block). That was true before cross-magnet interference existed.
It no longer is: `ForwardModel::evaluate()` now sums all three magnets'
contributions into a single 3×3 per sensor before returning
($J_{trans,i}=-M_i,\ M_i=\sum_j M_{ij}$, `virtual_sensor.cpp`'s
`J_displacement_world`) — the summation Math.md §4.F derives as exact but,
correspondingly, irreversible. A magnet-position or -tilt derivative needs
that one magnet's own $M_{ij}$, not the sum, so it cannot be recovered from
`ForwardModel::evaluate()`'s output anymore.

It doesn't need to be. `MagnetPlacement::near_approx_world()` and
`::far_approx_world()` (`magnet_local_model.{h,cpp}`) already return exactly
that, unsummed, and are already public methods that `VirtualSensor::evaluate()`
itself calls once per (sensor, magnet) pair, before it sums them:

```
B_ij, M_ij = placement_j.near_approx_world(sensor_i)   // paired magnet
B_ij, M_ij = placement_j.far_approx_world(sensor_i)    // cross magnets
```

This is the actual reuse boundary: calibration code calls these two methods
directly, once per (sensor, magnet, frame), and never touches
`ForwardModel::evaluate()`'s summing loop for the shared-parameter columns.
It still reuses `MagnetModel::place()` unchanged, once per magnet per frame.

## 2. One formula, both branches — why that's true, and wasn't obvious

Both `near_approx_world` and `far_approx_world` share an interface: world
position in, field and $\partial B/\partial(\text{world position})$ out. That
alone is enough to reuse the prior session's magnet position/tilt chain rule
(§2.2 below) across *both* branches, but it needs one fact the prior session
had no reason to check, since dipole cross-terms didn't exist yet: the dipole
field is rotation-*equivariant*, $B(Q\mathbf m,Q\mathbf r)=Q\,B(\mathbf m,\mathbf r)$
for any rotation $Q$. A magnet's polarization is always its own local
$-\hat z$, so the dipole's *local* moment never depends on tilt — only its
*world* moment does, through $R_{total,j}$. That makes
$\mathbf B_{w,ij}=R_{total,j}B_j(\mathbf v_{l,ij})$ (Math.md §2's general form)
hold for the far branch too, not only the near/interpolated one it was
written for. Consequence: the position derivative

$$\frac{\partial \mathbf B_{ij}}{\partial \mathbf m_j} = -M_{ij}\,R$$

holds unmodified for both branches, and the tilt derivative below (§2.2)
generalizes the same way — *provided* $\mathbf d$ (the magnet-to-sensor
vector the formula needs) is taken relative to the frame each branch actually
uses: `origin_world` for the near branch, `centre_world` for the far one.
That distinction is new; nothing before cross-magnet modelling would have
surfaced it.

## 3. Two forward models, one field, one test

| | Pose solver's forward model | Calibration's forward model |
|---|---|---|
| Solves for | one frame's $(\mathbf t,R)$ | $N$ frames' $(\mathbf t_k,R_k)$ + 45 shared params |
| Per-frame block (6 cols) | `ForwardModel::evaluate()` | the same call, unchanged |
| Shared-param block (45 cols) | doesn't exist | new: built from `near_/far_approx_world` |
| Underlying field/Jacobian math | Math.md §3–4 | same, chain rule extended to the shared params |

The pose-solver forward model is a special case of the calibration one, at
$\theta=\theta_{nominal}$, one frame. That gives the agreement test the user
asked for almost for free: evaluate the calibration model's field prediction
and its 6 pose columns at nominal $\theta$, for one frame, and require it to
match `ForwardModel::evaluate()`'s own output to float tolerance. Because the
pose columns literally are that same call, this is less "agreement between
two implementations" and more "the new code didn't disturb the old call
path" — which is the point: there is one forward model, evaluated over a
wider parameter vector, not two forward models kept in sync by hand.

## 4. Where the 45 shared columns come from

| Group | Cols | Derivative | Reuses |
|---|--:|---|---|
| magnet position | 3 (of 9, gauge-reduced) | $-M_{ij}R$ | $M_{ij}$ from either `*_approx_world` call |
| magnet tilt | 2 per magnet (6) | $M_{ij}R[\mathbf d]_\times - R[\mathbf b]_\times$, chart-projected | same, + gnomonic chart (§2.5 below, unaffected by cross-magnet) |
| strength, common + spread | 3 | $\mathbf B_{ij}/s_j \cdot s_{nom}$ | $\mathbf B_{ij}$ from either call — both branches linear in $s_j$ |
| sensor offset | 9 | see §5 | raw reading only |
| sensor gain | 24 (of 27, traceless) | see §5 | raw reading only |

Position, tilt and strength are the only groups that touch the field model,
and each is one 3×3 (or a field value) already computed by the calls in §1 —
no new field lookups, matching the prior session's efficiency claim, just
sourced one layer down from where it expected.

## 5. Gain and offset: fit the convention the firmware actually uses

The prior session assumed the PC fit's convention, prediction
$\hat{\mathbf B}=G\mathbf B_{model}+\text{offset}$, and warned (§2.4 below)
that gain then multiplies every field-derived column — a documented,
easy-to-drop factor. The firmware doesn't use that convention.
`SensorController::read_mT()` applies gain to the *measurement*:
`corrected = sensor_gain·raw − sensor_offset_mT`. Written into the residual
the pose solver already minimizes ($\mathbf r=\hat{\mathbf B}-\mathbf B_{measured}$,
`solve_pose.cpp`), fitting on-device against that same convention gives

$$\mathbf r_i = (G_i\,\mathbf{raw}_i - \mathbf{offset}_i) - \mathbf B_{w,i}(\theta_{geom},\rho)$$

$$\frac{\partial \mathbf r_i}{\partial \mathbf{offset}_i} = -I, \qquad \frac{\partial \mathbf r_i}{\partial g_k} = G_k\,\mathbf{raw}_i$$

Gain and offset columns now act only on a stored raw reading; every
field-derived column (position, tilt, strength, pose) is untouched by gain
entirely. The §2.4 pitfall below — a column scaled twice, or a missing $G$ —
has no way to occur in this formulation, since there is no shared factor
between the two column families to forget. This also matches
`TODO/resolved/sensor-gain-calibration.md`'s existing decision to keep
correction at the sensor, not the model — the calibration forward model
should follow the same split rather than reintroduce gain inside it.

## 6. Left open

Two things this exploration surfaced but didn't resolve, both blocking before
any solver code gets written:

- **The arrowhead/Schur elimination algebra has no home yet.** The prior
  session cites "Math.md §7" for it (§3.1 below); `Math.md` currently ends at
  §6. That derivation needs to exist somewhere citable before the two-pass
  solver (§3 below) can be implemented against it.
- **On-device frame volume vs. RAM.** `BundleCalibrationController` currently
  streams roughly a second of samples per step across 7 steps to the PC —
  enough frames that storing all of them, rather than the two-pass scheme's
  rebuild-from-raw-reading approach, is very unlikely to fit. Not a blocker
  for the math above, but the two-pass design's premise should be checked
  against real capture counts before committing to it.

---

Below is a summary of the approach taken by an older claude session, written by that same session.

That session ran into issues, so it was abandoned. Since then one significant piece of math changed:
we now model cross magnet interference. Another piece changed: we now use BLA instead of eigen.

Take this as a good source for first things to try. Not as gospel — see §1–§6 above for where it
has already been checked against the current code and where it needed correcting.

# Appendix: prior session's notes — Deriving the Jacobians, and the Two-Pass Solver

Calibration fits the constants the pose solver treats as frozen — where each
magnet sits in the knob, how it is tilted, how strong it is, and each sensor's
gain and DC offset — jointly across many captured frames, each of which also
contributes its own unknown pose.

This document covers two things a reimplementation needs and cannot get from
reading the pose solver alone:

1. **How to derive the calibration Jacobians by reusing the existing
`evaluate()` chain**, rather than writing a second forward model.
2. **How the resulting normal equations are solved** — a two-pass
Schur-complement scheme that never materializes the full system.

`Math.md` owns the forward model and the pose Jacobian (§1–6) and the algebra
of the arrowhead normal equations (§7). This document is the layer between:
the derivatives §7 declines to derive, and the practical shape of the solver
§7 only sketches.

\---

## 1\. What is being fitted

Following the PC-side fit in `magnet\_field\_model/calibration/`:

|group|free parameters|note|
|-|-:|-|
|magnet position|3|9 raw (3 magnets × xyz) minus a 6-DOF gauge|
|magnet tilt|6|2 per magnet; spin about a magnet's own axis is unobservable|
|magnet strength, common|1|the absolute field scale|
|magnet strength, spread|2|zero-sum deviations from that common level|
|sensor offset|9|3 per sensor|
|sensor gain|24|8 per sensor|

**45 shared parameters, plus 6 per frame.** All are *offsets from nominal*, so
the zero vector is the CAD/nominal configuration.

Three of those counts are reductions, and the reasons matter:

* **Position: 9 → 3.** Translating or rotating the whole magnet trio is exactly
cancelled by a compensating change in every frame's pose, so 6 of the 9 DOF
are unobservable. The fix is structural — project onto the 3-dimensional
nullspace complement — rather than relying on a prior to pin them.
* **Tilt: 3 → 2 per magnet.** A magnet is a solid of revolution about its
polarization axis, so spin about that axis changes nothing measurable. This
is an *exact* symmetry, not a weak direction.
* **Gain: 9 → 8 per sensor.** Overall gain scale and magnet strength are the
same physical degree of freedom. The split is fixed by a gauge: gain carries
no isotropic component (8 traceless basis matrices), and `magnet\_strength`
owns absolute scale instead.

Strength is best carried as a **dimensionless multiplier** around 1, not an
absolute mT offset. Both physical beliefs about it are relative — "magnets from
one batch are graded to within a few percent of each other", "the absolute
remanence could be anywhere from 500 to 1500 mT" — so a multiplier is the
parameterization those beliefs are actually stated in, and prior widths
transfer between implementations without a scale factor.

\---

## 2\. Deriving the Jacobians from the existing forward model

### 2.1 The reuse that makes this cheap

The expensive part of a forward evaluation is the bicubic field lookup. The
naive approach — differentiating the calibration constants from scratch —
would repeat it. It does not need to.

`Math.md` §4.E defines the shared 3×3 core

$$M = R\_{total},J\_{local},R\_{total}^T$$

and the pose Jacobian's translation block is exactly $-M$ (pre-gain). So a
forward evaluation *already computes and returns* everything the calibration
derivatives need. Recover it from the output:

```
M  =  -(translation block of the returned pose Jacobian)
```

No second field lookup, no reaching into the model's internals. Every
chain-rule group below is one 3×3 product away from data already in hand,
which is what makes the bundle forward problem a thin wrapper around the
existing one rather than a parallel implementation to keep in sync.

### 2.2 The three groups that need the chain rule

Work in the knob frame. With

* $\\mathbf v = \\mathbf s\_i - \\mathbf t$ — sensor position relative to the knob origin, global frame
* $\\mathbf d = R^T\\mathbf v - \\mathbf m\_j$ — magnet-to-sensor vector, knob frame
* $\\mathbf b = R^T \\mathbf B\_g$ — the predicted field, knob frame
* $s$ — the magnet's strength

$$\\frac{\\partial \\mathbf B\_g}{\\partial \\mathbf m\_j} = -M R
\\qquad
\\frac{\\partial \\mathbf B\_g}{\\partial \\boldsymbol\\varepsilon} = M R \[\\mathbf d]*\\times - R\[\\mathbf b]*\\times
\\qquad
\\frac{\\partial \\mathbf B\_g}{\\partial s} = \\frac{\\mathbf B\_g}{s}$$

where $\\boldsymbol\\varepsilon$ is a **left** perturbation of the magnet's
rotation, $R\_{mag} \\to \\exp(\[\\boldsymbol\\varepsilon]*\\times)R*{mag}$ — the same
convention `Math.md` Identity 2 uses for the pose rotation, so tilt and pose
step the same way.

Derivation sketch: start from
$\\mathbf B\_g = R,R\_{mag},B!\\left(R\_{mag}^T(R^T\\mathbf v - \\mathbf m\_j)\\right)$
and differentiate with respect to $\\mathbf m\_j$ and to $\\boldsymbol\\varepsilon$.
The strength column is not a chain rule at all — strength enters the field as a
scalar multiplier, so the derivative is the prediction rescaled.

Note the tilt expression's second term. $R\_{mag}$ cancels out of
$\\mathbf b = R\_{mag}^T R^T \\mathbf B\_g$ rotated back, which is why the formula
needs only the knob-frame field and not any local-frame intermediate.

**Keep this layer 3-DOF.** The derivative above is a full 3×3 — how the field
responds to *any* rotation of the magnet. That is a fact about physics with no
modelling choice in it. Which 2-D subspace gets fitted is a chart decision and
belongs in the caller. Fusing them means re-deriving and re-verifying physics
every time the chart changes, and it collapses the two-tier test structure of
§4 into one untestable step.

### 2.3 The five groups that need no model call

Sensor offset and the three gain groups are linear maps of quantities the
forward pass already produced:

$$\\frac{\\partial \\hat{\\mathbf B}\_i}{\\partial (\\text{offset}\_i)} = I
\\qquad
\\frac{\\partial \\hat{\\mathbf B}*i}{\\partial g\_k} = G\_k \\mathbf B*{g,i}$$

where $G\_k$ is the $k$-th traceless gain basis matrix. These belong to a
*physical sensor*, so they are touched only by that sensor's own three rows —
a structural fact that survives any change to how magnets couple to sensors.

The strength mean/diff split is bookkeeping over the already-computed
per-magnet strength column, weighted by each magnet's coefficient in the two
strength bases.

### 2.4 Two chain-rule factors that are easy to drop

Both of these produce a plausible-looking fit rather than an obvious failure,
because Levenberg-Marquardt absorbs the error by mis-fitting everything else.

**Gain multiplies the field-derived columns.** The prediction is
$\\hat{\\mathbf B} = G\\mathbf B\_g + \\text{offset}$, so every derivative acting
through the field carries a factor of $G$:

$$\\frac{\\partial (G\\mathbf B\_g)}{\\partial(\\text{magnet param})} = G,\\frac{\\partial \\mathbf B\_g}{\\partial(\\text{magnet param})}$$

The forward model returns a **pre-gain** derivative. This cannot be applied by
scaling the assembled row, because the gain's own columns are already complete
derivatives and must not be scaled twice. `Math.md` §4.E flags the same trap
for the pose Jacobian, where $\[\\mathbf B\_g]\_\\times$ must be built from the
pre-gain field. The pose block carries the same factor of $G$ here.

**A multiplier strength parameter needs the nominal strength.** The forward
model returns $\\partial \\mathbf B/\\partial s$, an *absolute* derivative. If the
fitted parameter is a dimensionless multiplier (§1), then

$$\\frac{\\partial \\mathbf B}{\\partial(\\text{multiplier})} = \\frac{\\partial \\mathbf B}{\\partial s}\\cdot s\_{nominal}$$

Omitting it makes the strength columns wrong by the nominal strength — a
factor of \~1000.

### 2.5 Parameterizing tilt

The measurable configuration of an axially-polarized magnet is the *direction*
of its axis — a point on $S^2$, two degrees of freedom. Any two-parameter chart
works in principle; what differs is where it degenerates.

**Spherical / Euler angles are a poor choice here**, because their singularity
is at the pole and the pole is nominal. With
$\\hat n = (\\sin\\theta\\cos\\varphi, \\sin\\theta\\sin\\varphi, \\cos\\theta)$ the
azimuth column scales as $\\sin\\theta$, so at a tolerance-sized tilt of a degree
or two its weight in the normal equations is \~$10^{-3}$ of the polar column's.
The ridge prior swallows it and two nominal DOF quietly become about one
fitted.

**A gnomonic (pinhole) chart avoids this.** Name the axis by where its line
crosses a plane one unit from the magnet, in the nominal magnet's frame:

$$\\hat n(u,v) = \\frac{(-u,\\ -v,\\ 1)}{\\sqrt{1+u^2+v^2}}$$

with nominal at $(0,0)$. Its derivative there is an orthonormal basis of the
tangent plane — perfectly conditioned — and its singularity sits at 90°, which
the chart cannot reach since $n\_z > 0$ for all finite $(u,v)$. The coordinates
also read physically: an offset in millimetres of where the axis lands.

The chart Jacobian converts the 3×3 above into the two fitted columns. A left
perturbation moves the axis by $d\\hat n = \\boldsymbol\\varepsilon \\times \\hat n$,
and components of $\\boldsymbol\\varepsilon$ along $\\hat n$ (spin) move it not at
all, so the canonical spin-free choice is
$\\boldsymbol\\varepsilon = \\hat n \\times d\\hat n$:

$$\\frac{\\partial \\boldsymbol\\varepsilon}{\\partial(u,v)} = \[\\hat n\_{knob}]*\\times, R*{nominal}, \\frac{\\partial \\hat n\_{local}}{\\partial (u,v)}$$

Because these columns are built as $\[\\hat n]\_\\times(\\cdot)$, they are
perpendicular to the magnet's *current* axis at any tilt, by construction —
spin is unrepresentable rather than approximately projected away. Contrast the
obvious alternative of dropping a fixed coordinate column, which annihilates
the dead direction only at zero tilt.

The magnet's rotation matrix becomes a *derived* quantity, rebuilt from the
two parameters by a shortest-arc lift from nominal. It therefore cannot
accumulate orthogonality drift, and there is no rotation-vector-versus-tangent
-space convention to reconcile: you differentiate your own chart.

### 2.6 Where each piece is computed

Two nested loops, and the split matters for performance:

* **Per magnet, per solver iteration** (3 calls): rebuild the magnet model and
the chart Jacobian from the current parameters. These depend only on the
magnet's own parameters, not on any frame.
* **Per (sensor, frame)** (\~3N calls): one forward evaluation, then the
derivative formulas above.

Rebuilding the magnet model inside the frame loop instead is a measurable
waste — model construction is a meaningful fraction of one evaluation's cost,
and doing it per-frame pays that fraction tens of times over.

\---

## 3\. The two-pass Schur solver

### 3.1 Why the normal equations are arrowhead-shaped

Frame *k*'s residual depends on the shared parameters and on **its own pose
only** — never on another frame's. Differentiating gives a stacked Jacobian
whose off-diagonal pose blocks are exactly zero, and $J^TJ$ inherits that
structure: a dense shared block, one small pose block per frame, thin coupling
blocks, and nothing else.

`Math.md` §7 derives this properly and shows the elimination is an exact
linear-algebra identity rather than an approximation. The rest of this section
assumes that result.

Worth stating the assumption explicitly, since it is the one thing that would
break everything: **no term may couple two frames.** A temporal-smoothness
prior linking consecutive frames would violate it. Modelling cross-magnet
interference would *not* — that coupling acts within a frame, between a sensor
and a non-paired magnet, and never links frame *k* to frame *l*.

### 3.2 The two passes

Per solver iteration:

**Pass 1 — accumulate.** For each frame: build its local system from its
sensors' rows, then fold it into a persistent shared accumulator, eliminating
that frame's pose block. Discard the frame's data. After every frame, add the
ridge/prior terms and the LM damping, then solve the reduced shared system
once.

**Pass 2 — back-substitute.** For each frame: rebuild its pose row and recover
that frame's own pose update from the now-known shared update.

Nothing per-frame persists between the passes. Peak memory is one accumulator
plus one frame's worth of transient blocks — independent of the frame count,
which is the entire point. The direct alternative is a dense system of side
(shared + 6 × frames), which at realistic sizes does not fit in this device's
RAM at all.

### 3.3 Three details that are easy to get wrong

**Pass 2 must linearize where pass 1 did.** The shared update and every pose
update may only be applied to the iterate *after* pass 2 has finished for every
frame. Update early and each frame's blocks come from a different linearization
than the reduced system that produced the shared update, and the
back-substitution silently stops being the exact identity it is derived as.
This is a hazard of rebuilding specifically — a design that stored the blocks
could not get it wrong, because stored blocks carry their linearization point
with them.

**Pass 2 is the cheaper pass, and the types should say so.** Back-substitution
reads only the pose block, the coupling block and the pose right-hand side —
never the shared block, which is by far the most expensive term to accumulate.
Giving pass 2 a distinct, smaller type makes that structural: a caller cannot
accidentally pay for a shared block it will not read, and cannot forget it in
pass 1.

**Rebuild rather than store.** Keeping every frame's coupling block between
the passes would cost tens of kilobytes for data needed microseconds later.
Rebuilding costs a second forward-model pass over the frames, which is a good
trade for a once-per-unit operation that is not latency-sensitive.

### 3.4 Solving the reduced system

A plain dense factorization of the reduced shared system is adequate. The
shared block has exploitable structure of its own — sensor-local parameters
(offset, gain) are touched only by their own sensor, making it bordered
block-diagonal — but eliminating that too optimizes a term that is a small
percentage of an iteration, since the factorization is cubic in \~45 while the
*accumulation* it sits on top of is far larger.

That structure is still worth having, for a different reason: it makes the
**accumulation** sparse. One sensor's rows touch roughly 40% of the shared
columns, so a sparsity-aware accumulator is several times cheaper on the term
that actually dominates. Structure for accumulation, dense solve.

If the column order is chosen to consolidate that structure — parameters
grouped by the unit they belong to, rather than by parameter type — each
sensor's live columns form two contiguous runs instead of many scattered ones,
which is what lets the accumulation use fixed-size block operations. Note that
the *number* of nonzeros is a property of the problem and is unaffected by
ordering; what ordering buys is that they are contiguous.

\---

## 4\. Verifying it

Hand-derived derivatives fail silently: a wrong sign or a dropped term does not
crash, it makes the solver converge slowly to a slightly wrong answer.

**Finite differences against the real forward model**, in the style of
`firmware/test/test\_jacobian.cpp`. Two tiers are needed, and the second is the
one that catches wiring:

1. Each raw derivative against perturbing that one physical quantity.
2. The *assembled* parameter column against perturbing the real parameter —
through the gauge projection, the chart, and the column layout. Tier 1
passes happily with a transposed basis or an off-by-one magnet index.

**The Schur elimination is checkable exactly**, not by finite difference. It is
a linear-algebra identity, so assemble the full arrowhead system densely, solve
it in one shot, and require the frame-by-frame path to agree to float32
precision. Finite differences would be a strictly weaker check of something
exactly checkable.

Four things that cost real time to discover, all worth knowing in advance:

* **Probe poses must be inside the bicubic table's domain.** The knob rests
about 21mm above the sensor plane; a hand-picked pose can easily land outside
the table, where the field is extrapolated nonsense. Every finite-difference
test still *passes* out there, because analytic-vs-numeric consistency holds
in the extrapolation region too. What fails is the solver, which stalls on
the extrapolated surface. Derive test poses from the nominal rest pose.
* **Finite-difference step sizes carry unit assumptions**, and one step does not
fit all groups. Parameters in field units entering a large prediction
additively need a *large* step or roundoff swamps the difference; tilt moves
the field only by $|B|\\cdot O(h)$ and also needs a larger step than geometry
does. Set each by sweeping and watching which way the error moves — error
*growing* as the step shrinks means the instrument is at fault, not the
algebra.
* **An exact symmetry is not a derivative.** Spin about a magnet's own axis
changes nothing, and differencing two large near-equal floats to extract a
zero is where central differences are worst. Test it as a direct invariance
instead: rotate by real, finite angles and compare outputs.
* **Check synthetic test data for rank.** Generating "arbitrary" matrices as
`sin()` of a linear combination of indices produces matrices that all live in
a 2-D subspace regardless of size, which makes every synthetic system
singular and every comparison meaningless.

A test that shares a helper with the code it tests is a consistency check, not
an oracle — if the same wrong constant feeds both the state being differenced
and the Jacobian being checked, they cancel. Transcribe the basis constants
independently in the test. Mutation testing is how to find out which of your
tests actually bite.

\---

## 5\. Practical notes

**Stack pressure is a real constraint.** This core's stack is small — single
kilobytes — and C++ makes it easy to exceed without writing anything that looks
large: value-returning factorizations, accumulator resets via assignment from a
temporary, and matrix expression temporaries all land there. Compiling with
`-Wstack-usage` turns this into a build failure rather than something to
reason about, and is worth doing from the start.

**Absolute field scale is weakly determined without cross-magnet coupling.**
Cross-talk is what separates magnet *strength* from magnet *distance* — a
strength change scales near and far contributions equally, a height change does
not. Without it the two are near-degenerate over the few millimetres of heave
the hardware allows. This does not hurt fit quality, only the physical meaning
of the fitted numbers. See `TODO/cross-magnet-interference.md`.

**A prior on the common strength level is not obviously wanted.** Leaving it
unregularized is safe even though it is near-degenerate with position and gain,
because a flat direction only survives if it lies *entirely* within
unregularized coordinates — and every parameter it trades against does carry a
justified prior, which supplies curvature along the whole combined direction.

