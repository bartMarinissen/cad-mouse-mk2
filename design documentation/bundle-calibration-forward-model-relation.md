# On-Device Bundle Calibration — How the Full Model Relates to the Pose Solver

Written from reading the current forward-model code (`firmware/src/magnet_model/`)
against `design documentation/Math.md` and against the prior session's notes in
`design documentation/on-device-bundle-calibration.md`. Supersedes that file
wherever the two disagree — its own header explains why: it predates the
cross-magnet interference model, and that is exactly what changes below.

## 1. The reuse point is one level lower than the prior session assumed

The prior session's plan (its §2.1) recovers a magnet's field gradient $M$
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
(its §2.2) across *both* branches, but it needs one fact the prior session
had no reason to check, since dipole cross-terms didn't exist yet: the dipole
field is rotation-*equivariant*, $B(Q\mathbf m,Q\mathbf r)=Q\,B(\mathbf m,\mathbf r)$
for any rotation $Q$. A magnet's polarization is always its own local
$-\hat z$, so the dipole's *local* moment never depends on tilt — only its
*world* moment does, through $R_{total,j}$. That makes
$\mathbf B_{w,ij}=R_{total,j}B_j(\mathbf v_{l,ij})$ (Math.md §2's general form)
hold for the far branch too, not only the near/interpolated one it was
written for. Consequence: the position derivative

$$\frac{\partial \mathbf B_{ij}}{\partial \mathbf m_j} = -M_{ij}\,R$$

holds unmodified for both branches, and the tilt derivative (prior session's
§2.2) generalizes the same way — *provided* $\mathbf d$ (the magnet-to-sensor
vector the formula needs) is taken relative to the frame each branch actually
uses: `origin_world` for the near branch, `centre_world` for the far one.
That distinction is new; nothing before cross-magnet modelling would have
surfaced it.

## 3. Two forward models, one field, one test

| | Pose solver's forward model | Calibration's forward model |
|---|---|---|
| Solves for | one frame's $(\mathbf t,R)$ | $N$ frames' $(\mathbf t_k,R_k)$ + 27 shared params |
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

## 4. Where the 27 shared columns come from

| Group | Cols | Derivative | Reuses |
|---|--:|---|---|
| magnet position | 3 (of 9, gauge-reduced) | $-M_{ij}R$ | $M_{ij}$ from either `*_approx_world` call |
| magnet tilt | 2 per magnet (6) | $M_{ij}R[\mathbf d]_\times - R[\mathbf b]_\times$, chart-projected | same, + gnomonic chart (prior session's §2.5, unaffected by cross-magnet) |
| strength, common + spread | 3 | $\mathbf B_{ij}/s_j \cdot s_{nom}$ | $\mathbf B_{ij}$ from either call — both branches linear in $s_j$ |
| sensor offset | 9 | see §5 | raw reading only |
| sensor gain | 6 (of 9, traceless-diagonal) | see §5 | raw reading only |

Position, tilt and strength are the only groups that touch the field model,
and each is one 3×3 (or a field value) already computed by the calls in §1 —
no new field lookups, matching the prior session's efficiency claim, just
sourced one layer down from where it expected.

**Gain is now diagonal** — $G_i=\mathrm{diag}(g_{i,x},g_{i,y},g_{i,z})$, 3 raw
numbers per sensor instead of 9. The gauge story from the prior session's §1
is unchanged, just smaller: the isotropic direction ($g_x=g_y=g_z$) is still
the same degree of freedom as magnet strength, still fixed by the same
`det(G)=1`-style gauge, still leaving strength to own absolute scale — it's
just that "isotropic" now lives inside a 3-dimensional space of diagonal
matrices instead of a 9-dimensional space of general ones. That leaves 2
free (traceless-diagonal) params per sensor rather than 8, so the group
drops from 24 columns to 6, and the total shared-parameter count drops from
45 to 27.

## 5. Gain and offset: fit the convention the firmware actually uses

The prior session assumed the PC fit's convention, prediction
$\hat{\mathbf B}=G\mathbf B_{model}+\text{offset}$, and warned (its §2.4)
that gain then multiplies every field-derived column — a documented,
easy-to-drop factor. The firmware doesn't use that convention.
`SensorController::read_mT()` applies gain to the *measurement*:
`corrected = sensor_gain·raw − sensor_offset_mT`. Written into the residual
the pose solver already minimizes ($\mathbf r=\hat{\mathbf B}-\mathbf B_{measured}$,
`solve_pose.cpp`), fitting on-device against that same convention gives

$$\mathbf r_i = (G_i\,\mathbf{raw}_i - \mathbf{offset}_i) - \mathbf B_{w,i}(\theta_{geom},\rho)$$

$$\frac{\partial \mathbf r_i}{\partial \mathbf{offset}_i} = -I, \qquad \frac{\partial \mathbf r_i}{\partial g_k} = G_k\,\mathbf{raw}_i$$

With gain now diagonal, $G_k$ is one of the 2 traceless *diagonal* basis
matrices per sensor rather than one of 8 general traceless ones, so
$G_k\,\mathbf{raw}_i$ is an elementwise scale-and-pick of the raw reading's 3
components — no matrix-vector product at all, just as cheap as the offset
column.

Gain and offset columns now act only on a stored raw reading; every
field-derived column (position, tilt, strength, pose) is untouched by gain
entirely. The prior session's §2.4 pitfall — a column scaled twice, or a
missing $G$ — has no way to occur in this formulation, since there is no
shared factor between the two column families to forget. This also matches
`TODO/resolved/sensor-gain-calibration.md`'s existing decision to keep
correction at the sensor, not the model — the calibration forward model
should follow the same split rather than reintroduce gain inside it.

## 6. Left open

Two things this exploration surfaced but didn't resolve, both blocking before
any solver code gets written:

- **The arrowhead/Schur elimination algebra has no home yet.** The prior
  session cites "Math.md §7" for it (its §3.1); `Math.md` currently ends at
  §6. That derivation needs to exist somewhere citable before the two-pass
  solver (prior session's §3) can be implemented against it.
- **On-device frame volume vs. RAM.** `BundleCalibrationController` currently
  streams roughly a second of samples per step across 7 steps to the PC —
  enough frames that storing all of them, rather than the two-pass scheme's
  rebuild-from-raw-reading approach, is very unlikely to fit. Not a blocker
  for the math above, but the two-pass design's premise should be checked
  against real capture counts before committing to it.

## Addendum: two equally valid framings of the tilt collapse

§4's tilt column is $N_{ij}[\mathbf d]_\times A_j - [\mathbf B_{ij}]_\times(RA_j)$, where
$A_j$ is the fixed knob-frame map from $(u,v)$ to a rotation-perturbation
vector $\boldsymbol\varepsilon_j$. There are two correct ways to describe where
$RA_j$ comes from, and it's worth having both on record rather than re-deriving
this the next time it comes up:

- **$\boldsymbol\varepsilon_j$ stays knob-frame; $R\boldsymbol\varepsilon_j$ is a second,
  world-frame representation of that same physical rotation**, used only by
  the term that needs one. This is the framing that explains *why* $(u,v)$
  can't be defined in world frame in the first place: it has to mean the same
  physical tilt in every captured frame, and only a knob-frame (pose-independent)
  definition guarantees that.
- **Equivalently: for that one term, $(u,v)$ is mapped to a rotation directly
  in the world frame**, via $RA_j$ instead of $A_j$. This is the more direct
  answer to "is this a world-frame rotation" — yes, for that term's purposes.

Both describe the same computation. The first is the one to reach for when
the question is what $(u,v)$ *means*; the second is the one to reach for when
the question is what a specific intermediate *is*.
