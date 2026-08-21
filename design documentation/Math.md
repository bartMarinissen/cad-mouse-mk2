# Forward Model and Jacobian — the Mathematics Behind the Pose Solver

This document derives the mathematics of the 6DOF magnetic pose solver: the forward
model that predicts sensor readings from a candidate pose, and the exact analytic
Jacobian of that model, which drives a Levenberg-Marquardt (damped Gauss-Newton)
optimization loop.

The structure below follows the natural computational order — world pose, to local
geometry, to field lookup, to sensor output — so that each section corresponds to one
conceptual stage of the pipeline.

---

## 1. Notation

Indices: $i$ always denotes a **sensor**, $j$ always a **magnet**. Both run over
$\{1,2,3\}$. Every sensor sees every magnet, so most quantities below carry both.

| Symbol | Meaning |
|---|---|
| $\mathbf{s}_i$ | sensor $i$ position, world/PCB frame (constant) |
| $\mathbf{m}_j$ | magnet $j$ resting position (bottom face), knob frame (constant) |
| $\hat{\mathbf{a}}_j$ | magnet $j$'s polarization axis, knob frame (constant, unit vector) |
| $G_i$ | sensor $i$'s gain/distortion matrix (constant) |
| $\mathbf{t}$ | knob translation, world frame (**solved for**) |
| $R$ | knob rotation matrix (**solved for**) |
| $\boldsymbol{\rho}=(\mathbf t,\boldsymbol\omega)$ | 6DOF pose coordinates: translation + rotation-update vector |
| $R(\boldsymbol\omega)=\exp([\boldsymbol\omega]_\times)$ | rotation as a function of $\boldsymbol\omega\in\mathfrak{so}(3)$ (Rodrigues) |
| $F,\,f_i$ | the forward model, overall and per-sensor (§2) |
| $B_j$ | the knob-frame magnet-field function for magnet $j$ (§3.2–3.4) |
| $\hat{\mathbf{a}}_{w,j} = R\,\hat{\mathbf{a}}_j$ | magnet $j$'s polarization axis, world frame |
| $\mathbf{u}_{ij}$ | sensor $i$'s position relative to magnet $j$, in the **knob** frame (not further rotated into a magnet-local frame — see §3.2) |
| $\mathbf{B}_{w,ij}$ | field at sensor $i$ from magnet $j$, world frame (pre-gain) |
| $\mathbf{B}_{w,i}=\sum_j \mathbf{B}_{w,ij}$ | total field at sensor $i$, world frame (pre-gain) |
| $\hat{\mathbf{B}}_i$ | predicted sensor reading |
| $h$ | magnet half-height; the dipole sits this far above $\mathbf{m}_j$ along the magnet's axis (§3.6) |
| $\boldsymbol{\mu}_j$ | magnet $j$'s dipole moment, world frame (§3.6) |
| $\mathbf{c}_j$ | magnet $j$'s geometric centre, world frame (§3.6) |

Only $\mathbf{t}$ and $R$ are optimization variables. Everything else — $\mathbf{s}_i$,
$\mathbf{m}_j$, $\hat{\mathbf{a}}_j$, $G_i$ — is a frozen calibration constant, and so
contributes **zero** to every derivative below. That fact is used repeatedly to prune
terms.

---

## 2. Functional Definition

The forward model is a single map from a 6-parameter pose to 9 predicted sensor
readings (three 3-axis sensors):

$$F:\mathbb{R}^6\to\mathbb{R}^9, \qquad F(\boldsymbol\rho) = \begin{bmatrix} f_1(\boldsymbol\rho)\\ f_2(\boldsymbol\rho)\\ f_3(\boldsymbol\rho)\end{bmatrix}$$

Its input $\boldsymbol\rho=(\mathbf t,\boldsymbol\omega)\in\mathbb{R}^3\times\mathbb{R}^3$
pairs the translation with a rotation-update vector, related to an actual rotation
matrix through the exponential map $R(\boldsymbol\omega)=\exp([\boldsymbol\omega]_\times)$.
Each $f_i:\mathbb{R}^6\to\mathbb{R}^3$ is the prediction for sensor $i$ alone. A sensor
sees **all three** magnets, and magnetic fields superpose, so it is a sum over $j$ of
the per-magnet composition of every stage in §3:

$$f_i(\boldsymbol\rho) = G_i \sum_{j} R(\boldsymbol\omega)\; B_j\!\Big(R(\boldsymbol\omega)^T(\mathbf{s}_i-\mathbf{t})-\mathbf{m}_j\Big)$$

where $B_j:\mathbb{R}^3\to\mathbb{R}^3$ — defined in §3.2–3.4 — maps a knob-frame query
point to the field magnet $j$ produces there. $B_j$ has the magnet's own tilt
$\hat{\mathbf{a}}_j$ baked in as a fixed parameter rather than as a rotation applied to
the input: unlike the old rotation-matrix representation, there is no longer a second,
magnet-local frame to rotate into. This is why only $R$ — the knob's own rotation, the
actual solved-for quantity — appears above, not a magnet-specific composite.

**Where the sum sits matters.** Superposition is a property of the *field*, so the sum
has to be taken there, at $\mathbf{B}_w$, and not somewhere more convenient further out.
Everything downstream of it — the gain $G_i$, and both Jacobian blocks in §4.F — is
linear in the field, which is what lets the implementation add up the per-magnet
contributions first and apply the outer stages once per sensor rather than once per
pair. §4.F returns to this, because it is the step that makes nine (sensor, magnet)
pairs affordable.

The one term that is *not* a shared composition is $B_j$ itself. For the magnet a
sensor sits under it is the interpolated near field (§3.3–3.4); for the other two it is
a point dipole (§3.6). Which applies is fixed by the geometry, not chosen per
evaluation — see §3.6.

Note that $\boldsymbol\omega$ appears **twice**:
once rotating into the knob frame ($R^T$, inside $B_j$'s argument) and once rotating
back out to world frame ($R$, outside). This is the reason the Jacobian derivation in §4 needs
the product rule rather than a single pass of the chain rule — both occurrences must be
differentiated and their contributions summed.

In practice, the solver never evaluates $F$ over the whole range of $\boldsymbol\rho$: at
each iteration it fixes a current estimate $(\mathbf t_0, R_0)$, treats $\boldsymbol\rho$
as a small *local* perturbation around it —

$$\mathbf{t}(\boldsymbol\rho)=\mathbf{t}_0+\Delta\mathbf{t}, \qquad R(\boldsymbol\rho) = \exp([\Delta\boldsymbol\omega]_\times)\,R_0$$

— and computes the Jacobian $J = D_{\boldsymbol\rho}F\big|_{\boldsymbol\rho=\mathbf 0} \in\mathbb{R}^{9\times6}$
of this local chart. That is precisely what §4 derives, and why it must be recomputed
freshly at every iteration rather than known once in closed form.

---

## 3. The Forward Model, Stage by Stage

This section unpacks $f_i$ from §2 into the sequence of physical steps that compute it,
one sensor/magnet pair at a time.

### 3.1 The magnet's axis in world coordinates

Each magnet is a solid of revolution about its own polarization axis, so spin about
that axis is physically unobservable — it is not represented at all. A magnet's whole
orientation is carried as one unit vector, $\hat{\mathbf{a}}_j$, fixed in the knob
frame. The one place it needs to be in world coordinates is the far-field branch
(§3.6):

$$\hat{\mathbf{a}}_{w,j} = R\,\hat{\mathbf{a}}_j$$

### 3.2 Knob-frame vector geometry

The vector from a magnet to its sensor only needs to reach the *knob* frame, not a
further magnet-local one: the sensor position is expressed in the knob's frame, then
the magnet's own resting offset is subtracted in that same frame.

$$\mathbf{u}_{ij} = R^T(\mathbf{s}_i - \mathbf{t}) - \mathbf{m}_j$$

There is no second rotation here. Compare the old two-rotation form
($R_{m,j}^T\big(R^T(\mathbf s_i-\mathbf t)-\mathbf m_j\big)$, rotating first into the
knob frame and then again into a magnet-local frame) — dropping the second rotation is
exactly what removes the "rotate in, evaluate, rotate out" cost from the near-field
branch, the same way §3.6's dipole always avoided it.

### 3.3 Cylindrical query and interpolation

The magnets are axially polarized, so the field they produce is symmetric around
$\hat{\mathbf{a}}_j$ and depends only on the position relative to that axis. Project
$\mathbf{u}_{ij}$ onto it to get height and radius:

$$z = \hat{\mathbf{a}}_j\cdot\mathbf{u}_{ij}, \qquad \mathbf{u}_{\perp} = \mathbf{u}_{ij} - z\,\hat{\mathbf{a}}_j, \qquad r = |\mathbf{u}_{\perp}|$$

A precomputed, $C^1$-continuous bicubic interpolation table is queried at $(r,z)$,
returning the two cylindrical field components $B_r, B_z$ and their four spatial
partials $\partial B_r/\partial r$, $\partial B_z/\partial r$, $\partial B_r/\partial z$,
$\partial B_z/\partial z$.

### 3.4 Field reconstruction

The scalar cylindrical field is reassembled directly along $\hat{\mathbf{a}}_j$ and the
radial direction $\hat{\mathbf{u}}_\perp = \mathbf{u}_\perp/r$ — there is no arbitrary
in-plane reference direction to construct, because the field has no component
perpendicular to both (no azimuthal component, by the same axial symmetry §3.3 uses).
Together, §3.2–3.4 constitute the function $B_j$ from §2:

$$\mathbf{B}_{ij} = B_j(\mathbf{u}_{ij}) = B_z\,\hat{\mathbf{a}}_j + B_r\,\hat{\mathbf{u}}_\perp$$

### 3.5 Back to world frame, then gain

The knob-frame field is rotated into the world frame by $R$ — the knob's own rotation,
and the *only* rotation this branch ever applies — summed over magnets, then passed
through the sensor's own gain/distortion matrix to give the predicted sensor reading:

$$\mathbf{B}_{w,ij} = R\, \mathbf{B}_{ij}, \qquad \mathbf{B}_{w,i} = \sum_j \mathbf{B}_{w,ij}, \qquad \hat{\mathbf{B}}_i = f_i(\boldsymbol\rho) = G_i\, \mathbf{B}_{w,i}$$

(An additive sensor baseline/offset, if calibrated, is handled upstream as a
correction to the raw measurement rather than as a term in this model.)

### 3.6 The far-field branch — magnets a sensor does not sit under

§3.2–3.4 is the model for the magnet a sensor is paired with. The other two sit a
knob-triangle side away (28.58mm, `Positions::triangle_sidelength_mm`), far outside the
interpolation grid's domain, and are modelled as **ideal point dipoles**:

$$\mathbf{B} = \frac{1}{4\pi}\,\frac{3(\boldsymbol\mu\cdot\hat{\mathbf{r}})\hat{\mathbf{r}} - \boldsymbol\mu}{|\mathbf{r}|^{3}}$$

**This is the only approximation in this document.** Every other stage is exact given
its inputs; this one replaces a finite cylinder with a point. Its error is 0.14–0.40%
of the cross-magnet term at the geometry it is used at — and since that term is itself
1.7–4.5% of the signal, under 0.02% of the total field, comfortably below the ~0.3%
residual the fit achieves. The Python calibration model does *not* make this
approximation: it evaluates all nine pairs with the exact cylinder solution, which is
what the firmware's dipole is measured against.

Two details are load-bearing rather than incidental:

- **The dipole sits at the magnet's geometric centre**, not at $\mathbf{m}_j$. The knob
  frame's origin for this offset is the magnet's *bottom face* (§3.2), so the centre is
  half a magnet higher **along the magnet's own axis**:
  $\mathbf{c}_j = \mathbf{t} + R\,(\mathbf{m}_j + h\,\hat{\mathbf{a}}_j)$. Placing it at
  the bottom face instead is a ~50% error at cross-magnet range, not a refinement.
- **The moment is the same magnet the table describes.** $|\boldsymbol\mu_j|$ is the
  reference moment (polarization × volume, in mT·mm³) scaled by the same per-magnet
  strength ratio that scales the interpolated field, so the near and far models cannot
  describe magnets of different strength. Polarization runs along the magnet's own
  $-\hat{\mathbf{a}}_j$, so in world coordinates
  $\boldsymbol\mu_j = -|\boldsymbol\mu_j|\,\hat{\mathbf{a}}_{w,j} = -|\boldsymbol\mu_j|\,R\,\hat{\mathbf{a}}_j$.

**Which branch applies is fixed by geometry, not measured per evaluation.** Over the
knob's full travel the paired magnet never exceeds ~12mm from its own centre and the
cross magnets never come closer than ~25mm, so the two regimes cannot overlap and there
is nothing to test at runtime. This is why no blending between the models is needed:
the gap between them is never visited.

This branch has always been **frame-agnostic**: a magnet's orientation reaches the
formula entirely through $\boldsymbol\mu$, a single vector, so supplying
$\boldsymbol\mu_j$ and $\mathbf{r} = \mathbf{s}_i - \mathbf{c}_j$ in world coordinates
yields the world field and (§4.E) the world gradient directly. §3.2–3.5 now shares that
property: both branches reach a query point through exactly one rotation ($R$), not two
— the interpolated branch is tabulated in $(r,z)$, but $(r,z)$ themselves are obtained
by projecting against $\hat{\mathbf{a}}_j$ rather than by a second rotation into a
magnet-local frame.

---

## 4. Jacobian Derivation

**Goal:** find $J_{trans}, J_{rot}$ (each $3\times3$, per sensor) — together forming the
$3\times 6$ block $D_{\boldsymbol\rho}f_i$ of $J=D_{\boldsymbol\rho}F$ from §2 — such that
a small pose change produces a first-order change in the predicted reading:

$$\Delta\hat{\mathbf{B}}_i = J_{trans}\,\Delta\mathbf{t} + J_{rot}\,\Delta\boldsymbol{\omega}$$

The strategy is to work **outside-in**: take the variation of each layer in §3 in turn,
and substitute inward until only $\Delta\mathbf{t}$ and $\Delta\boldsymbol{\omega}$
remain.

Two skew-symmetric identities recur throughout. They follow from the convention that a
small rotation update is a **left** (world/spatial) perturbation of the current
rotation: $R_{new}=\exp([\Delta\boldsymbol{\omega}]_\times)R \approx (I+[\Delta\boldsymbol{\omega}]_\times)R$.

- **Identity 1** (cross-product anticommutativity): for any vectors $\mathbf{a},\mathbf{b}$: $[\mathbf{a}]_\times \mathbf{b} = -[\mathbf{b}]_\times \mathbf{a}$
- **Identity 2** (rotation variation): $\Delta R \approx [\Delta\boldsymbol{\omega}]_\times R$, and consequently, from $R^TR=I \Rightarrow \Delta(R^T)R + R^T\Delta R = 0$:

$$\Delta(R^T) = -R^T\,\Delta R\,R^T = -R^T[\Delta\boldsymbol{\omega}]_\times$$

### 4.A Outer layer — gain (trivial)

$G_i$ is a frozen constant, so it passes straight through:

$$\Delta\hat{\mathbf{B}}_i = G_i\, \Delta\mathbf{B}_{w,i}$$

### 4.B Middle layer — world field variation

§4.B–4.D take **one** (sensor, magnet) pair at a time, through the interpolated branch.
§4.E does the same for the far-field branch, and §4.F sums the results over $j$.

From $\mathbf{B}_{w,ij}=R\,\mathbf{B}_{ij}$, the product rule gives:

$$\Delta\mathbf{B}_{w,ij} = (\Delta R)\,\mathbf{B}_{ij} + R\,\Delta\mathbf{B}_{ij}$$

By Identity 2, $(\Delta R)\mathbf{B}_{ij} = [\Delta\boldsymbol{\omega}]_\times R\,\mathbf{B}_{ij} = [\Delta\boldsymbol{\omega}]_\times \mathbf{B}_{w,ij}$. Applying Identity 1 with $\mathbf{a}=\Delta\boldsymbol{\omega}$, $\mathbf{b}=\mathbf{B}_{w,ij}$:

$$[\Delta\boldsymbol{\omega}]_\times \mathbf{B}_{w,ij} = -[\mathbf{B}_{w,ij}]_\times \Delta\boldsymbol{\omega}$$

$$\Rightarrow \quad \Delta\mathbf{B}_{w,ij} = -[\mathbf{B}_{w,ij}]_\times \Delta\boldsymbol{\omega} + R\,\Delta\mathbf{B}_{ij}$$

(This is the same derivation the old two-rotation form used, with $R_{total,j}$ replaced
by the bare $R$ — the magnet's own tilt no longer appears here at all, having been
absorbed into $B_j$ itself in §3.2–3.4.)

### 4.C Inner layer — knob-frame vector variation

The knob-frame field's variation is driven by the knob-frame query vector, via the
analytic Jacobian of §4.D:

$$\Delta\mathbf{B}_{ij} = J_{axis}\,\Delta\mathbf{u}_{ij}$$

From $\mathbf{u}_{ij} = R^T(\mathbf{s}_i-\mathbf{t}) - \mathbf{m}_j$, and since $\mathbf{m}_j$
is entirely constant, only the first term varies:

$$\Delta\mathbf{u}_{ij} = \Delta(R^T)(\mathbf{s}_i-\mathbf{t}) + R^T(-\Delta\mathbf{t})$$

Let $\mathbf{v} = \mathbf{s}_i - \mathbf{t}$. Note this depends on the sensor only, **not**
on the magnet — all three magnets ride the same rigid knob — which is what §4.F later
exploits to pull the rotation block out of the sum over $j$.
Using Identity 2, then Identity 1 with $\mathbf{a}=\Delta\boldsymbol{\omega}$,
$\mathbf{b}=\mathbf{v}$:

$$\Delta(R^T)\,\mathbf{v} = -R^T[\Delta\boldsymbol{\omega}]_\times \mathbf{v} = R^T[\mathbf{v}]_\times \Delta\boldsymbol{\omega}$$

$$\Rightarrow \quad \Delta\mathbf{u}_{ij} = R^T[\mathbf{v}]_\times\,\Delta\boldsymbol{\omega} \;-\; R^T\,\Delta\mathbf{t}$$

### 4.D $J_{axis}$ — the axis-projection chain rule

This is the one link in the chain not handled by rigid-body identities — it's a genuine
multivariable chain rule, this time through $r$ and $z$ as obtained by projecting onto
$\hat{\mathbf{a}}_j$ (§3.3) rather than through a directional cosine in a fixed local
frame.

We need $\partial\mathbf{B}/\partial\mathbf{u}$ for
$\mathbf{B} = B_z\hat{\mathbf{a}}_j + B_r\hat{\mathbf{u}}_\perp$, with
$z=\hat{\mathbf{a}}_j\cdot\mathbf{u}$, $\mathbf{u}_\perp=\mathbf{u}-z\hat{\mathbf{a}}_j$,
$r=|\mathbf{u}_\perp|$, $\hat{\mathbf{u}}_\perp=\mathbf{u}_\perp/r$, and $B_r,B_z$
functions of $(r,z)$ only. Both $z$ and $r$ are linear-then-norm functions of
$\mathbf{u}$, so their gradients are the constant/unit vectors one would expect:

$$\frac{\partial z}{\partial \mathbf{u}} = \hat{\mathbf{a}}_j^T, \qquad \frac{\partial r}{\partial \mathbf{u}} = \hat{\mathbf{u}}_\perp^T$$

($\partial|\mathbf x|/\partial\mathbf x=\hat{\mathbf x}^T$ applied to $\mathbf u_\perp$;
its own dependence on $\mathbf u$ goes through the constant projector
$P_\perp = I-\hat{\mathbf a}_j\hat{\mathbf a}_j^T$, but $\hat{\mathbf u}_\perp^T P_\perp=\hat{\mathbf u}_\perp^T$
already, so $P_\perp$ drops out of this particular gradient.)

$\hat{\mathbf{a}}_j$ is constant, so the $B_z\hat{\mathbf{a}}_j$ term is a plain chain
rule through $r,z$:

$$\frac{\partial(B_z\hat{\mathbf{a}}_j)}{\partial\mathbf{u}} = \hat{\mathbf{a}}_j\left(\frac{\partial B_z}{\partial r}\hat{\mathbf{u}}_\perp^T + \frac{\partial B_z}{\partial z}\hat{\mathbf{a}}_j^T\right)$$

$B_r\hat{\mathbf{u}}_\perp$ needs the product rule too, because $\hat{\mathbf u}_\perp$
itself varies with $\mathbf u$ — the one genuinely new piece, the quotient-rule
derivative of $\mathbf u_\perp/r$:

$$\frac{\partial\hat{\mathbf{u}}_\perp}{\partial\mathbf{u}} = \frac{P_\perp - \hat{\mathbf{u}}_\perp\hat{\mathbf{u}}_\perp^T}{r}$$

(divide the numerator's $P_\perp$ by $r$, then subtract $\mathbf u_\perp(\partial r/\partial\mathbf u)/r^2=\hat{\mathbf u}_\perp\hat{\mathbf u}_\perp^T/r$
to correct for the denominator's own derivative — the same two-step quotient rule the
old local-frame derivation used for $\partial(x_l/r)/\partial x_l$, just in vector form
instead of per-component.) So:

$$\frac{\partial(B_r\hat{\mathbf{u}}_\perp)}{\partial\mathbf{u}} = \hat{\mathbf{u}}_\perp\left(\frac{\partial B_r}{\partial r}\hat{\mathbf{u}}_\perp^T + \frac{\partial B_r}{\partial z}\hat{\mathbf{a}}_j^T\right) + \frac{B_r}{r}\Big(P_\perp - \hat{\mathbf{u}}_\perp\hat{\mathbf{u}}_\perp^T\Big)$$

Assembled:

$$J_{axis} = \frac{\partial B_z}{\partial r}\,\hat{\mathbf{a}}_j\hat{\mathbf{u}}_\perp^T + \frac{\partial B_z}{\partial z}\,\hat{\mathbf{a}}_j\hat{\mathbf{a}}_j^T + \frac{\partial B_r}{\partial r}\,\hat{\mathbf{u}}_\perp\hat{\mathbf{u}}_\perp^T + \frac{\partial B_r}{\partial z}\,\hat{\mathbf{u}}_\perp\hat{\mathbf{a}}_j^T + \frac{B_r}{r}\Big(P_\perp - \hat{\mathbf{u}}_\perp\hat{\mathbf{u}}_\perp^T\Big)$$

Note there is no arbitrary in-plane basis anywhere in this expression — only
$\hat{\mathbf a}_j$ and $\hat{\mathbf u}_\perp$, both determined by the query point
itself. That is the concrete payoff of §3.4's observation that no azimuthal reference
is needed: the old $J_{local}$ needed an entire local *frame* (rotation matrix) to
project into; this needs only the two vectors already in hand.

**Singularity at $r=0$:** exactly the old limit, transplanted to this basis —
$B_r/r\to\partial B_r/\partial r$ as $r\to0$ (L'Hôpital, since $B_r(0,z)=0$ by the
magnet's axial symmetry), which collapses the transverse terms to an isotropic
$\partial B_r/\partial r\cdot P_\perp$:

$$J_{axis}\big|_{r=0} = \frac{\partial B_r}{\partial r}\big(I-\hat{\mathbf{a}}_j\hat{\mathbf{a}}_j^T\big) + \frac{\partial B_z}{\partial z}\,\hat{\mathbf{a}}_j\hat{\mathbf{a}}_j^T$$

which is the old $\operatorname{diag}(\partial B_r/\partial r,\ \partial B_r/\partial r,\ \partial B_z/\partial z)$
written basis-free — that diagonal form was this same expression in the special case
$\hat{\mathbf{a}}_j=\hat{\mathbf{z}}$.

### 4.E $J_{dipole}$ — the far-field Jacobian

The §3.6 branch needs its own analytic derivative. Writing $\rho=|\mathbf{r}|$,
$k=1/4\pi$, and expanding $\hat{\mathbf{r}}=\mathbf{r}/\rho$ so the powers are explicit:

$$\mathbf{B} = k\Big[3(\boldsymbol\mu\cdot\mathbf{r})\,\mathbf{r}\,\rho^{-5} - \boldsymbol\mu\,\rho^{-3}\Big]$$

Differentiating component-wise, using $\partial(\boldsymbol\mu\cdot\mathbf{r})/\partial r_b = \mu_b$,
$\partial r_a/\partial r_b=\delta_{ab}$, and $\partial\rho^{-n}/\partial r_b = -n\,r_b\,\rho^{-n-2}$:

$$\frac{\partial B_a}{\partial r_b} = k\Big[3\rho^{-5}\big(\mu_a r_b + r_a \mu_b + (\boldsymbol\mu\cdot\mathbf{r})\delta_{ab}\big) - 15(\boldsymbol\mu\cdot\mathbf{r})\,\rho^{-7} r_a r_b\Big]$$

or in matrix form:

$$J_{dipole} = k\Big[3\rho^{-5}\big(\boldsymbol\mu\mathbf{r}^T + \mathbf{r}\boldsymbol\mu^T + (\boldsymbol\mu\cdot\mathbf{r})I\big) - 15(\boldsymbol\mu\cdot\mathbf{r})\rho^{-7}\,\mathbf{r}\mathbf{r}^T\Big]$$

**$J_{dipole}$ is symmetric**, visibly so: every term above is invariant under
$a\leftrightarrow b$. That is not a coincidence of this algebra but a physical
requirement — away from its source the field is curl-free, so it is the gradient of a
scalar potential and its Jacobian is (minus) that potential's Hessian. The
implementation computes six entries and mirrors three on exactly this basis, and
`test_dipole_field_jacobian` asserts the symmetry so that an edit breaking it fails
loudly rather than producing a plausible-looking solver.

Because §3.6 is evaluated in the world frame, $J_{dipole}$ **is** that magnet's
contribution to $M$ below — there is no $R\,J\,R^T$ congruence to apply.

### 4.F Assembly — folding it all back together

Substitute §4.C's $\Delta\mathbf{u}_{ij}$ into $\Delta\mathbf{B}_{ij}=J_{axis}\Delta\mathbf{u}_{ij}$, then into §4.B, for one magnet $j$:

$$\Delta\mathbf{B}_{w,ij} = -[\mathbf{B}_{w,ij}]_\times\Delta\boldsymbol{\omega} + R\,J_{axis}\Big(R^T[\mathbf{v}]_\times\Delta\boldsymbol{\omega} - R^T\Delta\mathbf{t}\Big)$$

Define that magnet's field gradient in the world frame:

$$M_{ij} = R\,J_{axis}\,R^T \quad\text{(interpolated branch)}, \qquad M_{ij} = J_{dipole} \quad\text{(§3.6 branch, already in the world frame)}$$

$M_{ij}$ is nothing but $\partial\mathbf{B}_{w,ij}/\partial\mathbf{s}_i$ — the world-frame
field gradient with respect to the query point, independent of any particular way of
computing it. The firmware does not actually form $R\,J_{axis}\,R^T$: it computes the
algebraically identical closed form directly in world coordinates, using
$\hat{\mathbf{a}}_{w,j}=R\hat{\mathbf{a}}_j$ (already needed for §3.6) and the world-frame
radial unit vector, which is exactly $J_{axis}$ (§4.D) with every $\hat{\mathbf a}_j$ and
$\hat{\mathbf u}_\perp$ replaced by its world-frame counterpart. Both routes give the same
$M_{ij}$; see `MagnetPlacement::near_approx_world` in `magnet_local_model.cpp`.

Distributing and grouping by $\Delta\mathbf{t}$ / $\Delta\boldsymbol{\omega}$, then summing
over magnets — note $\mathbf{v} = \mathbf{s}_i-\mathbf{t}$ carries no $j$, since all three
magnets ride the same rigid knob:

$$\Delta\mathbf{B}_{w,i} = \sum_j \Delta\mathbf{B}_{w,ij} = \underbrace{\Big(-\sum_j M_{ij}\Big)}_{\text{translation}}\Delta\mathbf{t} + \underbrace{\Big(\Big(\sum_j M_{ij}\Big)[\mathbf{v}]_\times - \Big[\sum_j \mathbf{B}_{w,ij}\Big]_\times\Big)}_{\text{rotation}}\Delta\boldsymbol{\omega}$$

**Summing before assembling is exact, not an approximation**, and it is the step that
makes cross-magnet modelling affordable. Both blocks are linear in $M_{ij}$ and
$\mathbf{B}_{w,ij}$, and $[\,\cdot\,]_\times$ is linear in its argument, so
$\sum_j$ commutes with everything outside it. The consequence is that the skew products
and block assembly run **once per sensor** rather than once per (sensor, magnet) pair —
three times per evaluation instead of nine.

Folding in the gain from §4.A gives the final $3\times3$ blocks, with
$M_i = \sum_j M_{ij}$ and $\mathbf{B}_{w,i} = \sum_j \mathbf{B}_{w,ij}$:

$$J_{trans} = G_i(-M_i), \qquad J_{rot} = G_i\big(M_i[\mathbf{v}]_\times - [\mathbf{B}_{w,i}]_\times\big)$$

**Crucial detail:** $[\mathbf{B}_{w,i}]_\times$ must be built from the pre-gain, physical
field $\mathbf{B}_{w,i}$, *before* $G_i$ is applied — it appears inside §4.B, ahead of
§4.A's gain step, so using the gained field here would be a subtly wrong Jacobian even
though the residual itself is computed post-gain. It must also be the **summed** field:
using the paired magnet's contribution alone would drop the cross terms from the
rotation block while keeping them in the prediction.

### 4.G Stacking to 9×6

Each sensor's $3\times6$ block $D_{\boldsymbol\rho}f_i = [\,J_{trans,i}\;\;J_{rot,i}\,]$
— now summed over all three magnets, per §4.F — occupies its own three rows of the
shared $9\times6$ matrix $J=D_{\boldsymbol\rho}F$ from §2. All three sensors share the
same $\mathbf{t}, R$ (and hence the same $\Delta\mathbf{t}, \Delta\boldsymbol{\omega}$
columns), since they observe one rigid knob:

$$J = \begin{bmatrix} J_{trans,1} & J_{rot,1} \\ J_{trans,2} & J_{rot,2} \\ J_{trans,3} & J_{rot,3} \end{bmatrix} \in \mathbb{R}^{9\times6}$$

---

## 5. The Solve Loop

Each iteration of the pose solver performs one Levenberg-Marquardt step:

1. **Residual:** $\mathbf{r} = F(\boldsymbol\rho) - \mathbf{B}_{measured} \in\mathbb{R}^9$, evaluated at the current pose estimate ($\boldsymbol\rho=\mathbf 0$ in the local chart of §2).
2. **Damped normal equations:** $H = J^TJ + \lambda I$, $\ \mathbf{g}=-J^T\mathbf{r}$ — the
   $\lambda I$ term is what makes this Levenberg-Marquardt rather than plain Gauss-Newton,
   keeping $H$ invertible even when $J$ is rank-deficient (e.g. near $r=0$ or degenerate
   poses).
3. **Solve** the $6\times6$ system $H\,\Delta\boldsymbol{\rho} = \mathbf{g}$, giving
   $\Delta\boldsymbol{\rho}=(\Delta\mathbf{t},\Delta\boldsymbol{\omega})$.
4. **Convergence check** on $\|\Delta\boldsymbol{\rho}\|^2$ against a small tolerance.
5. **Apply the update**, which re-centers the local chart at the new estimate:
   $$\mathbf{t} \mathrel{+}= \Delta\mathbf{t}, \qquad R \leftarrow \exp([\Delta\boldsymbol{\omega}]_\times)\,R$$
   The derivation in §4 used the *first-order* approximation
   $\Delta R\approx[\Delta\boldsymbol{\omega}]_\times R$ (Identity 2), valid for
   infinitesimal steps; the actual update instead applies the *exact* matrix exponential
   so that a finite-sized step still leaves $R$ exactly orthogonal, with no
   re-orthonormalization needed. The next iteration then repeats §2's linearization
   around this new $(\mathbf t, R)$.

---

## 6. Validation

Every analytic piece above — the interpolation table's partials, $J_{axis}$ (including
the $r=0$ branch), $J_{dipole}$, and the full $9\times6$ forward-model Jacobian — can be,
and should continue to be, checked against central finite differences swept over a grid
of poses, sensor gains, and magnet tilts. Perturbing $R$ with the exact matrix exponential
$\exp([\mathbf{w}]_\times)$ (not the first-order $I+[\mathbf{w}]_\times$) for the
rotation columns is important here: using the same first-order approximation on both
sides would let a shared error hide from the comparison.

Two things finite differences **cannot** catch here, both because they compare the model
against its own derivative and so are blind to the model being consistently wrong:

- **A wrong dipole moment or a misplaced dipole.** Either produces a smooth, entirely
  self-consistent field of the wrong magnitude. What catches it is comparing the two
  branches against each other where both are valid — the §3.6 dipole against the §3.3–3.4
  table at the far edge of the grid's domain, since they are supposed to describe the
  same physical magnet.
- **The cross-magnet terms being absent altogether.** A model with $\sum_j$ silently
  reduced to the paired magnet is a perfectly differentiable model. What catches it is
  asserting the cross contribution is present and of the measured size (1.7–4.5% of the
  field).

`firmware/test/test_jacobian.cpp` carries all of the above.
