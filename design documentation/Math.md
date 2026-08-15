# Forward Model and Jacobian — the Mathematics Behind the Pose Solver

This document derives the mathematics of the 6DOF magnetic pose solver: the forward
model that predicts sensor readings from a candidate pose, and the exact analytic
Jacobian of that model, which drives a Levenberg-Marquardt (damped Gauss-Newton)
optimization loop.

The structure below follows the natural computational order — global pose, to local
geometry, to field lookup, to sensor output — so that each section corresponds to one
conceptual stage of the pipeline.

---

## 1. Notation

Indices: $i$ always denotes a **sensor**, $j$ always a **magnet**. Both run over
$\{1,2,3\}$. Every sensor sees every magnet, so most quantities below carry both.

| Symbol | Meaning |
|---|---|
| $\mathbf{s}_i$ | sensor $i$ position, global/PCB frame (constant) |
| $\mathbf{m}_j$ | magnet $j$ resting position (bottom face), knob frame (constant) |
| $R_{m,j}$ | magnet $j$'s tilt inside the knob (constant) |
| $G_i$ | sensor $i$'s gain/distortion matrix (constant) |
| $\mathbf{t}$ | knob translation, global frame (**solved for**) |
| $R$ | knob rotation matrix (**solved for**) |
| $\boldsymbol{\rho}=(\mathbf t,\boldsymbol\omega)$ | 6DOF pose coordinates: translation + rotation-update vector |
| $R(\boldsymbol\omega)=\exp([\boldsymbol\omega]_\times)$ | rotation as a function of $\boldsymbol\omega\in\mathfrak{so}(3)$ (Rodrigues) |
| $F,\,f_i$ | the forward model, overall and per-sensor (§2) |
| $B_j$ | the local magnet-field function for magnet $j$ (§3.3–3.4) |
| $R_{total,j} = R\,R_{m,j}$ | combined knob + magnet rotation |
| $\mathbf{v}_{l,ij}$ | sensor $i$'s position relative to magnet $j$, in magnet $j$'s local frame |
| $\mathbf{B}_{l,ij}$ | field at sensor $i$ from magnet $j$, in magnet $j$'s local frame |
| $\mathbf{B}_{g,ij}$ | field at sensor $i$ from magnet $j$, global frame (pre-gain) |
| $\mathbf{B}_{g,i}=\sum_j \mathbf{B}_{g,ij}$ | total field at sensor $i$, global frame (pre-gain) |
| $\hat{\mathbf{B}}_i$ | predicted sensor reading |
| $h$ | magnet half-height; the dipole sits this far above $\mathbf{m}_j$ along the magnet's axis (§3.6) |
| $\boldsymbol{\mu}_j$ | magnet $j$'s dipole moment, global frame (§3.6) |
| $\mathbf{c}_j$ | magnet $j$'s geometric centre, global frame (§3.6) |

Only $\mathbf{t}$ and $R$ are optimization variables. Everything else — $\mathbf{s}_i$,
$\mathbf{m}_j$, $R_{m,j}$, $G_i$ — is a frozen calibration constant, and so contributes
**zero** to every derivative below. That fact is used repeatedly to prune terms.

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

$$f_i(\boldsymbol\rho) = G_i \sum_{j} R(\boldsymbol\omega)R_{m,j}\; B_j\!\Big(R_{m,j}^T\big(R(\boldsymbol\omega)^T(\mathbf{s}_i-\mathbf{t})-\mathbf{m}_j\big)\Big)$$

where $B_j:\mathbb{R}^3\to\mathbb{R}^3$ — defined in §3.3–3.4 — maps a local query point
to the field magnet $j$ produces there.

**Where the sum sits matters.** Superposition is a property of the *field*, so the sum
has to be taken there, at $\mathbf{B}_g$, and not somewhere more convenient further out.
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
once rotating into the local frame ($R^T$, inside $B_j$'s argument) and once rotating
back out to global ($R$, outside). This is the reason the Jacobian derivation in §4 needs
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

### 3.1 Combined rotation

The knob's rotation $R$ (the solved-for state) and the magnet's fixed tilt $R_{m,j}$
compose into a single rotation from magnet-local space to global space:

$$R_{total,j} = R \, R_{m,j}$$

### 3.2 Local vector geometry

The vector from a magnet to its sensor is built up frame by frame, not in one jump: first
the sensor position is expressed in the knob's frame, then the magnet's own resting
offset is subtracted *in that frame*, and only then is the result rotated into the
magnet's own (possibly tilted) local frame:

$$\mathbf{v}_{l,ij} = R_{m,j}^T\Big(R^T(\mathbf{s}_i - \mathbf{t}) - \mathbf{m}_j\Big) = R_{total,j}^T(\mathbf{s}_i - \mathbf{t}) \;-\; R_{m,j}^T \mathbf{m}_j$$

The order matters for getting the right physical answer: $\mathbf{m}_j$ is subtracted
before the magnet-tilt rotation is applied, so the correct constant offset term is
$R_{m,j}^T\mathbf{m}_j$, not a bare $\mathbf{m}_j$.

### 3.3 Cylindrical query and interpolation

The magnets are axially polarized, so the field they produce is symmetric around their
own local axis and depends only on radius and height relative to that axis:

$$r = \sqrt{x_l^2+y_l^2}, \qquad z = z_l, \qquad \text{where } \mathbf{v}_{l,ij}=(x_l,y_l,z_l)$$

A precomputed, $C^1$-continuous bicubic interpolation table is queried at $(r,z)$,
returning the two cylindrical field components $B_r, B_z$ and their four spatial
partials $\partial B_r/\partial r$, $\partial B_z/\partial r$, $\partial B_r/\partial z$,
$\partial B_z/\partial z$.

### 3.4 Local field reconstruction

The scalar cylindrical field is rotated back into the 3D local Cartesian frame using
directional cosines $c_x = x_l/r$, $c_y = y_l/r$. Together, §3.3–3.4 constitute the
function $B_j$ from §2:

$$\mathbf{B}_{l,ij} = B_j(\mathbf{v}_{l,ij}) = \begin{bmatrix} B_r c_x \\ B_r c_y \\ B_z \end{bmatrix}$$

### 3.5 Back to global frame, then gain

The local field is rotated into the global frame, summed over magnets, then passed
through the sensor's own gain/distortion matrix to give the predicted sensor reading:

$$\mathbf{B}_{g,ij} = R_{total,j}\, \mathbf{B}_{l,ij}, \qquad \mathbf{B}_{g,i} = \sum_j \mathbf{B}_{g,ij}, \qquad \hat{\mathbf{B}}_i = f_i(\boldsymbol\rho) = G_i\, \mathbf{B}_{g,i}$$

(An additive sensor baseline/offset, if calibrated, is handled upstream as a
correction to the raw measurement rather than as a term in this model.)

### 3.6 The far-field branch — magnets a sensor does not sit under

§3.3–3.4 is the model for the magnet a sensor is paired with. The other two sit a
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

- **The dipole sits at the magnet's geometric centre**, not at $\mathbf{m}_j$. The local
  frame's origin is the magnet's *bottom face* (§3.2), so the centre is half a magnet
  higher **along the magnet's own axis**: $\mathbf{c}_j = \mathbf{t} + R\,(\mathbf{m}_j + R_{m,j}\,h\hat{\mathbf{z}})$.
  Placing it at the bottom face instead is a ~50% error at cross-magnet range, not a
  refinement.
- **The moment is the same magnet the table describes.** $|\boldsymbol\mu_j|$ is the
  reference moment (polarization × volume, in mT·mm³) scaled by the same per-magnet
  strength ratio that scales the interpolated field, so the near and far models cannot
  describe magnets of different strength. Polarization runs along the magnet's local
  $-\hat{\mathbf{z}}$, so in global coordinates $\boldsymbol\mu_j = -|\boldsymbol\mu_j|\,R_{total,j}\,\hat{\mathbf{z}}$
  — the third column of $R_{total,j}$, scaled.

**Which branch applies is fixed by geometry, not measured per evaluation.** Over the
knob's full travel the paired magnet never exceeds ~12mm from its own centre and the
cross magnets never come closer than ~25mm, so the two regimes cannot overlap and there
is nothing to test at runtime. This is why no blending between the models is needed:
the gap between them is never visited.

Unlike §3.3–3.4 this branch is **frame-agnostic**. A magnet's orientation reaches the
formula entirely through $\boldsymbol\mu$, a single vector, so supplying $\boldsymbol\mu_j$
and $\mathbf{r} = \mathbf{s}_i - \mathbf{c}_j$ in global coordinates yields the global
field and (§4.E) the global gradient directly — no rotation into the magnet's frame and
no rotation back. The interpolated branch has no such freedom: it is tabulated in
$(r,z)$ and must be handed magnet-local coordinates.

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
small rotation update is a **left** (global/spatial) perturbation of the current
rotation: $R_{new}=\exp([\Delta\boldsymbol{\omega}]_\times)R \approx (I+[\Delta\boldsymbol{\omega}]_\times)R$.

- **Identity 1** (cross-product anticommutativity): for any vectors $\mathbf{a},\mathbf{b}$: $[\mathbf{a}]_\times \mathbf{b} = -[\mathbf{b}]_\times \mathbf{a}$
- **Identity 2** (rotation variation): $\Delta R \approx [\Delta\boldsymbol{\omega}]_\times R$, and consequently, from $R^TR=I \Rightarrow \Delta(R^T)R + R^T\Delta R = 0$:

$$\Delta(R^T) = -R^T\,\Delta R\,R^T = -R^T[\Delta\boldsymbol{\omega}]_\times$$

### 4.A Outer layer — gain (trivial)

$G_i$ is a frozen constant, so it passes straight through:

$$\Delta\hat{\mathbf{B}}_i = G_i\, \Delta\mathbf{B}_{g,i}$$

### 4.B Middle layer — global field variation

§4.B–4.D take **one** (sensor, magnet) pair at a time, through the interpolated branch.
§4.E does the same for the far-field branch, and §4.F sums the results over $j$.

From $\mathbf{B}_{g,ij}=R_{total,j}\mathbf{B}_{l,ij}$, the product rule gives:

$$\Delta\mathbf{B}_{g,ij} = (\Delta R_{total,j})\,\mathbf{B}_{l,ij} + R_{total,j}\,\Delta\mathbf{B}_{l,ij}$$

$R_{m,j}$ is constant, so $\Delta R_{total,j} = (\Delta R)R_{m,j} = [\Delta\boldsymbol{\omega}]_\times R\, R_{m,j} = [\Delta\boldsymbol{\omega}]_\times R_{total,j}$ (Identity 2). Substituting, then applying Identity 1 with $\mathbf{a}=\Delta\boldsymbol{\omega}$, $\mathbf{b}=\mathbf{B}_{g,ij}=R_{total,j}\mathbf{B}_{l,ij}$:

$$[\Delta\boldsymbol{\omega}]_\times \mathbf{B}_{g,ij} = -[\mathbf{B}_{g,ij}]_\times \Delta\boldsymbol{\omega}$$

$$\Rightarrow \quad \Delta\mathbf{B}_{g,ij} = -[\mathbf{B}_{g,ij}]_\times \Delta\boldsymbol{\omega} + R_{total,j}\,\Delta\mathbf{B}_{l,ij}$$

### 4.C Inner layer — local vector variation

The local field's variation is driven by the local query vector, via the analytic
Jacobian of §4.D:

$$\Delta\mathbf{B}_{l,ij} = J_{local}\,\Delta\mathbf{v}_{l,ij}$$

From $\mathbf{v}_{l,ij} = R_{total,j}^T(\mathbf{s}_i-\mathbf{t}) - R_{m,j}^T\mathbf{m}_j$, and since the
$R_{m,j}^T\mathbf{m}_j$ term is entirely constant, only the first term varies:

$$\Delta\mathbf{v}_{l,ij} = \Delta(R_{total,j}^T)(\mathbf{s}_i-\mathbf{t}) + R_{total,j}^T(-\Delta\mathbf{t})$$

Let $\mathbf{v} = \mathbf{s}_i - \mathbf{t}$. Note this depends on the sensor only, **not**
on the magnet — all three magnets ride the same rigid knob — which is what §4.F later
exploits to pull the rotation block out of the sum over $j$.
Using Identity 2, then Identity 1 with $\mathbf{a}=\Delta\boldsymbol{\omega}$,
$\mathbf{b}=\mathbf{v}$:

$$\Delta(R_{total,j}^T)\,\mathbf{v} = -R_{total,j}^T[\Delta\boldsymbol{\omega}]_\times \mathbf{v} = R_{total,j}^T[\mathbf{v}]_\times \Delta\boldsymbol{\omega}$$

$$\Rightarrow \quad \Delta\mathbf{v}_{l,ij} = R_{total,j}^T[\mathbf{v}]_\times\,\Delta\boldsymbol{\omega} \;-\; R_{total,j}^T\,\Delta\mathbf{t}$$

*(Remark: had we instead used the simpler-looking but incomplete $\mathbf{v}_{l,ij}=R_{total,j}^T(\mathbf{s}_i-\mathbf{t})-\mathbf{m}_j$ from §3.2, the derivative would come out identical — differentiating any constant term, whatever its exact form, always contributes zero. The correction in §3.2 matters for the field value, not for the Jacobian.)*

### 4.D $J_{local}$ — the cylindrical-to-Cartesian chain rule

This is the one link in the chain not handled by rigid-body identities — it's a genuine
multivariable chain rule through $r=\sqrt{x_l^2+y_l^2}$.

We need $\partial(\mathbf{B}_l)_a/\partial x_j$ for $a,j \in \{x_l,y_l,z_l\}$, with
$\mathbf{B}_l = (B_r c_x,\; B_r c_y,\; B_z)$, $c_x=x_l/r$, $c_y=y_l/r$, and $B_r,B_z$
functions of $(r,z_l)$ only. First, the geometric partials of $r$ itself:

$$\frac{\partial r}{\partial x_l} = \frac{x_l}{r} = c_x, \qquad \frac{\partial r}{\partial y_l}=c_y, \qquad \frac{\partial r}{\partial z_l}=0$$

**Row 1** ($B_x = B_r(r,z_l)\cdot x_l/r$), by the product rule:

$$\frac{\partial B_x}{\partial x_l} = \underbrace{\frac{\partial B_r}{\partial r}c_x}_{\text{chain rule thru } r}\cdot c_x \;+\; B_r\,\frac{\partial}{\partial x_l}\!\left(\frac{x_l}{r}\right)$$

The second term needs the quotient rule: $\partial(x_l/r)/\partial x_l = (r - x_l c_x)/r^2$.
Using $x_l c_x = x_l^2/r$ and $r^2=x_l^2+y_l^2$:

$$r - x_l c_x = r - \frac{x_l^2}{r} = \frac{r^2-x_l^2}{r} = \frac{y_l^2}{r} \quad\Rightarrow\quad \frac{\partial}{\partial x_l}\!\left(\frac{x_l}{r}\right) = \frac{y_l^2}{r^3} = \frac{c_y^2}{r}$$

So:

$$\frac{\partial B_x}{\partial x_l} = \frac{\partial B_r}{\partial r}c_x^2 + \frac{B_r}{r}c_y^2$$

The off-diagonal term follows the same pattern (product rule + quotient rule, this time
$\partial(x_l/r)/\partial y_l = -x_l y_l/r^3 = -c_xc_y/r$):

$$\frac{\partial B_x}{\partial y_l} = \frac{\partial B_r}{\partial r}c_xc_y - \frac{B_r}{r}c_xc_y = \left(\frac{\partial B_r}{\partial r}-\frac{B_r}{r}\right)c_xc_y$$

and, since $c_x$ has no $z_l$-dependence:

$$\frac{\partial B_x}{\partial z_l} = \frac{\partial B_r}{\partial z}c_x$$

**Row 2** ($B_y=B_r c_y$) is identical by the $x_l\leftrightarrow y_l$ symmetry of $r$, giving
the mirrored entries (note $\partial B_x/\partial y_l = \partial B_y/\partial x_l$ — the
top-left $2\times2$ block is symmetric).

**Row 3** ($B_z(r,z_l)$, no directional cosine to differentiate) is a plain chain rule
through $r$:

$$\frac{\partial B_z}{\partial x_l} = \frac{\partial B_z}{\partial r}c_x, \qquad \frac{\partial B_z}{\partial y_l} = \frac{\partial B_z}{\partial r}c_y, \qquad \frac{\partial B_z}{\partial z_l} = \frac{\partial B_z}{\partial z}$$

Assembled:

$$J_{local} = \begin{bmatrix}
\dfrac{\partial B_r}{\partial r}c_x^2+\dfrac{B_r}{r}c_y^2 & \left(\dfrac{\partial B_r}{\partial r}-\dfrac{B_r}{r}\right)c_xc_y & \dfrac{\partial B_r}{\partial z}c_x \\[1.2em]
\left(\dfrac{\partial B_r}{\partial r}-\dfrac{B_r}{r}\right)c_xc_y & \dfrac{\partial B_r}{\partial r}c_y^2+\dfrac{B_r}{r}c_x^2 & \dfrac{\partial B_r}{\partial z}c_y \\[1.2em]
\dfrac{\partial B_z}{\partial r}c_x & \dfrac{\partial B_z}{\partial r}c_y & \dfrac{\partial B_z}{\partial z}
\end{bmatrix}$$

**Singularity at $r=0$:** every off-diagonal term above has an explicit $1/r$
(inside $B_r/r$ or the $c_xc_y$ products), so naive evaluation blows up on-axis. But
physically the field must stay smooth there, and $B_r/r \to \partial B_r/\partial r$ as
$r\to0$ (L'Hôpital, since $B_r(0,z)=0$ by the magnet's axial symmetry — no radial field
component on the axis itself). Substituting this limit collapses the whole matrix: the
$c_xc_y$ off-diagonal terms vanish (finite $\times$ $r\to0$ factor), and the top-left
$2\times2$ block becomes an isotropic $\partial B_r/\partial r \cdot I$:

$$J_{local}\big|_{r=0} = \operatorname{diag}\!\left(\frac{\partial B_r}{\partial r},\ \frac{\partial B_r}{\partial r},\ \frac{\partial B_z}{\partial z}\right)$$

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

Because §3.6 is evaluated in global coordinates, $J_{dipole}$ **is** that magnet's
contribution to $M$ below — there is no $R\,J\,R^T$ congruence to apply.

### 4.F Assembly — folding it all back together

Substitute §4.C's $\Delta\mathbf{v}_{l,ij}$ into $\Delta\mathbf{B}_{l,ij}=J_{local}\Delta\mathbf{v}_{l,ij}$, then into §4.B, for one magnet $j$:

$$\Delta\mathbf{B}_{g,ij} = -[\mathbf{B}_{g,ij}]_\times\Delta\boldsymbol{\omega} + R_{total,j}J_{local}\Big(R_{total,j}^T[\mathbf{v}]_\times\Delta\boldsymbol{\omega} - R_{total,j}^T\Delta\mathbf{t}\Big)$$

Define that magnet's field gradient in global coordinates:

$$M_{ij} = R_{total,j}\,J_{local}\,R_{total,j}^T \quad\text{(interpolated branch)}, \qquad M_{ij} = J_{dipole} \quad\text{(§3.6 branch, already global)}$$

Distributing and grouping by $\Delta\mathbf{t}$ / $\Delta\boldsymbol{\omega}$, then summing
over magnets — note $\mathbf{v} = \mathbf{s}_i-\mathbf{t}$ carries no $j$, since all three
magnets ride the same rigid knob:

$$\Delta\mathbf{B}_{g,i} = \sum_j \Delta\mathbf{B}_{g,ij} = \underbrace{\Big(-\sum_j M_{ij}\Big)}_{\text{translation}}\Delta\mathbf{t} + \underbrace{\Big(\Big(\sum_j M_{ij}\Big)[\mathbf{v}]_\times - \Big[\sum_j \mathbf{B}_{g,ij}\Big]_\times\Big)}_{\text{rotation}}\Delta\boldsymbol{\omega}$$

**Summing before assembling is exact, not an approximation**, and it is the step that
makes cross-magnet modelling affordable. Both blocks are linear in $M_{ij}$ and
$\mathbf{B}_{g,ij}$, and $[\,\cdot\,]_\times$ is linear in its argument, so
$\sum_j$ commutes with everything outside it. The consequence is that the skew products
and block assembly run **once per sensor** rather than once per (sensor, magnet) pair —
three times per evaluation instead of nine.

Folding in the gain from §4.A gives the final $3\times3$ blocks, with
$M_i = \sum_j M_{ij}$ and $\mathbf{B}_{g,i} = \sum_j \mathbf{B}_{g,ij}$:

$$J_{trans} = G_i(-M_i), \qquad J_{rot} = G_i\big(M_i[\mathbf{v}]_\times - [\mathbf{B}_{g,i}]_\times\big)$$

**Crucial detail:** $[\mathbf{B}_{g,i}]_\times$ must be built from the pre-gain, physical
field $\mathbf{B}_{g,i}$, *before* $G_i$ is applied — it appears inside §4.B, ahead of
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

Every analytic piece above — the interpolation table's partials, $J_{local}$ (including
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
