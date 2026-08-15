# Forward Model and Jacobian — the Mathematics Behind the Pose Solver

This document derives the mathematics of the 6DOF magnetic pose solver: the forward
model that predicts sensor readings from a candidate pose, and the exact analytic
Jacobian of that model, which drives a Levenberg-Marquardt (damped Gauss-Newton)
optimization loop.

The structure below follows the natural computational order — global pose, to local
geometry, to field lookup, to sensor output — so that each section corresponds to one
conceptual stage of the pipeline. §1–6 cover the single-frame pose solve. §7 extends
this to bundle calibration — fitting the constants §1 holds frozen, jointly across many
frames — and derives the block structure of that larger problem's normal equations.

---

## 1. Notation

| Symbol | Meaning |
|---|---|
| $\mathbf{s}_i$ | sensor $i$ position, global/PCB frame (constant) |
| $\mathbf{m}_i$ | magnet $i$ resting position, knob frame (constant) |
| $R_{m,i}$ | magnet $i$'s tilt inside the knob (constant) |
| $G_i$ | sensor $i$'s gain/distortion matrix (constant) |
| $\mathbf{t}$ | knob translation, global frame (**solved for**) |
| $R$ | knob rotation matrix (**solved for**) |
| $\boldsymbol{\rho}=(\mathbf t,\boldsymbol\omega)$ | 6DOF pose coordinates: translation + rotation-update vector |
| $R(\boldsymbol\omega)=\exp([\boldsymbol\omega]_\times)$ | rotation as a function of $\boldsymbol\omega\in\mathfrak{so}(3)$ (Rodrigues) |
| $F,\,f_i$ | the forward model, overall and per-sensor (§2) |
| $B_i$ | the local magnet-field function for magnet $i$ (§3.3–3.4) |
| $R_{total} = R\,R_{m,i}$ | combined knob + magnet rotation |
| $\mathbf{v}_{l,i}$ | sensor position relative to magnet $i$, in magnet-local frame |
| $\mathbf{B}_{l,i}$ | field at the sensor, in magnet-local frame |
| $\mathbf{B}_{g,i}$ | field at the sensor, in global frame (pre-gain) |
| $\hat{\mathbf{B}}_i$ | predicted sensor reading |

Only $\mathbf{t}$ and $R$ are optimization variables. Everything else — $\mathbf{s}_i$,
$\mathbf{m}_i$, $R_{m,i}$, $G_i$ — is a frozen calibration constant, and so contributes
**zero** to every derivative below. That fact is used repeatedly to prune terms.

---

## 2. Functional Definition

The forward model is a single map from a 6-parameter pose to 9 predicted sensor
readings (three 3-axis sensors):

$$F:\mathbb{R}^6\to\mathbb{R}^9, \qquad F(\boldsymbol\rho) = \begin{bmatrix} f_1(\boldsymbol\rho)\\ f_2(\boldsymbol\rho)\\ f_3(\boldsymbol\rho)\end{bmatrix}$$

Its input $\boldsymbol\rho=(\mathbf t,\boldsymbol\omega)\in\mathbb{R}^3\times\mathbb{R}^3$
pairs the translation with a rotation-update vector, related to an actual rotation
matrix through the exponential map $R(\boldsymbol\omega)=\exp([\boldsymbol\omega]_\times)$.
Each $f_i:\mathbb{R}^6\to\mathbb{R}^3$ is the prediction for sensor $i$ alone, given in
closed form by composing every stage in §3:

$$f_i(\boldsymbol\rho) = G_i\; R(\boldsymbol\omega)R_{m,i}\; B_i\!\Big(R_{m,i}^T\big(R(\boldsymbol\omega)^T(\mathbf{s}_i-\mathbf{t})-\mathbf{m}_i\big)\Big)$$

where $B_i:\mathbb{R}^3\to\mathbb{R}^3$ — defined in §3.3–3.4 — maps a local query point
to the field the magnet produces there. Note that $\boldsymbol\omega$ appears **twice**:
once rotating into the local frame ($R^T$, inside $B_i$'s argument) and once rotating
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

The knob's rotation $R$ (the solved-for state) and the magnet's fixed tilt $R_{m,i}$
compose into a single rotation from magnet-local space to global space:

$$R_{total} = R \, R_{m,i}$$

### 3.2 Local vector geometry

The vector from a magnet to its sensor is built up frame by frame, not in one jump: first
the sensor position is expressed in the knob's frame, then the magnet's own resting
offset is subtracted *in that frame*, and only then is the result rotated into the
magnet's own (possibly tilted) local frame:

$$\mathbf{v}_{l,i} = R_{m,i}^T\Big(R^T(\mathbf{s}_i - \mathbf{t}) - \mathbf{m}_i\Big) = R_{total}^T(\mathbf{s}_i - \mathbf{t}) \;-\; R_{m,i}^T \mathbf{m}_i$$

The order matters for getting the right physical answer: $\mathbf{m}_i$ is subtracted
before the magnet-tilt rotation is applied, so the correct constant offset term is
$R_{m,i}^T\mathbf{m}_i$, not a bare $\mathbf{m}_i$.

### 3.3 Cylindrical query and interpolation

The magnets are axially polarized, so the field they produce is symmetric around their
own local axis and depends only on radius and height relative to that axis:

$$r = \sqrt{x_l^2+y_l^2}, \qquad z = z_l, \qquad \text{where } \mathbf{v}_{l,i}=(x_l,y_l,z_l)$$

A precomputed, $C^1$-continuous bicubic interpolation table is queried at $(r,z)$,
returning the two cylindrical field components $B_r, B_z$ and their four spatial
partials $\partial B_r/\partial r$, $\partial B_z/\partial r$, $\partial B_r/\partial z$,
$\partial B_z/\partial z$.

### 3.4 Local field reconstruction

The scalar cylindrical field is rotated back into the 3D local Cartesian frame using
directional cosines $c_x = x_l/r$, $c_y = y_l/r$. Together, §3.3–3.4 constitute the
function $B_i$ from §2:

$$\mathbf{B}_{l,i} = B_i(\mathbf{v}_{l,i}) = \begin{bmatrix} B_r c_x \\ B_r c_y \\ B_z \end{bmatrix}$$

### 3.5 Back to global frame, then gain

The local field is rotated into the global frame, then passed through the sensor's own
gain/distortion matrix to give the predicted sensor reading:

$$\mathbf{B}_{g,i} = R_{total}\, \mathbf{B}_{l,i}, \qquad \hat{\mathbf{B}}_i = f_i(\boldsymbol\rho) = G_i\, \mathbf{B}_{g,i}$$

(An additive sensor baseline/offset, if calibrated, is handled upstream as a
correction to the raw measurement rather than as a term in this model.)

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

From $\mathbf{B}_{g,i}=R_{total}\mathbf{B}_{l,i}$, the product rule gives:

$$\Delta\mathbf{B}_{g,i} = (\Delta R_{total})\,\mathbf{B}_{l,i} + R_{total}\,\Delta\mathbf{B}_{l,i}$$

$R_{m,i}$ is constant, so $\Delta R_{total} = (\Delta R)R_{m,i} = [\Delta\boldsymbol{\omega}]_\times R\, R_{m,i} = [\Delta\boldsymbol{\omega}]_\times R_{total}$ (Identity 2). Substituting, then applying Identity 1 with $\mathbf{a}=\Delta\boldsymbol{\omega}$, $\mathbf{b}=\mathbf{B}_{g,i}=R_{total}\mathbf{B}_{l,i}$:

$$[\Delta\boldsymbol{\omega}]_\times \mathbf{B}_{g,i} = -[\mathbf{B}_{g,i}]_\times \Delta\boldsymbol{\omega}$$

$$\Rightarrow \quad \Delta\mathbf{B}_{g,i} = -[\mathbf{B}_{g,i}]_\times \Delta\boldsymbol{\omega} + R_{total}\,\Delta\mathbf{B}_{l,i}$$

### 4.C Inner layer — local vector variation

The local field's variation is driven by the local query vector, via the analytic
Jacobian of §4.D:

$$\Delta\mathbf{B}_{l,i} = J_{local}\,\Delta\mathbf{v}_{l,i}$$

From $\mathbf{v}_{l,i} = R_{total}^T(\mathbf{s}_i-\mathbf{t}) - R_{m,i}^T\mathbf{m}_i$, and since the
$R_{m,i}^T\mathbf{m}_i$ term is entirely constant, only the first term varies:

$$\Delta\mathbf{v}_{l,i} = \Delta(R_{total}^T)(\mathbf{s}_i-\mathbf{t}) + R_{total}^T(-\Delta\mathbf{t})$$

Let $\mathbf{v} = \mathbf{s}_i - \mathbf{t}$ (the **global** sensor-to-magnet vector).
Using Identity 2, then Identity 1 with $\mathbf{a}=\Delta\boldsymbol{\omega}$,
$\mathbf{b}=\mathbf{v}$:

$$\Delta(R_{total}^T)\,\mathbf{v} = -R_{total}^T[\Delta\boldsymbol{\omega}]_\times \mathbf{v} = R_{total}^T[\mathbf{v}]_\times \Delta\boldsymbol{\omega}$$

$$\Rightarrow \quad \Delta\mathbf{v}_{l,i} = R_{total}^T[\mathbf{v}]_\times\,\Delta\boldsymbol{\omega} \;-\; R_{total}^T\,\Delta\mathbf{t}$$

*(Remark: had we instead used the simpler-looking but incomplete $\mathbf{v}_{l,i}=R_{total}^T(\mathbf{s}_i-\mathbf{t})-\mathbf{m}_i$ from §3.2, the derivative would come out identical — differentiating any constant term, whatever its exact form, always contributes zero. The correction in §3.2 matters for the field value, not for the Jacobian.)*

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

### 4.E Assembly — folding it all back together

Substitute §4.C's $\Delta\mathbf{v}_{l,i}$ into $\Delta\mathbf{B}_{l,i}=J_{local}\Delta\mathbf{v}_{l,i}$, then into §4.B:

$$\Delta\mathbf{B}_{g,i} = -[\mathbf{B}_{g,i}]_\times\Delta\boldsymbol{\omega} + R_{total}J_{local}\Big(R_{total}^T[\mathbf{v}]_\times\Delta\boldsymbol{\omega} - R_{total}^T\Delta\mathbf{t}\Big)$$

Define the shared $3\times3$ core, computed once per sensor:

$$M = R_{total}\,J_{local}\,R_{total}^T$$

Distributing and grouping by $\Delta\mathbf{t}$ / $\Delta\boldsymbol{\omega}$:

$$\Delta\mathbf{B}_{g,i} = \underbrace{(-M)}_{\text{translation}}\Delta\mathbf{t} + \underbrace{\big(M[\mathbf{v}]_\times - [\mathbf{B}_{g,i}]_\times\big)}_{\text{rotation}}\Delta\boldsymbol{\omega}$$

Folding in the gain from §4.A gives the final $3\times3$ blocks:

$$J_{trans} = G_i(-M), \qquad J_{rot} = G_i\big(M[\mathbf{v}]_\times - [\mathbf{B}_{g,i}]_\times\big)$$

**Crucial detail:** $[\mathbf{B}_{g,i}]_\times$ must be built from the pre-gain, physical
field $\mathbf{B}_{g,i}$, *before* $G_i$ is applied — it appears inside §4.B, ahead of
§4.A's gain step, so using the gained field here would be a subtly wrong Jacobian even
though the residual itself is computed post-gain.

### 4.F Stacking to 9×6

Each sensor's $3\times6$ block $D_{\boldsymbol\rho}f_i = [\,J_{trans,i}\;\;J_{rot,i}\,]$
occupies its own three rows of the shared $9\times6$ matrix $J=D_{\boldsymbol\rho}F$ from
§2 — all three sensors share the same $\mathbf{t}, R$ (and hence the same
$\Delta\mathbf{t}, \Delta\boldsymbol{\omega}$ columns), since they observe one rigid
knob:

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
the $r=0$ branch), and the full $9\times6$ forward-model Jacobian — can be, and should
continue to be, checked against central finite differences swept over a grid of poses,
sensor gains, and magnet tilts. Perturbing $R$ with the exact matrix exponential
$\exp([\mathbf{w}]_\times)$ (not the first-order $I+[\mathbf{w}]_\times$) for the
rotation columns is important here: using the same first-order approximation on both
sides would let a shared error hide from the comparison.

---

## 7. Bundle Calibration — the Arrowhead Normal Equations

§1–6 treat $\mathbf{s}_i$, $\mathbf{m}_i$, $R_{m,i}$, $G_i$ as frozen and solve for one
pose. Calibration is the inverse problem: fit those constants themselves, from $N$
captured frames at once, each contributing its own unknown pose. This section derives
the shape of the resulting normal equations and how they're solved without ever forming
the full dense system. It takes §2–§4's Jacobian as given — frame $k$'s own-pose block
below is exactly $J$ from §4.F, unmodified — and does not re-derive how the shared
constants' columns are computed; that extension of §4's chain rule to $\mathbf s_i$,
$\mathbf m_i$, $R_{m,i}$, $G_i$ (plus sensor offset) lives with the prototype
implementation in `firmware/experimental/bundle_calibration_jacobian/`.

### 7.1 Setup

Let $\mathbf{x}\in\mathbb{R}^P$ collect the shared constants being fit (magnet
position/tilt/strength, sensor gain/offset — $P\approx45$), and let
$\Delta\boldsymbol{\rho}_k\in\mathbb{R}^6$ be frame $k$'s own pose update ($k=1,\dots,N$,
$N\approx60$), in the same local-chart sense as §2. The joint unknown is the stack
$(\Delta\mathbf{x},\,\Delta\boldsymbol{\rho}_1,\dots,\Delta\boldsymbol{\rho}_N)\in
\mathbb{R}^{P+6N}$.

Frame $k$ contributes its own residual $\mathbf{r}_k\in\mathbb{R}^9$ — the same
per-sensor forward model as §2, evaluated at frame $k$'s pose and the current shared
estimate. **This section assumes $\mathbf{r}_k$ depends on $\mathbf{x}$ and
$\boldsymbol{\rho}_k$ only** — not on any other frame's pose, not even indirectly through
some function of the relationship between poses. A term like a temporal-smoothness prior
linking consecutive frames would violate this and break everything below; nothing of
that kind exists here, but it's the specific thing that would need to hold for whatever
is added later.

### 7.2 Block structure of the stacked Jacobian

Under that assumption, differentiating $\mathbf{r}_k$ produces exactly two nonzero
blocks — $C_k=\partial\mathbf{r}_k/\partial\mathbf{x}$ and
$P_k=\partial\mathbf{r}_k/\partial\boldsymbol{\rho}_k$, the latter being §4.F's $J$
verbatim, one instance per frame — and every other column of frame $k$'s row-block is
exactly zero, since $\mathbf{r}_k$ doesn't depend on those variables at all:

$$J = \begin{bmatrix}
C_1 & P_1 & 0   & \cdots & 0 \\
C_2 & 0   & P_2 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
C_N & 0   & 0   & \cdots & P_N
\end{bmatrix} \in \mathbb{R}^{9N\times(P+6N)}$$

### 7.3 The normal equations inherit the shape

Gauss-Newton (§5) forms $H=J^TJ$ and $\mathbf{g}=-J^T\mathbf{r}$ from whatever $J$ is
handed to it; §5 used the single-frame $9\times6$ one, this is the same step applied to
§7.2's stacked one. Multiplying $J^T$ by $J$ block-by-block:

$$J^TJ = \begin{bmatrix}
A        & B_1    & B_2    & \cdots & B_N   \\
B_1^\top & D_1    & 0      & \cdots & 0     \\
B_2^\top & 0      & D_2    & \cdots & 0     \\
\vdots   & \vdots & \vdots & \ddots & \vdots \\
B_N^\top & 0      & 0      & \cdots & D_N
\end{bmatrix} = H, \qquad
A=\sum_{k=1}^N C_k^\top C_k,\quad B_k=C_k^\top P_k,\quad D_k=P_k^\top P_k$$

with the right-hand side splitting the same way:
$\mathbf{a}=-\sum_k C_k^\top\mathbf{r}_k$, $\mathbf{b}_k=-P_k^\top\mathbf{r}_k$.

The off-diagonal pose blocks are **exactly** zero, not merely small: block $(k,l)$ for
$k\ne l$ multiplies row $k$'s pose-$l$ column — which is $0$ by §7.2 — against
$P_l$, so it vanishes regardless of the data. The arrowhead shape is a direct
consequence of §7.1's independence assumption, not a numerical coincidence that happens
to hold approximately. ($A$ is dense: every frame contributes a $C_k^\top C_k$ term to
the same $P\times P$ block. A damping term $\lambda I$, exactly as in §5, can be added to
$D_k$ — and $A$ — without changing this structure; omitted above for clarity.)

### 7.4 Eliminating the pose blocks

As pure linear algebra, independent of where this system came from: row $k$ of $H\,
\Delta=\mathbf{g}$ reads $B_k^\top\Delta\mathbf{x}+D_k\Delta\boldsymbol{\rho}_k=
\mathbf{b}_k$, so — provided $D_k$ is invertible —

$$\Delta\boldsymbol{\rho}_k = D_k^{-1}\big(\mathbf{b}_k - B_k^\top\Delta\mathbf{x}\big)$$

is an *exact* expression for frame $k$'s pose update, still carrying the unknown
$\Delta\mathbf{x}$. Substituting into the top block row and collecting
$\Delta\mathbf{x}$ terms:

$$\left(A - \sum_{k=1}^N B_kD_k^{-1}B_k^\top\right)\Delta\mathbf{x} \;=\; \mathbf{a} - \sum_{k=1}^N B_kD_k^{-1}\mathbf{b}_k$$

a $P\times P$ system for $\Delta\mathbf{x}$ alone — the Schur complement of
$\mathrm{diag}(D_1,\dots,D_N)$ in $H$. Nothing here is approximate: solving this for
$\Delta\mathbf{x}$ and reading each $\Delta\boldsymbol{\rho}_k$ off the boxed line above
gives the identical answer (to roundoff) that factoring the full $(P+6N)\times(P+6N)$
system directly would. It's a reduction in work, not in accuracy — the same trick
classical bundle adjustment uses to eliminate camera poses before solving for scene
structure.

### 7.5 Streaming the elimination

The reduced system above still looks like it needs every frame's $B_k,D_k$ at once to
form its two sums. It doesn't: each term touches only frame $k$'s own $C_k,P_k,
\mathbf{r}_k$, so both sums accumulate one frame at a time. Build frame $k$'s local
blocks, factor its $6\times6$ $D_k$ once, fold its contribution into a running
$P\times P$ accumulator, and discard everything about that frame except what the
accumulator retains — no frame's data is ever held alongside another's. After all $N$
frames, solve the accumulated $P\times P$ system once for $\Delta\mathbf{x}$, then make
a second pass, rebuilding each frame's $B_k,D_k,\mathbf{b}_k$ to recover its
$\Delta\boldsymbol{\rho}_k$ from the boxed line in §7.4. Peak memory is one accumulator
plus one frame's transient blocks — $O(P^2)$, independent of $N$ — instead of the full
$(P+6N)^2$ a direct solve would need.

`firmware/experimental/bundle_calibration_jacobian/schur_normal_equations.h` implements
exactly this: `FrameNormalEquations`/`FramePoseBlock` accumulate one frame's $C_k,P_k$
contribution, `SharedNormalEquations::absorb_frame` performs §7.4's fold into $A,
\mathbf{a}$, and `solve_frame_pose_update` performs the back-substitution. It's checked
against a dense assembly of the full arrowhead system rather than finite differences —
§7.4 is an identity with a ground-truth answer, not an approximation to test for
plausibility — see that directory's `README.md` for the verification methodology and
`TODO/on-device-calibration.md` for what is and isn't built on top of it.
