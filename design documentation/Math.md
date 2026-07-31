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
