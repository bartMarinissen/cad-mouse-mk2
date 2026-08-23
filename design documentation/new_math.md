# Rewrite of the mathmatics, this time human-based

The core of this work is the forward model for a single sensor.
Everything difficult is downstream of that. So that is where we start.
This part ends at the actual magnet-model (dipole or interpolated).
The magnet-model is generally treated as opaque. In [[magnet-model]] we will go through it. Outside of it, it is a (mostly) black box.

The full forward model for a full frame (so for 3 sensor readings given a single knob pose) is then just
vertically stacking 3 of those predictions for 1 knob pose. When we start to look at the jacobian, its again just vertically stacking things.

Once we have both full forward models, we will move towards the first challenge: the jacobian of the full forward model w.r.t. the knob pose.

At that point, we are done with the things we need for our solver. But then we look towards bundle-callibration.
That widens the jacobian to include, roughly-speaking, everything.
That part might just go into its own file.

## Notation conventions.

### Frames
Frames are written as superscripts. We know three frames:
- The world frame $w$
- The knob frame $k$
- The manget-local frame $l$ (this one might be unused).

### Multi-letter variables
Mathematicians and physicist went to far with demanind single letter variables.
Subscripts are precious, I will have multi-letter variables.
Where they are used with implicit application (linalg, multiplication) we will use spacing like so: $Ab \; bc$. Where feasible, we shall be shackled by the single-letter convention.

## Single sensor forward model
The core of doing any type of modeling right is picking the right shape for your data. So thats where we start.

### Knob variables
$x^w \in \R^3$ (mm) The position of the knob origin in the world frame.

$\omega \in \R^3$ (franken-units) the orientation of the knob as a 3d vector. It maps orientations in the knob frame $k$ to the world from $w$

$R(\omega) \in SO(3) \subset \R^{3 \times 3}$. The rotation matrix corresponding to $\omega$. \
$R(\omega) = \exp([\omega]_\times) $

### Sensor variables (for sensor index j)
${Bm}^w_j \in \R^3$ (mT) The raw measurement at sensor $j$. This is indisputably an input and cannot be changed by the model or modeling choices.

$S_j^w \in \R^3$ (mm) the sensor position in the world frame

$g_j \in \R^3$ the sensor gain per axis.

$\theta_j \in \R^3$ (mT) the sensor offset

$Bc^w_j \in \R^3$ (mT) the callibrated reading of sensor j. \
$Bc^w_j = \textrm{diag}(g_j) \; Bm^w_j + \theta_j $

$\hat{B}^w_j \in \R^3$ (mT) the model predicted magnetic field at sensor $j$ in the world frame. Calculating this is the entire point of this section.

### Magnet variables (for magnet index i) (all in knob frame)
$M_i^k \in \R^3$ (mm) Magnet position in the knob frame

$d_i^k \in \R^3$ (mT) Magnetic Dipole moment. Direction encodes the orientation of the magnet. Magnitude encodes the strength.

### The magnet model

$F: (\R^3 \times \R^3) \mapsto \R^3$ The magnet model. \
$F(x, d) \in \R^3$ (mT) is the magnetic field strengt at position $x$ due to a magnet at the origin with orientation and strength $d$.

We will mix this between a dipole model and an interpolated more accurate model. But both models will have this interface.

#### Properties
These are useful properties to know about the model that we assume:

- Radial symmerty around $d$ (because our magnet has radial symmetry and is polarized across its axis of symmetry)
- Commutation with rotation:
$$
  \textrm{if }\; R \in SO(3) \;\textrm{ then: }\; F(Rx, Rd) = R\; F(x, d)
$$
- Linear in the magnitude of $d$:
$$
  \textrm{if }\; a \in \R \;\textrm{ then: }\; F(x, a\,d) =  a \,F(x, d)
$$

### Derived variables (world frame)
First some standard frame transformations.

$M_i^w = R(\omega) M_i^k + x^w$ the magnet position in the world frame

$d_i^w = R(\omega) d_i^k$ the magnet dipole representation in the world frame (no translation because this is a direction not a position)


$B_{i,j}^w \in \R^3$ (mT) the magnetic field at sensor $j$ due to magnet $i$ in the world frame\
$B_{i,j}^w=F(S_j^w - M_i^w, d_i^w) $

### Derived variables (knob frame)

$S^k_j = R(-\omega) \; (S_j^w - x^w)$ The sensor position in the knob frame.

$B_{i,j}^k \in \R^3$ (mT) the magnetic field at sensor $j$ due to magnet $i$. \
$B_{i,j}^k=F(S_j^k - M_i^k, d_i^k) $

Which then also lets us write
$$
B_{i,j}^w = R(-\omega) B_{i,j}^k 
= R(-\omega) F(S_j^k - M_i^k, d_i^k) 
$$

### Two solutions for $\hat{B}^w_j$
Both solutions depend on the following obvious supperposition principle:
$$
\hat{B}^w_j = \sum_i B_{i,j}^w 
$$

But we can go through the knob frame, or we can do all the work in the world frame. Depending on which expression for $B_{i,j}^w$ we use.

$$
\begin{align*}
\hat{B}^w_j 
 = \sum_i B_{i,j}^w 
 =& \sum_i R(-\omega) B_{i,j}^k \\
 =& \sum_i  R(-\omega) F(S_j^k - M_i^k, d_i^k) \\
 = R(-\omega)&  \sum_i  F\big( R(-\omega) \; (S_j^w - x^w) - M_i^k, d_i^k \big) \\
\end{align*}

\begin{align*}
\hat{B}^w_j 
&= \sum_i B_{i,j}^w \\
&= \sum_i F(S_j^w - M_i^w, d_i^w) \\
&= \sum_i F\big( S_j^w - R(\omega) M_i^k - x^w, R(\omega) d_i^k \big) \\
\end{align*}
$$

Comparing the two approaches computational cost:
- Going through the knob frame lets us pull one rotation out of the matrix. But the remaining rotation cannot be precomputed. So it yields 4 rotation matrix applications.
    - So for a full frame it costs 12 applications.
- Going through the world frame does not allow pulling a rotation out. But it does allow pre-computing the two terms that require a matrix multiplication $ M_i^w$ and $d_i^w$ because they do not depend on the sensor $j$.
    - So for a full frame it takes only 6 matrix applications

### Residual for one sensor

$r^w_j \in \R^3$ (mT) the residual at sensor $j$ (in the world frame) we want to minimize.

$$
\begin{align*}
r^w_j &= Bc_j^w - \hat{B}_j^w \\
      &= \textrm{diag}(g_j) \; Bm^w_j + \theta_j - \sum_i F\big( S_j^w - R(\omega) M_i^k - x^w, R(\omega) d_i^k \big)

\end{align*}
$$

## Full frame forward model
We retain the knob position and orientation $x$ and $\omega$ for all sensor readings in a frame.

So the residual for a full frame is just the $9\times1$ stacked vector:
$$
r = \begin{pmatrix}
    r_0^w \\
    r_1^w \\
    r_2^w \\
\end{pmatrix}
$$

As we said, this part is just trivial assembly. We might also define the full prediction of all 3 sensors first. But really, that is just more vector stacking.

---------------
Barier
---------------

## Jacobian w.r.t knob pose

### What are we varying?

The residual for sensor $j$ is:
$$
r^w_j = Bc^w_j - \hat{B}^w_j
$$
Since the calibrated sensor reading $Bc^w_j$ is fixed, the residual variation is simply the negative variation of the predicted field:
$$
\delta r^w_j = - \delta \hat{B}^w_j
$$

Because $\hat{B}^w_j = \sum_i B_{i,j}^w$, the total variation is the sum of variations across all magnets $i$:
$$
\delta \hat{B}^w_j = \sum_i \delta B_{i,j}^w
$$

The knob pose has 6 degrees of freedom:
1. **Translation variation** $\delta x^w \in \R^3$ (3 DOF)
2. **Rotation variation** $\delta \omega \in \R^3$ (3 DOF), where the orientation updates as $R \to \exp([\delta \omega]_\times) R \approx (I + [\delta \omega]_\times) R$.

---

### 1. Translation Variation w.r.t. $x^w$

From the forward model in the world frame:
$$
B_{i,j}^w = F\big(S_j^w - M_i^w, \; d_i^w\big) = F\big(S_j^w - R(\omega)M_i^k - x^w, \; R(\omega)d_i^k\big)
$$

Let $J_p = \frac{\partial F}{\partial x} \in \R^{3 \times 3}$ be the spatial Jacobian of $F$ with respect to its first arugment, the position displacement.

By the chain rule, and the fact that only one term depends on $x^w$ we immediately get:
$$
\delta B_{i,j}^w = J_p (-\delta x^w) = - J_p \, \delta x^w
$$

Summing over all magnets, the translation Jacobian for sensor $j$ is:
$$
\frac{\partial \hat{B}^w_j}{\partial x^w} = - \sum_i J_p
$$

---

### 2. Rotation Variation w.r.t. $\omega$

We start with some basics on the variations of rotations.

Under a rotation variation by an infinitesimal angle-axis vector $\delta \omega \in \R^3$:
1. **Variation of $R(\omega)$:** 
   An incremental rotation matrix is $\Delta R = \exp([\delta \omega]_\times) \approx I + [\delta \omega]_\times$. Applying it to $R(\omega)$ gives:
   $$
   R_{\text{new}} = \Delta R \; R(\omega) \approx (I + [\delta \omega]_\times) R(\omega) = R(\omega) + [\delta \omega]_\times R(\omega)
   $$
   The variation is the change $\delta R(\omega) = R_{\text{new}} - R(\omega)$:
   $$
   \delta R(\omega) = [\delta \omega]_\times R(\omega)
   $$

2. **Variation of the inverse $R(-\omega) = R(\omega)^T$:** 
   Because transposition is linear, the variation of the transpose is the transpose of the variation:
   $$
   \delta\big(R(\omega)^T\big) = R_{\text{new}}^T - R(\omega)^T  = \big(\delta R(\omega)\big)^T
   $$
   Substituting $\delta R(\omega) = [\delta \omega]_\times R(\omega)$:
   $$
   \delta R(-\omega) = \big([\delta \omega]_\times R(\omega)\big)^T
   $$
   Applying the product transpose rule and skew-symmetry:
   $$
   \delta R(-\omega) = R(\omega)^T [\delta \omega]_\times^T = R(\omega)^T \big(- [\delta \omega]_\times\big) = - R(\omega)^T [\delta \omega]_\times = - R(-\omega) [\delta \omega]_\times
   $$


---------
Checked till here
--------

With that done we use the knob-frame formulation as an intermediate step:
$$
B_{i,j}^w = R(\omega) B_{i,j}^k = R(\omega) \, F\big( R(-\omega)(S_j^w - x^w) - M_i^k, \; d_i^k \big)
$$

Inside $F$, the dipole $d_i^k$ and position $M_i^k$ are constant parameters fixed in the knob frame, so $\delta d_i^k = 0$ and $\delta M_i^k = 0$.


Applying the product rule of variations to $B_{i,j}^w = R(\omega) B_{i,j}^k$:
$$
\delta B_{i,j}^w = \underbrace{\delta R(\omega) \, B_{i,j}^k}_{\text{Term 1: rotating the field vector}} + \underbrace{R(\omega) \, \delta B_{i,j}^k}_{\text{Term 2: sensor moving through the field}}
$$

#### Term 1: Rotating the field vector
$$
\delta R(\omega) \cdot B_{i,j}^k = [\delta \omega]_\times \big(R(\omega) B_{i,j}^k\big) = [\delta \omega]_\times B_{i,j}^w = - [B_{i,j}^w]_\times \delta \omega
$$

#### Term 2: Sensor moving through the knob-frame field
In the knob frame, the sensor position is $S_j^k = R(-\omega)(S_j^w - x^w)$. Its variation is:
$$
\delta S_j^k = \delta R(-\omega) (S_j^w - x^w) = - R(-\omega) [\delta \omega]_\times (S_j^w - x^w) = - R(-\omega) \big( \delta \omega \times (S_j^w - x^w) \big)
$$
Using cross-product anti-symmetry $\delta \omega \times v = - [v]_\times \delta \omega$:
$$
\delta S_j^k = + R(-\omega) [S_j^w - x^w]_\times \delta \omega
$$

Since $M_i^k$ and $d_i^k$ are constant in the knob frame, the variation of $B_{i,j}^k = F(S_j^k - M_i^k, d_i^k)$ is:
$$
\delta B_{i,j}^k = J_p^k \, \delta S_j^k = J_p^k \, R(-\omega) [S_j^w - x^w]_\times \delta \omega
$$

Multiplying by $R(\omega)$ from the left rotates this contribution into world coordinates:
$$
R(\omega) \cdot \delta B_{i,j}^k = \underbrace{\big(R(\omega) J_p^k R(-\omega)\big)}_{J_p} [S_j^w - x^w]_\times \delta \omega = J_p [S_j^w - x^w]_\times \delta \omega
$$
where $J_p = R(\omega) J_p^k R(-\omega) \in \R^{3 \times 3}$ is the spatial Jacobian in world coordinates.

---

### Total Rotation Jacobian for Sensor $j$

Adding Term 2 and Term 1 together:
$$
\delta B_{i,j}^w = \Big( J_p [S_j^w - x^w]_\times - [B_{i,j}^w]_\times \Big) \delta \omega
$$

Summing over all magnets $i$, the rotation Jacobian for sensor $j$ is:
$$
\frac{\partial \hat{B}^w_j}{\partial \omega} = \left( \sum_i J_p \right) [S_j^w - x^w]_\times - [\hat{B}^w_j]_\times
$$

---

### 3. Assembling the Full Sensor Jacobian ($3 \times 6$)

For sensor $j$, let:
- $J_{\text{disp}, j} = \sum_i J_p(S_j^w - M_i^w, d_i^w) \in \R^{3 \times 3}$ (accumulated spatial displacement Jacobian)
- $v_j^w = S_j^w - x^w$ (lever arm from knob origin to sensor $j$)
- $\hat{B}_j^w = \sum_i B_{i,j}^w$ (predicted total field at sensor $j$)

The $3 \times 6$ Jacobian of the predicted field $\hat{B}_j^w$ w.r.t. knob pose $(x^w, \omega)$ is:
$$
J_{\hat{B}, j} = \begin{pmatrix} -J_{\text{disp}, j} & \Big( J_{\text{disp}, j} [v_j^w]_\times - [\hat{B}_j^w]_\times \Big) \end{pmatrix} \in \R^{3 \times 6}
$$

And the Jacobian of the residual $r_j^w = Bc_j^w - \hat{B}_j^w$ is:
$$
J_{r, j} = - J_{\hat{B}, j} = \begin{pmatrix} J_{\text{disp}, j} & \Big( [\hat{B}_j^w]_\times - J_{\text{disp}, j} [v_j^w]_\times \Big) \end{pmatrix} \in \R^{3 \times 6}
$$

---

### 4. Full Frame Assembly ($9 \times 6$ Jacobian)

Stacking the 3 sensors vertically gives the full $9 \times 6$ Jacobian matrix:
$$
J = \begin{pmatrix}
J_{r, 0} \\
J_{r, 1} \\
J_{r, 2}
\end{pmatrix} \in \R^{9 \times 6}
$$

In each Gauss-Newton / Levenberg-Marquardt solver iteration:
1. Evaluate $\hat{B}^w_j$ and $J_{r, j}$ for each sensor $j \in \{0, 1, 2\}$.
2. Form the normal equations: $H = J^T J + \lambda I$ (a $6 \times 6$ system) and gradient $g = - J^T r$.
3. Solve $H \Delta = g$ for $\Delta = (\delta x^w, \delta \omega) \in \R^6$.
4. Apply the state updates:
   $$x^w \leftarrow x^w + \delta x^w$$
   $$R \leftarrow \exp([\delta \omega]_\times) R$$

---

### 5. Spatial Jacobian $J_p$ for Specific Magnet Models

#### A. Far Field: Point Dipole Model
For a point dipole with moment $m \in \R^3$ at displacement $r \in \R^3$ with distance $\rho = \|r\|$:
$$
B(r, m) = \frac{1}{4\pi} \left( \frac{3 (m \cdot r) r}{\rho^5} - \frac{m}{\rho^3} \right)
$$
Differentiating w.r.t. displacement $r$:
$$
J_p(r, m) = \frac{1}{4\pi} \left[ \frac{3}{\rho^5} \Big( m r^T + r m^T + (m \cdot r) I_3 \Big) - \frac{15 (m \cdot r)}{\rho^7} r r^T \right]
$$
Note that $J_p$ is symmetric ($J_p = J_p^T$) because $\nabla \times B = 0$ in vacuum.

#### B. Near Field: Axisymmetric Bicubic Model
For an axially symmetric cylinder with axis unit vector $\hat{a}$ and origin $M_0$:
1. Decompose displacement $r = S - M_0$ into axial ($z$) and radial ($r_\perp, \rho$) components:
   $$z = r \cdot \hat{a}, \quad r_\perp = r - z \hat{a}, \quad \rho = \|r_\perp\|$$
2. Interpolate 2D field and derivatives from the table: $(B_\rho, B_z)$ and $\begin{pmatrix} \frac{\partial B_\rho}{\partial \rho} & \frac{\partial B_\rho}{\partial z} \\ \frac{\partial B_z}{\partial \rho} & \frac{\partial B_z}{\partial z} \end{pmatrix}$.
3. Construct the 3D field and 3D Jacobian $J_p$ in world coordinates using radial unit vector $\hat{\rho} = r_\perp / \rho$:
   $$B = B_z \hat{a} + B_\rho \hat{\rho}$$
   $$J_p = \hat{a} \left( \frac{\partial B_z}{\partial \rho} \hat{\rho}^T + \frac{\partial B_z}{\partial z} \hat{a}^T \right) + \hat{\rho} \left( \frac{\partial B_\rho}{\partial \rho} \hat{\rho}^T + \frac{\partial B_\rho}{\partial z} \hat{a}^T \right) + \frac{B_\rho}{\rho} \left( I_3 - \hat{a}\hat{a}^T - \hat{\rho}\hat{\rho}^T \right)$$

