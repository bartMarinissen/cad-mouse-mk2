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

## Jacobian w.r.t. the knob pose

This is the first thing that is actually hard. The forward model above was
assembly; this is where we have to differentiate through a rotation, twice, and
keep the signs.

### What we are varying

We hold everything fixed except the knob pose. The measurement $Bm^w_j$ is an
input, the gain $g_j$ and offset $\theta_j$ are fixed for now, so the calibrated
reading $Bc^w_j$ does not move at all:
$$
\delta r^w_j = - \delta \hat{B}^w_j
$$

And because $\hat{B}^w_j = \sum_i B^w_{i,j}$ is a sum, its variation is the sum
of the variations:
$$
\delta \hat{B}^w_j = \sum_i \delta B^w_{i,j}
$$

So the whole job reduces to one (magnet $i$, sensor $j$) pair. There are nine of
them per frame, and all nine are the same derivation.

The pose has six degrees of freedom, which we vary as
$$
x^w \to x^w + \delta x^w
\qquad\text{and}\qquad
R(\omega) \to \exp([\delta \omega]_\times) \, R(\omega)
$$

The rotation update is applied on the **left**, i.e. in the world frame. That is
a convention and we are free to pick it, but everything downstream depends on
having picked it, so it is worth saying loudly: $\delta \omega$ is a rotation
*of the knob as seen from the world*, not a rotation expressed in knob
coordinates.

### Naming the two derivatives of the magnet model

We keep $F$ opaque. What we do need is that it is differentiable in both of its
arguments, and a name for each derivative:

$Jp(x, d) \in \R^{3\times3}$ the derivative of $F$ w.r.t. its **position**
argument. Entry $(a,b)$ is $\partial F_a / \partial x_b$. This is the field
gradient at $x$.

$Jd(x, d) \in \R^{3\times3}$ the derivative of $F$ w.r.t. its **dipole**
argument. Entry $(a,b)$ is $\partial F_a / \partial d_b$.

Both are functions of the same two arguments $F$ takes, and both are things the
magnet model owes us — see [[magnet-model]]. We will write
$$
Jp_{i,j} = Jp(p^w_{i,j},\; d_i^w)
\qquad
Jd_{i,j} = Jd(p^w_{i,j},\; d_i^w)
$$
for the pair-specific values, where
$$
p^w_{i,j} = S_j^w - M_i^w = S_j^w - R(\omega) M_i^k - x^w
$$
is the displacement from magnet $i$ to sensor $j$ in the world frame — exactly
the first argument the world-frame forward model hands to $F$. So
$B^w_{i,j} = F(p^w_{i,j}, d_i^w)$, and to first order
$$
\delta B^w_{i,j} = Jp_{i,j} \; \delta p^w_{i,j} \;+\; Jd_{i,j} \; \delta d_i^w
$$

Everything from here is working out what $\delta p^w_{i,j}$ and $\delta d_i^w$
are in terms of $\delta x^w$ and $\delta \omega$.

### Variations of a rotation

Two facts, used constantly below.

**Cross-product antisymmetry.** For any $a, b \in \R^3$:
$$
[a]_\times b = a \times b = - b \times a = - [b]_\times a
$$
This is what lets us move a $\delta \omega$ from *inside* a cross product to the
right-hand side where it can be factored out as a column of a Jacobian. Every
time a rotation variation appears we will be reaching for it.

**Variation of $R(\omega)$.** From the left-update convention,
$$
R_{new} = \exp([\delta \omega]_\times) R(\omega) \approx (I + [\delta \omega]_\times) R(\omega)
$$
so
$$
\delta R(\omega) = R_{new} - R(\omega) = [\delta \omega]_\times R(\omega)
$$

**Variation of $R(-\omega)$.** A rotation matrix is orthogonal, so
$R(-\omega) = R(\omega)^{-1} = R(\omega)^T$, and transposition is linear, so the
variation of the transpose is the transpose of the variation:
$$
\delta R(-\omega) = \big(\delta R(\omega)\big)^T = \big([\delta \omega]_\times R(\omega)\big)^T
= R(\omega)^T [\delta \omega]_\times^T = - R(-\omega) [\delta \omega]_\times
$$
using $[\,a\,]_\times^T = -[\,a\,]_\times$.

Note where the minus sign and the *side* went: the perturbation was on the left
of $R(\omega)$ and ends up on the right of $R(-\omega)$. Getting this backwards
produces a Jacobian that is wrong only in the rotation columns, which is exactly
the kind of error a solver hides by converging anyway, just slower.

### What the model's properties buy us

Above the barrier we asserted three properties of $F$. Two of them are
statements about *all* $x$ and $d$, so we may differentiate them, and doing so
constrains $Jp$ and $Jd$ against each other. This turns out to matter a lot.

**Differentiating the rotation-commutation property.** The property is
$$
F(Rx, Rd) = R \; F(x,d) \qquad \textrm{for all } R \in SO(3)
$$
Take $R = I + [w]_\times$ for infinitesimal $w$ and expand both sides to first
order. On the left, the arguments move by $[w]_\times x$ and $[w]_\times d$:
$$
F(Rx, Rd) \approx F(x,d) + Jp(x,d)\,[w]_\times x + Jd(x,d)\,[w]_\times d
$$
Apply antisymmetry to both, $[w]_\times x = -[x]_\times w$ and
$[w]_\times d = -[d]_\times w$:
$$
F(Rx, Rd) \approx F(x,d) - \Big( Jp(x,d)\,[x]_\times + Jd(x,d)\,[d]_\times \Big) w
$$
On the right:
$$
R\,F(x,d) \approx F(x,d) + [w]_\times F(x,d) = F(x,d) - [F(x,d)]_\times w
$$
These agree for every $w$, so the matrices are equal:
$$
\boxed{\;[F(x,d)]_\times = Jp(x,d)\,[x]_\times + Jd(x,d)\,[d]_\times\;}
$$

This says something physically reasonable: rigidly rotating the whole
configuration rotates the field, and the two ways the field can respond to that
rotation — the sensor sweeping through the field, and the magnet's axis turning
— must add up to exactly a rotation of the field vector.

**Differentiating the magnitude-linearity property.** The property is
$F(x, a\,d) = a\,F(x,d)$ for all $a \in \R$. Differentiate w.r.t. $a$ and set
$a = 1$:
$$
Jd(x,d)\; d = F(x,d)
$$
$F$ is homogeneous of degree one in $d$, so this is just Euler's theorem. It is
a free consistency check on any implementation of $Jd$: contracting $Jd$ with
its own dipole must return the field.

**Consequence: $Jd$ is not independent.** The two identities together determine
$Jd$ completely from $Jp$ and $F$. To see it, split an arbitrary $u \in \R^3$
into its component along $d$ and its component perpendicular to $d$:
$$
u = \underbrace{\frac{d\,(d \cdot u)}{|d|^2}}_{\textrm{along } d} \;+\; u_\perp
$$
Euler's identity handles the first part directly. For the second, note that
$[d]_\times[d]_\times u_\perp = d \times (d \times u_\perp) = d\,(d\cdot u_\perp) - |d|^2 u_\perp = -|d|^2 u_\perp$,
so $u_\perp = -[d]_\times [d]_\times u_\perp / |d|^2$, and since
$[d]_\times u_\perp = [d]_\times u$ (the along-$d$ part is annihilated), we get
$u_\perp = [d]_\times w$ with $w = -[d]_\times u/|d|^2$. That puts $u_\perp$
inside the range of $[d]_\times$, which is precisely where the commutation
identity tells us what $Jd$ does. Assembling:
$$
Jd(x,d) = \frac{1}{|d|^2}\Big( F(x,d)\, d^T \;-\; \big([F(x,d)]_\times - Jp(x,d)\,[x]_\times\big)[d]_\times \Big)
$$

So the magnet model only really owes us $Jp$. $Jd$ follows from the field, the
gradient, and the geometry we already have in hand. We will still write $Jd$
wherever it appears, because it is the honest name for that derivative and
substituting the reconstruction inline would make the derivations unreadable —
but it is never a second thing to tabulate or interpolate.

### Translation block

Only $p^w_{i,j}$ depends on $x^w$, and it does so trivially:
$$
\delta p^w_{i,j} = -\,\delta x^w, \qquad \delta d_i^w = 0
$$
so
$$
\delta B^w_{i,j} = - Jp_{i,j} \; \delta x^w
$$

Summing over magnets and writing
$$
JpSum_j = \sum_i Jp_{i,j} \in \R^{3\times3}
$$
for the total field gradient at sensor $j$:
$$
\frac{\partial \hat{B}^w_j}{\partial x^w} = - JpSum_j
$$

### Rotation block

Now the rotation, through the world-frame formulation
$$
B^w_{i,j} = F\big( S_j^w - R(\omega) M_i^k - x^w, \; R(\omega) d_i^k \big)
$$
Here $R(\omega)$ appears **twice**, and both occurrences vary. That is the whole
content of this section; miss one and the result is still smooth, still
plausible, and wrong.

**The position argument.** $S_j^w$, $M_i^k$ and $x^w$ are all constant under a
rotation variation, so
$$
\delta p^w_{i,j} = -\,\delta R(\omega)\, M_i^k = -[\delta \omega]_\times R(\omega) M_i^k = +\big[R(\omega) M_i^k\big]_\times \delta \omega
$$
using antisymmetry in the last step. Note $R(\omega) M_i^k = M_i^w - x^w$ is the
magnet's offset from the knob origin, expressed in world axes — the lever arm
the magnet swings on.

**The dipole argument.** $d_i^k$ is constant in the knob frame, so
$$
\delta d_i^w = \delta R(\omega)\, d_i^k = [\delta \omega]_\times R(\omega) d_i^k = [\delta \omega]_\times d_i^w = -\big[d_i^w\big]_\times \delta \omega
$$

Putting both into $\delta B^w_{i,j} = Jp_{i,j}\,\delta p^w_{i,j} + Jd_{i,j}\,\delta d_i^w$:
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[R(\omega) M_i^k\big]_\times \;-\; Jd_{i,j} \big[d_i^w\big]_\times \Big)\, \delta \omega
$$

This is correct but unsatisfying: it needs $Jd$, and every term carries both
indices so nothing can be shared between magnets.

### Dropping $Jd$ from the pose Jacobian

The commutation identity, evaluated at this pair's arguments
$x = p^w_{i,j}$, $d = d_i^w$, reads
$$
Jd_{i,j}\big[d_i^w\big]_\times = \big[B^w_{i,j}\big]_\times - Jp_{i,j}\big[p^w_{i,j}\big]_\times
$$
which is exactly the term we want to remove. Substituting:
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[R(\omega) M_i^k\big]_\times + Jp_{i,j}\big[p^w_{i,j}\big]_\times - \big[B^w_{i,j}\big]_\times \Big) \delta \omega
$$
The skew map is linear, so the two $Jp_{i,j}$ terms merge, and their arguments
collapse:
$$
R(\omega)M_i^k + p^w_{i,j} = R(\omega)M_i^k + \big(S_j^w - R(\omega)M_i^k - x^w\big) = S_j^w - x^w
$$
The magnet drops out entirely. Name what is left
$$
arm^w_j = S_j^w - x^w
$$
the vector from the knob origin to sensor $j$ in world axes — it depends on the
sensor only, because all three magnets ride the same rigid knob. Then
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[arm^w_j\big]_\times - \big[B^w_{i,j}\big]_\times \Big) \delta \omega
$$

Now summing over magnets is worth something, because $[arm^w_j]_\times$ carries
no magnet index and factors out:
$$
\frac{\partial \hat{B}^w_j}{\partial \omega} = JpSum_j \big[arm^w_j\big]_\times - \big[\hat{B}^w_j\big]_\times
$$

Both pieces are quantities we already have: $JpSum_j$ is shared with the
translation block, and $\hat{B}^w_j$ is the prediction the residual is built
from. The pose Jacobian therefore needs nothing from the magnet model beyond
$Jp$ and the field itself.

One thing to be careful about: $\hat{B}^w_j$ here is the **predicted** field, the
sum over magnets, and not the calibrated reading. The gain never enters, because
$\hat B$ is a prediction of the physical field and the gain sits on the
measurement side of the residual.

### Cross-check through the knob frame

The forward model gave two equivalent expressions for $B^w_{i,j}$, and we used
the world-frame one. The knob-frame one is an independent route to the same
Jacobian, so it is worth walking as a check. Start from
$$
B^w_{i,j} = R(\omega)\, B^k_{i,j} = R(\omega)\, F\big( R(-\omega)(S_j^w - x^w) - M_i^k, \; d_i^k \big)
$$
Here $d_i^k$ and $M_i^k$ are genuinely constant, so $Jd$ cannot appear at all —
but $R(\omega)$ now sits outside $F$, so we owe a product rule instead:
$$
\delta B^w_{i,j} = \underbrace{\delta R(\omega)\, B^k_{i,j}}_{\textrm{the field vector turns}} \;+\; \underbrace{R(\omega)\, \delta B^k_{i,j}}_{\textrm{the sensor moves through the field}}
$$

The first term is immediate:
$$
\delta R(\omega)\, B^k_{i,j} = [\delta \omega]_\times R(\omega) B^k_{i,j} = [\delta \omega]_\times B^w_{i,j} = - \big[B^w_{i,j}\big]_\times \delta \omega
$$

For the second, only the sensor position moves:
$$
\delta S_j^k = \delta R(-\omega)\,(S_j^w - x^w) = - R(-\omega)[\delta \omega]_\times arm^w_j = + R(-\omega)\big[arm^w_j\big]_\times \delta \omega
$$
so, with $Jp^k_{i,j}$ the position derivative evaluated at the knob-frame
arguments,
$$
R(\omega)\, \delta B^k_{i,j} = R(\omega)\, Jp^k_{i,j}\, R(-\omega) \big[arm^w_j\big]_\times \delta \omega
$$

The commutation property closes the gap. Differentiating
$F(Rx, Rd) = R F(x,d)$ w.r.t. $x$ gives $Jp(Rx, Rd)\,R = R\,Jp(x,d)$, i.e.
$$
Jp_{i,j} = R(\omega)\, Jp^k_{i,j}\, R(-\omega)
$$
The position derivative is a tensor, and transporting it between frames is the
usual congruence. Substituting, the two terms add to
$$
\delta B^w_{i,j} = \Big( Jp_{i,j}\big[arm^w_j\big]_\times - \big[B^w_{i,j}\big]_\times \Big) \delta \omega
$$
which is the world-frame result, term for term. The two routes disagree about
*which* derivative of $F$ does the work — the world frame splits it between $Jp$
and $Jd$, the knob frame puts all of it in $Jp$ and a product rule — and agree on
the answer.

### The $3\times6$ block for one sensor

Collecting the two blocks, the derivative of the prediction w.r.t. the pose
$(x^w, \omega)$ is
$$
\frac{\partial \hat{B}^w_j}{\partial (x^w, \omega)} = \begin{pmatrix}
- JpSum_j & \quad JpSum_j \big[arm^w_j\big]_\times - \big[\hat{B}^w_j\big]_\times
\end{pmatrix} \in \R^{3\times6}
$$
and since $\delta r^w_j = -\delta \hat{B}^w_j$, the residual's block is its
negation:
$$
Jr_j = \frac{\partial r^w_j}{\partial (x^w, \omega)} = \begin{pmatrix}
JpSum_j & \quad \big[\hat{B}^w_j\big]_\times - JpSum_j \big[arm^w_j\big]_\times
\end{pmatrix} \in \R^{3\times6}
$$

### The $9\times6$ block for a frame

All three sensors observe one rigid knob, so they share the same six pose
columns. Stacking is trivial assembly, exactly as it was for the residual:
$$
J = \begin{pmatrix}
Jr_0 \\
Jr_1 \\
Jr_2
\end{pmatrix} \in \R^{9\times6}
$$
with rows in the same order as $r$, so that $J$ and $r$ line up row for row.

### What this costs

Per frame, on top of what the forward model already computes:

- **$Jp_{i,j}$ for all nine pairs.** Unavoidable, and the natural thing for the
  magnet model to return alongside the field, since both are read out of the
  same evaluation.
- **No $Jd$ anywhere.** This is the payoff of the substitution above: the pose
  solve never touches the dipole derivative.
- **Three skew matrices $[arm^w_j]_\times$**, one per sensor rather than one per
  pair, because the magnet index dropped out. Building them is free — a skew
  matrix is a rearrangement of three numbers, not arithmetic.
- **Three $3\times3$ products $JpSum_j [arm^w_j]_\times$**, again one per sensor.
  Summing $Jp_{i,j}$ over magnets *before* multiplying is what buys this: nine
  matrix additions are much cheaper than the six extra matrix products we would
  pay by assembling per pair and summing afterwards.
- **$[\hat{B}^w_j]_\times$ and $JpSum_j$ are shared** — the field with the
  residual, the gradient between the translation and rotation blocks.

The structural point is that the sum over magnets is pushed as early as it can
legally go. It is legal because everything between the per-pair field and the
assembled block — the sum, the skew map, and the matrix products — is linear.

---------------

## The full Jacobian, for bundle calibration

Bundle calibration fits the hardware and the poses at the same time, over many
captured frames at once. The parameter split is simple to state:

**Only the pose $(x^w, \omega)$ is per-frame. Every other parameter is shared
across all frames.** The raw measurements $Bm^w_j$ are of course per-frame too,
but those are data, not parameters. Everything else — magnet positions, magnet
dipoles, sensor gains, sensor offsets, sensor positions — describes one physical
device and takes one value for the whole capture.

**TODO (maintainer):** this shared/per-frame split is a fact about the whole
document, not about this section. It belongs at the very top, next to the
notation conventions. Left here for you to place.

That split is the entire reason bundle calibration is worth doing. A single
frame gives 9 equations against 6 pose unknowns and every shared parameter,
which is hopeless. $N$ frames give $9N$ equations against $6N$ pose unknowns
plus a *fixed* number of shared ones, so the shared parameters keep gaining
information as $N$ grows while the poses do not.

Gauge is not treated here.

### The parameters

Per frame, the pose, already handled:

$x^w \in \R^3$, $\omega \in \R^3$ — 6 columns per frame.

Shared, three magnets $i \in \{0,1,2\}$:

$M_i^k \in \R^3$ — magnet position in the knob frame, 9 columns.

$d_i^k \in \R^3$ — magnet dipole in the knob frame, 9 columns. Note this is
the full vector: its direction is the magnet's orientation and its magnitude is
the magnet's strength, fitted jointly rather than split into a tilt and a scale.

Shared, three sensors $j \in \{0,1,2\}$:

$g_j \in \R^3$ — per-axis gain, 9 columns.

$\theta_j \in \R^3$ — per-axis offset, 9 columns.

$S_j^w \in \R^3$ — sensor position in the world frame, 9 columns, usually held
fixed (see below).

We differentiate the same residual as before,
$$
r^w_j = \textrm{diag}(g_j)\, Bm^w_j + \theta_j - \sum_i F\big( S_j^w - R(\omega)M_i^k - x^w, \; R(\omega)d_i^k \big)
$$
now w.r.t. all of it. The two derivative primitives $Jp_{i,j}$ and $Jd_{i,j}$
are the same ones, evaluated at the same arguments; nothing new is asked of the
magnet model.

Below we do **one frame**. Stacking frames is trivial assembly, in the same
sense sensor stacking was — the only thing worth saying about it is where the
blocks land, which is the last subsection.

### Pose — reused unchanged

$$
\frac{\partial r^w_j}{\partial (x^w, \omega)} = Jr_j = \begin{pmatrix}
JpSum_j & \quad \big[\hat{B}^w_j\big]_\times - JpSum_j \big[arm^w_j\big]_\times
\end{pmatrix}
$$

These six columns belong to this frame alone.

### Sensor offset

$\theta_j$ enters the residual once, additively, and only for its own sensor:
$$
\frac{\partial r^w_j}{\partial \theta_j} = I_3, \qquad \frac{\partial r^w_j}{\partial \theta_{j'}} = 0 \;\textrm{ for } j' \neq j
$$

### Sensor gain

$\textrm{diag}(g_j)\,Bm^w_j = \textrm{diag}(Bm^w_j)\,g_j$ — the same product read
the other way round — so
$$
\frac{\partial r^w_j}{\partial g_j} = \textrm{diag}(Bm^w_j), \qquad \frac{\partial r^w_j}{\partial g_{j'}} = 0 \;\textrm{ for } j' \neq j
$$

These are the only two blocks that come from the measurement side of the
residual, and they are the only two that are positive; every block that comes
through $F$ inherits the minus sign in front of the sum. They are also the only
two blocks that do not depend on the parameters at all: for a given frame they
are constants, fixed the moment the frame is captured, and never recomputed as
the solver iterates.

### Magnet position

$M_i^k$ enters only the position argument, and only for its own magnet:
$$
\frac{\partial p^w_{i,j}}{\partial M_i^k} = - R(\omega)
$$
so
$$
\frac{\partial r^w_j}{\partial M_i^k} = + Jp_{i,j}\, R(\omega)
$$

There is no sum over $i$ here — this is a separate $3\times3$ block for each of
the nine (magnet, sensor) pairs. Moving one magnet changes what every sensor
sees, so these columns are dense across sensors, which is exactly why magnet
geometry is well determined by a bundle and a single frame cannot pin it down.

### Magnet dipole

$d_i^k$ enters only the dipole argument:
$$
\frac{\partial d_i^w}{\partial d_i^k} = R(\omega)
$$
so
$$
\frac{\partial r^w_j}{\partial d_i^k} = - Jd_{i,j}\, R(\omega)
$$

This is where $Jd$ finally earns its name. The pose Jacobian could dispose of it
because a rigid rotation moves the sensor and turns the dipole together, and the
commutation identity relates those two effects. Refitting a magnet's dipole
turns it *without* moving anything else, which is not a rigid motion of the
configuration, so no identity rescues us and the honest derivative is required.

We do not need a second table for it. Evaluated at this pair's arguments, the
reconstruction from the properties section reads
$$
Jd_{i,j} = \frac{1}{|d_i^w|^2}\Big( B^w_{i,j}\,(d_i^w)^T - \big(\big[B^w_{i,j}\big]_\times - Jp_{i,j}\big[p^w_{i,j}\big]_\times\big)\big[d_i^w\big]_\times \Big)
$$
and every quantity on the right is already in hand from evaluating the forward
model and its gradient at this pair.

### Sensor position

Included for completeness, because it costs one line:
$$
\frac{\partial r^w_j}{\partial S_j^w} = - JpSum_j
$$
which is exactly the negative of the pose translation block. That is not a
coincidence: only the *relative* position of knob and sensor enters $F$, so
moving the knob one way and the sensor the other way are the same change to
every prediction. Fitting both at once therefore adds three directions the data
cannot separate, per sensor. The sensors sit on a manufactured PCB and are the
best-known geometry in the device, so the usual choice is to hold $S_j^w$ fixed
and let the magnets absorb the placement error.

### One frame's row block

For a single frame the residual is $9\times1$ and the parameters it touches are
the 36 shared columns plus its own 6 pose columns. Writing the shared columns in
the order $(M^k, d^k, g, \theta)$, sensor $j$'s three rows are
$$
\begin{pmatrix}
\underbrace{Jp_{0,j}R(\omega) \;\; Jp_{1,j}R(\omega) \;\; Jp_{2,j}R(\omega)}_{9,\ \textrm{dense}} &
\underbrace{-Jd_{0,j}R(\omega) \;\; -Jd_{1,j}R(\omega) \;\; -Jd_{2,j}R(\omega)}_{9,\ \textrm{dense}} &
\underbrace{\cdots \textrm{diag}(Bm^w_j) \cdots}_{9,\ \textrm{only column block } j} &
\underbrace{\cdots I_3 \cdots}_{9,\ \textrm{only column block } j} &
\underbrace{Jr_j}_{6,\ \textrm{this frame}}
\end{pmatrix}
$$

Stacking the three sensors gives a $9 \times 42$ block for the frame
($9 \times 51$ if sensor positions are fitted). The magnet halves are dense; the
sensor halves are block diagonal, since sensor $j$'s gain and offset are
invisible to the other two sensors.

Over $N$ frames the full matrix is $9N \times (36 + 6N)$, and it has the arrowhead
shape that structure implies: a tall dense column strip for the 36 shared
parameters, and a block-diagonal strip of $9\times6$ pose blocks, one per frame,
with nothing off that diagonal because no frame's pose affects any other frame's
residual.

### What this costs

Per frame, on top of the pose Jacobian:

- **$Jd_{i,j}$ for all nine pairs**, reconstructed rather than tabulated. Each
  reconstruction is two outer-product-shaped $3\times3$ products and a scaling,
  from values already computed.
- **One $R(\omega)$ per frame, right-multiplying eighteen blocks.** Both the
  magnet-position and magnet-dipole blocks end in the same $R(\omega)$, and it
  does not depend on $i$ or $j$. Whether it is cheaper to apply it eighteen times
  or to factor it out of the block and apply it once to a stacked operand is an
  implementation question, but it is the same matrix every time.
- **The gain and offset blocks are free.** $I_3$ is not stored, and
  $\textrm{diag}(Bm^w_j)$ is three numbers copied from the frame's measurement,
  fixed for the life of the frame.
- **$JpSum_j$ is shared** between the pose translation block and the sensor
  position block, when the latter is used at all.

Note what does *not* get summed here. The pose Jacobian could sum over magnets
early because the pose moves all magnets together. Bundle parameters are
per-magnet, so each magnet gets its own columns and the sum stays unsummed —
nine blocks where the pose needed three. That is the real cost of widening: not
the arithmetic in any one block, but that the magnet index survives into the
output.
