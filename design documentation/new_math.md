# Rewrite of the mathmatics, this time human-based

The core of this work is the forward model for a single sensor.
Everything difficult is downstream of that. So that is where we start.
This part ends at the actual magnet-model (dipole or interpolated).
The magnet-model is generally treated as opaque. In [[magnet-model]] we will go through it. Outside of it, it is a (mostly) black box.

The full forward model for a full snapshot (so for 3 sensor readings given a single knob pose) is then just
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

### Per-snapshot variables
These are the variables that change per snapshot.
A snapshot is a single reading of all 3 sensors, taken with the knob in a fixed pose.
The whole point of motion tracking is to take the sensor measurements of a snapshot, and deduce the knob pose that corresponds best to these measurements.
(We might later index these variables by a snapshot-index $n$)

All other variables are effectively constant for a device as long as it isn't modified.

$x^w \in \R^3$ (mm) The position of the knob origin in the world frame.

$\omega \in \R^3$ (franken-units) the orientation of the knob as a 3d vector. It maps orientations in the knob frame $k$ to the world from $w$

$R(\omega) \in SO(3) \subset \R^{3 \times 3}$. The rotation matrix corresponding to $\omega$. \
$R(\omega) = \exp([\omega]_\times) $

${Bm}^w_j \in \R^3$ (mT) The raw measurement at sensor $j$. This is indisputably an input and cannot be changed by the model or modeling choices.

### Sensor variables (for sensor index j)
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
    - So for a full snapshot it costs 12 applications.
- Going through the world frame does not allow pulling a rotation out. But it does allow pre-computing the two terms that require a matrix multiplication $ M_i^w$ and $d_i^w$ because they do not depend on the sensor $j$.
    - So for a full snapshot it takes only 6 matrix applications

### Residual for one sensor

$r^w_j \in \R^3$ (mT) the residual at sensor $j$ (in the world frame) we want to minimize.

$$
\begin{align*}
r^w_j &= Bc_j^w - \hat{B}_j^w \\
      &= \textrm{diag}(g_j) \; Bm^w_j + \theta_j - \sum_i F\big( S_j^w - R(\omega) M_i^k - x^w, R(\omega) d_i^k \big)

\end{align*}
$$

## Full snapshot forward model
We retain the knob position and orientation $x$ and $\omega$ for all sensor readings in a snapshot.

So the residual for a full snapshot is just the $9\times1$ stacked vector:
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
assembly; this is where we have to take derivatives.
Especially needing differentiate through a rotation whilst keeping the signs correct is challenging.

The two formulations of the forward model are definitionally the same, so we
are free to pick whichever frame makes each derivative easiest, and we do:

- **Translation** is easiest in the **world frame**. There $x^w$ sits in the
  position argument on its own, and the dipole argument does not involve it at
  all.
- **Rotation** is easiest in the **knob frame**. There the magnet is nailed
  down — its position and its dipole are constants — and the only thing that
  moves is the sensor, so the derivative is a product rule with two readable
  terms. In the world frame $R(\omega)$ instead sits in *both* arguments of $F$,
  which is a good deal more work for the same answer.

Neither is a statement about how to *evaluate* anything: the forward model
picked the world frame on cost grounds and keeps it. Both results below are
stated in the world frame; the one line of bridging that requires is given just
before the rotation block.

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
them per snapshot, and all nine are the same derivation.

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

### The two derivatives of the magnet model

$F$ stays opaque, but we need it to be differentiable in both arguments, and we
need a name for each derivative:

$Jp(x, d) \in \R^{3\times3}$ the derivative of $F$ w.r.t. its **position**
argument. Entry $(a,b)$ is $\partial F_a / \partial x_b$. This is the field
gradient at $x$.

$Jd(x, d) \in \R^{3\times3}$ the derivative of $F$ w.r.t. its **dipole**
argument. Entry $(a,b)$ is $\partial F_a / \partial d_b$.

Both are functions of the same two arguments $F$ takes. The pose Jacobian below
uses only $Jp$; $Jd$ first appears in the device Jacobian.

Displacements get a name, in both frames:
$$
p^w_{i,j} = S_j^w - M_i^w
\qquad
p^k_{i,j} = S_j^k - M_i^k
$$
so that $B^w_{i,j} = F(p^w_{i,j}, d_i^w)$ and $B^k_{i,j} = F(p^k_{i,j}, d_i^k)$,
and $p^w_{i,j} = R(\omega)\, p^k_{i,j}$. We abbreviate the derivatives evaluated
at a pair's arguments as
$$
Jp_{i,j} = Jp(p^w_{i,j},\, d_i^w)
\qquad
Jp^k_{i,j} = Jp(p^k_{i,j},\, d_i^k)
$$
and similarly for $Jd$. Same function, different frame's arguments.

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

### Translation block

In the world frame,
$$
B^w_{i,j} = F\big( p^w_{i,j}, \; d_i^w \big)
\qquad\textrm{with}\qquad
p^w_{i,j} = S_j^w - R(\omega) M_i^k - x^w
$$

The orientation is held fixed while we vary $x^w$, so the dipole argument
$d_i^w = R(\omega) d_i^k$ does not move at all, and $x^w$ enters through the
position argument alone — where it appears as a bare subtraction:
$$
\delta p^w_{i,j} = -\, \delta x^w
\qquad\textrm{and}\qquad
\delta d_i^w = 0
$$
so
$$
\delta B^w_{i,j} = Jp_{i,j}\, \delta p^w_{i,j} = -\, Jp_{i,j}\; \delta x^w
$$

Summing over the magnets we get the total field jacobian at sensor $j$:
$$
\boxed{
\frac{\partial \hat{B}^w_j}{\partial x^w} = - \sum_i Jp_{i,j}
}
$$

### Moving the gradient between frames

The rotation block below is derived in the knob frame, so it produces
$Jp^k_{i,j}$, while the result is to be stated in terms of $Jp_{i,j}$. The
commutation property bridges the two in one step.
Differentiate $F(Rx, Rd) = R\,F(x,d)$ with respect to $x$:
$$
Jp(Rx, Rd)\, R = R\, Jp(x,d)
$$
Take $R = R(\omega)$, $x = p^k_{i,j}$, $d = d_i^k$, and use
$R(\omega)p^k_{i,j} = p^w_{i,j}$ and $R(\omega)d_i^k = d_i^w$:
$$
Jp_{i,j} = R(\omega)\; Jp^k_{i,j}\; R(-\omega)
$$

The gradient is a tensor, so transporting it between frames is the usual
congruence. This is the only bridge the derivation needs anywhere.

It is sometimes handier to write the same property as
$$
F(Rx, d) = R\; F(x, R^T d)
$$
which is the statement above with $d$ replaced by $R^T d$ — the same assumption,
rearranged. It says a rotation sitting on the position argument can always be
pulled out in front, at the price of counter-rotating the dipole. Differentiating
this form w.r.t. $x$ gives the same congruence, and it is the form to reach for
whenever only the position argument carries a rotation.

### Rotation block

Here we switch to the knob frame,
$$
B^w_{i,j} = R(\omega)\, B^k_{i,j} = R(\omega)\, F\big( S_j^k - M_i^k, \; d_i^k \big)
\qquad\textrm{with}\qquad
S_j^k = R(-\omega)\,\big(S_j^w - x^w\big)
$$

Name the vector from the knob origin to the sensor, since everything below is
written in terms of it:
$$
arm^w_j = S_j^w - x^w
$$
It carries a sensor index only — all three magnets ride the same rigid knob.

Now $R(\omega)$ appears in two places — once outside $F$, rotating the result,
and once inside $S_j^k$, moving the sensor. Neither $M_i^k$ nor $d_i^k$ moves,
because both are fixed in the knob frame. The product rule gives one term for
each:
$$
\delta B^w_{i,j} = \underbrace{\delta R(\omega)\, B^k_{i,j}}_{\textrm{the field vector turns}} \;+\; \underbrace{R(\omega)\, \delta B^k_{i,j}}_{\textrm{the sensor moves through the field}}
$$

**The field vector turns.** The knob-frame field $B^k_{i,j}$ is unchanged; only
the rotation carrying it into the world frame moves:
$$
\delta R(\omega)\, B^k_{i,j} = [\delta \omega]_\times R(\omega) B^k_{i,j} = [\delta \omega]_\times B^w_{i,j} = - \big[B^w_{i,j}\big]_\times \delta \omega
$$

**The sensor moves through the field.** In the knob frame the field is a fixed
object and the sensor slides through it:
$$
\delta S_j^k = \delta R(-\omega)\, arm^w_j = - R(-\omega)[\delta \omega]_\times arm^w_j = + R(-\omega)\big[arm^w_j\big]_\times \delta \omega
$$
so, converting the resulting field change into the world frame,
$$
R(\omega)\, \delta B^k_{i,j} = R(\omega)\, Jp^k_{i,j}\, \delta S_j^k = \underbrace{R(\omega)\, Jp^k_{i,j}\, R(-\omega)}_{Jp_{i,j}} \big[arm^w_j\big]_\times \delta \omega
$$

Adding the two:
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[arm^w_j\big]_\times - \big[B^w_{i,j}\big]_\times \Big) \delta \omega
$$

The magnet index appears only in $Jp_{i,j}$ and $B^w_{i,j}$, so summing over
magnets factors cleanly:
$$
\boxed{
\frac{\partial \hat{B}^w_j}{\partial \omega} = \left(\sum_i Jp_{i,j}\right) \big[arm^w_j\big]_\times - \big[\hat{B}^w_j\big]_\times
}
$$

Both pieces are already in hand: $\sum_i Jp_{i,j}$ is shared with the translation block,
and $\hat{B}^w_j$ is the prediction the residual is built from.

One thing to be careful about: $\hat{B}^w_j$ here is the **predicted** field, the
sum over magnets, and not the calibrated reading. The gain never enters, because
$\hat B$ is a prediction of the physical field and the gain sits on the
measurement side of the residual.

Appendix B does the same derivative through the world frame, where the dipole
argument does move and $Jd$ has to be carried. It lands on this same expression.

### The $3\times6$ block for one sensor

Collecting the two blocks, the derivative of the prediction w.r.t. the pose
$(x^w, \omega)$ is
$$
\frac{\partial \hat{B}^w_j}{\partial (x^w, \omega)} = \begin{pmatrix}
- \sum_i Jp_{i,j} & \quad \left(\sum_i Jp_{i,j}\right) \big[arm^w_j\big]_\times - \big[\hat{B}^w_j\big]_\times
\end{pmatrix} \in \R^{3\times6}
$$
and since $\delta r^w_j = -\delta \hat{B}^w_j$, the residual's block is its
negation:
$$
\boxed{
Jr_j = \frac{\partial r^w_j}{\partial (x^w, \omega)} = \begin{pmatrix}
\sum_i Jp_{i,j} & \quad \big[\hat{B}^w_j\big]_\times - \left(\sum_i Jp_{i,j}\right) \big[arm^w_j\big]_\times
\end{pmatrix} \in \R^{3\times6}
}
$$

### The $9\times6$ block for a snapshot

All three sensors in a snapshot observe one rigid knob, so they share the same
six pose columns. Stacking is trivial assembly, exactly as it was for the residual:
$$
J = \begin{pmatrix}
Jr_0 \\
Jr_1 \\
Jr_2
\end{pmatrix} \in \R^{9\times6}
$$
with rows in the same order as $r$, so that $J$ and $r$ line up row for row.

### What this costs

Per snapshot, on top of what the forward model already computes:

- **$Jp_{i,j}$ for all nine pairs.** Unavoidable, and the natural thing to read
  out of the same evaluation that produces the field.
- **$Jd$ does not appear.** Varying $x^w$ leaves the dipole argument untouched,
  and in the knob frame the dipole is a constant, so neither block ever has
  anything to differentiate it against. This is visible directly in both
  derivations rather than being something that cancels later.
- **Three skew matrices $[arm^w_j]_\times$**, one per sensor rather than one per
  pair, since the magnet index never enters $arm^w_j$. Building them is free — a
  skew matrix is a rearrangement of three numbers, not arithmetic.
- **Three $3\times3$ products $\sum_i Jp_{i,j} [arm^w_j]_\times$**, again one per sensor.
  Summing $Jp_{i,j}$ over magnets *before* multiplying is what buys this: nine
  matrix additions are much cheaper than the six extra matrix products we would
  pay by assembling per pair and summing afterwards.
- **$[\hat{B}^w_j]_\times$ and $\sum_i Jp_{i,j}$ are shared** — the field with the
  residual, the gradient between the translation and rotation blocks.

The structural point is that the sum over magnets is pushed as early as it can
legally go. It is legal because everything between the per-pair field and the
assembled block — the sum, the skew map, and the matrix products — is linear.

---------------

## Jacobian w.r.t. the device parameters

The previous section varied the knob pose and held the device fixed. Here we do
the opposite.

Recall that **only the pose $(x^w, \omega)$ is a per-snapshot variable.** The
raw measurements $Bm^w_j$ are of course per-snapshot too, but those are data,
not parameters. Every other parameter is shared across all snapshots: a set of
magnet positions, magnet dipoles, sensor gains, sensor offsets and sensor
positions describes one physical device and takes one value for the whole
capture. That split is why we speak of per-snapshot variables and device
variables; device variables are also called shared variables, because they apply
to all snapshots.

The reason we want their derivatives is bundle calibration, which fits the device
and the poses at the same time over many captured snapshots at once. It is again
a least-squares problem, and it requires the Jacobian of the residual w.r.t. all
of the device parameters. Hence this work here. We will speak no further of
bundle calibration in this section, and focus on the Jacobian instead.

### The device vector $D$

Give the shared parameters a name and a fixed order, so that "which derivative
goes in which columns" has an answer:
$$
D = \big(\;
M_0^k \;\; M_1^k \;\; M_2^k
\;\big|\;
d_0^k \;\; d_1^k \;\; d_2^k
\;\big|\;
g_0 \;\; g_1 \;\; g_2
\;\big|\;
\theta_0 \;\; \theta_1 \;\; \theta_2
\;\big)^T \in \R^{36}
$$

Twelve entries, each itself a 3-vector, so 36 numbers describing one physical
device:

$M_i^k$ — magnet position in the knob frame.

$d_i^k$ — magnet dipole in the knob frame. The full vector: its direction is
the magnet's orientation and its magnitude is the magnet's strength, fitted
jointly rather than split into a tilt and a scale.

$g_j$ — per-axis sensor gain.

$\theta_j$ — per-axis sensor offset.

Sensor positions $S_j^w$ are not in $D$. They can be added as a fifth group of
nine, taking it to $\R^{45}$; the derivative is given below along with the
reason they are normally held fixed instead.

### Where each derivative goes

The row block for sensor $j$ follows exactly the layout of $D$:
$$
\frac{\partial r^w_j}{\partial D} = \left(\;
\frac{\partial r^w_j}{\partial M_0^k}
\;\;
\frac{\partial r^w_j}{\partial M_1^k}
\;\;
\frac{\partial r^w_j}{\partial M_2^k}
\;\middle|\;
\frac{\partial r^w_j}{\partial d_0^k}
\;\;
\frac{\partial r^w_j}{\partial d_1^k}
\;\;
\frac{\partial r^w_j}{\partial d_2^k}
\;\middle|\;
\frac{\partial r^w_j}{\partial g_0}
\;\;
\frac{\partial r^w_j}{\partial g_1}
\;\;
\frac{\partial r^w_j}{\partial g_2}
\;\middle|\;
\frac{\partial r^w_j}{\partial \theta_0}
\;\;
\frac{\partial r^w_j}{\partial \theta_1}
\;\;
\frac{\partial r^w_j}{\partial \theta_2}
\;\right)
$$

Twelve slots, each a $3\times3$ block, giving $\R^{3\times36}$ per sensor. The
rest of this section fills the slots in; the layout above is the only place you
need to look to know where a block lands.

We differentiate the same residual as before,
$$
r^w_j = \textrm{diag}(g_j)\, Bm^w_j + \theta_j - \sum_i F\big( S_j^w - R(\omega)M_i^k - x^w, \; R(\omega)d_i^k \big)
$$
now w.r.t. all of $D$. We don't treat the snapshot-variables (i.e. the Pose) here. That was already done in [[#Jacobian w.r.t. the knob pose]].

In this case we use the world-frame formulation. This turns out to be the nicest form to calculate the derivatives.
Its only $R(\omega)$ that works nicer in the knob frame. That is because it occurs on both arguments to $F$.

### Magnet position

$M_i^k$ enters only the position argument, and only for its own magnet:
$$
\frac{\partial p^w_{i,j}}{\partial M_i^k} = - R(\omega)
$$
so by simple application of the chain rule:
$$
\boxed{
\frac{\partial r^w_j}{\partial M_i^k} = + Jp_{i,j}\, R(\omega)
}
$$

There is no sum over $i$ here — this is a separate $3\times3$ block for each of
the nine (magnet, sensor) pairs. Moving one magnet changes what every sensor
sees, so these columns are dense across sensors, which is exactly why magnet
geometry is well determined by a bundle and a single snapshot cannot pin it down.

### Magnet dipole

$d_i^k$ enters only the dipole argument:
$$
\frac{\partial d_i^w}{\partial d_i^k} = R(\omega)
$$
so by simple application of the chain rule:
$$
\boxed{
\frac{\partial r^w_j}{\partial d_i^k} = - Jd_{i,j}\, R(\omega)
}
$$

This is where $Jd$ earns its name. The pose Jacobian never needed it, because in
the knob frame the dipole is a constant. Refitting a magnet's dipole changes
that constant, so the honest derivative is required. **Note: if the magnet model
does not supply $Jd$ directly, it can be computed from $Jp$ and the field
instead;** Appendix A gives the construction.

### Sensor gain

$\textrm{diag}(g_j)\,Bm^w_j = \textrm{diag}(Bm^w_j)\,g_j$ — the same product read
the other way round — so
$$
\boxed{
\frac{\partial r^w_j}{\partial g_j} = \textrm{diag}(Bm^w_j),
\qquad
\frac{\partial r^w_j}{\partial g_{j'}} = 0 \;\textrm{ for } j' \neq j
}
$$

### Sensor offset

$\theta_j$ enters the residual once, additively, and only for its own sensor:
$$
\boxed{
\frac{\partial r^w_j}{\partial \theta_j} = I_3,
\qquad
\frac{\partial r^w_j}{\partial \theta_{j'}} = 0 \;\textrm{ for } j' \neq j
}
$$

These two are the only blocks that come from the measurement side of the
residual, and the only two that are positive; every block that comes through $F$
inherits the minus sign in front of the sum. They are also the only two that do
not depend on the parameters at all: for a given snapshot they are constants,
fixed the moment the snapshot is captured, and never recomputed as the solver
iterates.

### Sensor position (non-critical)

The sensors sit on a manufactured PCB and are therefore quite accurately placed.
Hence we consider these quite fixed.
One might consider calibrating out errors on the sensor positions (specifically errors that can't be produced by a rigid transformation of the nominal positions). But we likely will not.

We still include it for completeness, because it costs one line:
$$
\boxed{
\frac{\partial r^w_j}{\partial S_j^w} = - \sum_i Jp_{i,j}
}
$$

### The assembled row block

Dropping each result into the slot the layout gave it, the device Jacobian for
sensor $j$ is
$$
\boxed{
\begin{aligned}
\frac{\partial r^w_j}{\partial D} = \Big(\;
& Jp_{0,j} R(\omega) \;\; Jp_{1,j} R(\omega) \;\; Jp_{2,j} R(\omega)
\;\Big|\;
{-}Jd_{0,j} R(\omega) \;\; {-}Jd_{1,j} R(\omega) \;\; {-}Jd_{2,j} R(\omega) \\[4pt]
&\Big|\;
\underbrace{0 \cdots \textrm{diag}(Bm^w_j) \cdots 0}_{\textrm{slot } j}
\;\Big|\;
\underbrace{0 \cdots I_3 \cdots 0}_{\textrm{slot } j}
\;\Big)
\end{aligned}
}
$$

Twelve $3\times3$ slots, so $\R^{3\times36}$, in the order $D$ fixed.

The two halves have opposite shapes, and that is the whole structure of the
matrix. Every magnet reaches every sensor, so the six magnet slots are dense and
carry both indices. A sensor's gain and offset are invisible to the other two
sensors, so of those six slots only the two belonging to sensor $j$ survive — the
other four are identically zero, which is why the layout has twelve slots but a
row only ever has eight live ones.

Stacking the three sensors gives $\R^{9\times36}$ for the snapshot, with the
magnet columns dense down all three row blocks and the gain and offset columns
block diagonal.

### What this costs

Per snapshot, the Jacobian of the residual w.r.t. the device parameters requires:

- **$Jd_{i,j}$ for all nine pairs**, which the pose solve never needed. This is
  the one genuinely new quantity bundle calibration asks for.
- **One $R(\omega)$ per snapshot, right-multiplying eighteen blocks.** Both the
  magnet-position and magnet-dipole blocks end in the same $R(\omega)$, and it
  does not depend on $i$ or $j$. Whether it is cheaper to apply it eighteen times
  or to factor it out of the block and apply it once to a stacked operand is an
  implementation question, but it is the same matrix every time.
- **The gain and offset blocks are free.** $I_3$ is not stored, and
  $\textrm{diag}(Bm^w_j)$ is three numbers copied from the snapshot's
  measurement, fixed for the life of the snapshot.
- **$\sum_i Jp_{i,j}$ is shared** between the pose translation block and the sensor
  position block, when the latter is used at all.

Note what does *not* get summed here. The pose Jacobian could sum over magnets
early because the pose moves all magnets together. The entries of $D$ are
per-magnet, so each magnet gets its own columns and the sum stays unsummed —
nine blocks where the pose needed three. That is the real cost of widening: not
the arithmetic in any one block, but that the magnet index survives into the
output.

---------------

## Appendix A: computing $Jd$ from $Jp$

A magnet model that supplies $Jp$ but not $Jd$ is not stuck. The properties
asserted above the barrier are statements about *all* $x$ and $d$, so they can
be differentiated, and doing so pins $Jd$ down completely.

**From rotation-commutation.** Take $F(Rx, Rd) = R\,F(x,d)$ with
$R = I + [w]_\times$ for infinitesimal $w$, and expand both sides to first order.
On the left the arguments move by $[w]_\times x$ and $[w]_\times d$:
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

**From magnitude-linearity.** Differentiate $F(x, a\,d) = a\,F(x,d)$ with
respect to $a$ and set $a = 1$:
$$
Jd(x,d)\; d = F(x,d)
$$
$F$ is homogeneous of degree one in $d$, so this is just Euler's theorem.

**Putting them together.** Split an arbitrary $u \in \R^3$ into its component
along $d$ and its component perpendicular to $d$:
$$
u = \underbrace{\frac{d\,(d \cdot u)}{|d|^2}}_{\textrm{along } d} \;+\; u_\perp
$$
Euler's identity handles the first part directly. For the second, note that
$[d]_\times[d]_\times u_\perp = d \times (d \times u_\perp) = d\,(d\cdot u_\perp) - |d|^2 u_\perp = -|d|^2 u_\perp$,
so $u_\perp = -[d]_\times [d]_\times u_\perp / |d|^2$, and since
$[d]_\times u_\perp = [d]_\times u$ (the along-$d$ part is annihilated), we get
$u_\perp = [d]_\times w$ with $w = -[d]_\times u/|d|^2$. That puts $u_\perp$
inside the range of $[d]_\times$, which is precisely where the commutation
identity says what $Jd$ does. Assembling:
$$
Jd(x,d) = \frac{1}{|d|^2}\Big( F(x,d)\, d^T \;-\; \big([F(x,d)]_\times - Jp(x,d)\,[x]_\times\big)[d]_\times \Big)
$$

Everything on the right is the field, its gradient, and the geometry — all of
which are in hand wherever $Jd$ is wanted. Evaluated at a pair's arguments this
reads
$$
Jd_{i,j} = \frac{1}{|d_i^w|^2}\Big( B^w_{i,j}\,(d_i^w)^T - \big(\big[B^w_{i,j}\big]_\times - Jp_{i,j}\big[p^w_{i,j}\big]_\times\big)\big[d_i^w\big]_\times \Big)
$$

## Appendix B: the rotation derivative through the world frame

The main text derives the pose rotation block in the knob frame. The world-frame
route is an independent derivation of the same quantity, and walking it is a
check on the result. It is also the route the translation block already takes,
so this is the one place the two blocks part company. Start from
$$
B^w_{i,j} = F\big( S_j^w - R(\omega) M_i^k - x^w, \; R(\omega) d_i^k \big)
$$
Here $R(\omega)$ appears **twice**, and both occurrences vary — so unlike the
knob-frame route, this one has to carry $Jd$.

**The position argument.** $S_j^w$, $M_i^k$ and $x^w$ are constant under a
rotation variation, so
$$
\delta p^w_{i,j} = -\,\delta R(\omega)\, M_i^k = -[\delta \omega]_\times R(\omega) M_i^k = +\big[R(\omega) M_i^k\big]_\times \delta \omega
$$
where $R(\omega) M_i^k = M_i^w - x^w$ is the magnet's offset from the knob
origin, in world axes.

**The dipole argument.**
$$
\delta d_i^w = \delta R(\omega)\, d_i^k = [\delta \omega]_\times d_i^w = -\big[d_i^w\big]_\times \delta \omega
$$

Together:
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[R(\omega) M_i^k\big]_\times \;-\; Jd_{i,j} \big[d_i^w\big]_\times \Big)\, \delta \omega
$$

Now apply Appendix A's identity at this pair's arguments,
$x = p^w_{i,j}$ and $d = d_i^w$:
$$
Jd_{i,j}\big[d_i^w\big]_\times = \big[B^w_{i,j}\big]_\times - Jp_{i,j}\big[p^w_{i,j}\big]_\times
$$
which removes $Jd$ again:
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[R(\omega) M_i^k\big]_\times + Jp_{i,j}\big[p^w_{i,j}\big]_\times - \big[B^w_{i,j}\big]_\times \Big) \delta \omega
$$
The skew map is linear, so the two $Jp_{i,j}$ terms merge and their arguments
collapse:
$$
R(\omega)M_i^k + p^w_{i,j} = R(\omega)M_i^k + \big(S_j^w - R(\omega)M_i^k - x^w\big) = S_j^w - x^w = arm^w_j
$$
leaving
$$
\delta B^w_{i,j} = \Big( Jp_{i,j} \big[arm^w_j\big]_\times - \big[B^w_{i,j}\big]_\times \Big) \delta \omega
$$
which is the main text's result, term for term.

The two routes disagree about *which* derivative of $F$ does the work — the knob
frame puts all of it in $Jp$ and a product rule, the world frame splits it
between $Jp$ and $Jd$ and then needs an identity to put it back together — and
agree on the answer. That is also the concrete reason the main text takes the
rotation through the knob frame while keeping the translation in the world
frame: each block is derived wherever $F$ has the fewest moving arguments.
