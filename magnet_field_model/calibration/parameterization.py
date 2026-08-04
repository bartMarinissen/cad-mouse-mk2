"""The single specification of the calibration parameter vector: what each
index of `x_shared` means, and how it maps onto physical quantities.

This is the direct answer to "how does the vector handed to scipy map to the
calibration parameters we are solving" - before this module existed, that
mapping was split three ways with no single owner: `PARAM_GROUPS`/
`GROUP_SLICES` declared the *layout*, `unpack_shared()` in the geometry
module *interpreted* it via four different ad hoc patterns (a plain reshape,
a custom rigid-body gauge basis, a mean/differential split, a Lie-algebra-
style basis sum), and `RegularizationSigmas.as_vector()` walked the same
groups a third time to attach priors. Nothing enforced that all three agreed.

Here, everything reduces to one idea: `BLOCKS`, an ordered tuple of
`ParamBlock`. Every free scalar the solver sees belongs to exactly one block;
every block says which physical `target` it contributes to and *how*, via a
basis matrix (`target_offset = basis @ x[block's slice]`, then reshaped to
that target's physical shape). `GROUP_SLICES` and `N_SHARED_PARAMS` are
*derived* from `BLOCKS` (cumulative free widths) instead of being a second,
independently-maintained definition of the same layout. `assemble()` is the
one place that turns a flat vector into named physical offsets - every other
consumer (`BundleGeometry.from_shared()`, `RegularizationSigmas`,
`bundle_params.BundleCalibrationProblem`'s masking) reads `BLOCKS` instead of
re-deriving any of this.

What looked like four different assembly patterns are really two, once named:

- a **dense joint basis**, applied once across a whole group - `magnet_pos`
  (the 9x3 rigid-body-gauge-fixed `MAGNET_POS_BASIS`; see its own docstring)
  and `magnet_strength` (mean + differential, which is literally
  `offset = M @ [mean, d0, d1]` for a fixed 3x3 `M` - not special, just not
  named as a basis before);
- a **per-unit replicated basis**, the same small basis applied
  independently to each magnet or sensor (`magnet_tilt`, `sensor_offset`,
  and all three gain sub-groups) - a block-diagonal special case of the same
  idea, built by `_replicate_per_unit()`.

The hot per-frame Jacobian in `bundle_geometry.py` does NOT route through
`assemble()` - a generic per-parameter loop would be slower and more awkward
for a tensor batched over frames/sensors/magnets than the bespoke einsum
contractions there. Instead it imports `MAGNET_POS_BASIS` and `GAIN_BASIS`
directly from here, so the fast path and the generic path are provably using
the same numbers, not two independently-authored copies that could drift.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .nominal_geometry import MAGNET_POS_NOMINAL_KNOB, N_MAGNETS, N_SENSORS


@dataclass(frozen=True)
class ParamBlock:
    """One named group of free scalar parameters and the basis that turns
    them into a physical offset.

    `basis` is (n_raw, n_free): `raw = basis @ x[GROUP_SLICES[name]]`, and
    `raw.reshape(target_shape)` is the physical quantity - an offset from
    nominal, in the units that quantity is naturally expressed in (mm, rad,
    a dimensionless multiplier, ...). Multiple blocks may share a `target`
    (gain's three sub-groups all build one gain-offset matrix per sensor;
    magnet strength's mean and differential both build one 3-vector) - their
    raw contributions are summed before reshaping, in `assemble()`.
    """

    name: str
    target: str
    basis: NDArray[np.float64]
    target_shape: tuple[int, ...]

    def __post_init__(self) -> None:
        n_raw = int(np.prod(self.target_shape))
        assert self.basis.shape[0] == n_raw, (
            f"{self.name}: basis has {self.basis.shape[0]} rows, "
            f"target_shape {self.target_shape} needs {n_raw}"
        )


def _replicate_per_unit(unit_basis: NDArray[np.float64], n_units: int) -> NDArray[np.float64]:
    """Block-diagonal replication of a basis that applies independently and
    identically to each of `n_units` magnets/sensors: kron(I_n_units, unit_basis).

    This is what turns a single small "how one magnet's tilt/one sensor's
    gain works" basis into the full per-unit-replicated block used by a
    whole group's ParamBlock, without writing the replication out by hand
    for each group.
    """
    return np.kron(np.eye(n_units), unit_basis).astype(np.float64)


# --- magnet_pos: the position gauge -----------------------------------------
#
# A global translation or rotation of the magnet trio is exactly cancelled by
# a compensating per-frame pose change - m_j -> Q m_j with R_n -> R_n Q^T
# leaves R_n Q^T Q m_j = R_n m_j untouched - so 6 of the 9 raw magnet
# coordinates carry no information at all. Measured: 6 eigenvalues at machine
# zero in the reduced Hessian, invariant to how many frames are captured; no
# dataset fixes this, since every new frame brings 9 equations but also 6 new
# pose unknowns.
#
# Rather than lean on priors to hold those 6 directions down, they are
# removed from the parameterization outright, by three constraints:
#
#   1. sum of offsets == 0     - fixes the trio's position (3 DOF)
#   2. all z offsets equal     - makes the triangle's plane horizontal,
#                                 fixing pitch and roll (2 DOF). This costs
#                                 nothing: three points are always coplanar,
#                                 so any arrangement can be rotated flat, and
#                                 the plane's tilt is pure gauge.
#   3. zero net yaw moment     - fixes twist (1 DOF), symmetrically in all
#                                 three magnets rather than by pinning one edge.
#
# Constraints 1 and 2 together force every z offset to exactly zero, so the
# three survivors are purely in-plane - the triangle's three side lengths,
# the only part of magnet position that can cause Phantom Tilt.
def _magnet_pos_gauge_basis() -> NDArray[np.float64]:
    constraints = []

    for axis in range(3):                                   # 1. zero mean offset
        row = np.zeros(9)
        row[axis::3] = 1.0
        constraints.append(row)

    for j in range(N_MAGNETS - 1):                          # 2. equal z => plane horizontal
        row = np.zeros(9)
        row[3 * j + 2] = 1.0
        row[3 * (j + 1) + 2] = -1.0
        constraints.append(row)

    row = np.zeros(9)                                       # 3. zero net yaw moment
    for j in range(N_MAGNETS):
        row[3 * j + 0] = -MAGNET_POS_NOMINAL_KNOB[j, 1]
        row[3 * j + 1] = MAGNET_POS_NOMINAL_KNOB[j, 0]
    constraints.append(row)

    # The nullspace of the constraint matrix is the admissible subspace.
    _, _, vt = np.linalg.svd(np.array(constraints))
    basis = vt[len(constraints):].T
    assert basis.shape == (9, 3), f"expected 3 free shape DOF, got {basis.shape}"
    return basis


MAGNET_POS_BASIS: NDArray[np.float64] = _magnet_pos_gauge_basis()


# --- gain: 8 exactly-traceless basis matrices -------------------------------
#
# Deliberately 8, not 9: there is no isotropic/scale basis matrix. Sensor
# gain scale and magnet strength describe the same physical effect from
# opposite ends (a sensor reading 5% high and its magnet being 5% strong
# differ only through cross-talk, ~1% of the signal here), so they are not
# meaningfully separable - freeing both would just leave an arbitrary split
# to the priors. Rather than fit both and transfer scale between them
# afterwards, the gauge is fixed structurally: every one of these 8 matrices
# is exactly traceless, so det(I + A) = 1 - tr(A^2)/2 + det(A) collapses to
# "close to 1, to second order" for any value the free parameters take,
# automatically, with no runtime step - magnet_strength is left to carry all
# of the scale instead, with nothing to compete with it.
def _gain_basis() -> NDArray[np.float64]:
    def skew(v: NDArray[np.float64]) -> NDArray[np.float64]:
        x, y, z = v
        return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])

    basis = [
        np.diag([1.0, 0.0, -1.0]),                          # aniso 0
        np.diag([0.0, 1.0, -1.0]),                           # aniso 1
    ]
    for a, b in ((0, 1), (0, 2), (1, 2)):                    # sym off-diagonal
        m = np.zeros((3, 3))
        m[a, b] = m[b, a] = 1.0
        basis.append(m)
    for k in range(3):                                       # antisymmetric
        e = np.zeros(3)
        e[k] = 1.0
        basis.append(skew(e))

    basis_array = np.array(basis)
    assert np.allclose(np.einsum("kaa->k", basis_array), 0.0), (
        "gain basis must be exactly traceless - that is the whole gauge fix"
    )
    return basis_array


GAIN_BASIS: NDArray[np.float64] = _gain_basis()
# Which of the 8 basis matrices each named gain group draws from - the one
# place this grouping is decided, referenced both here (to build each
# group's ParamBlock) and directly by bundle_geometry.py's hot-path Jacobian.
GAIN_GROUP_BASIS_INDICES: dict[str, tuple[int, ...]] = {
    "gain_aniso": (0, 1),
    "gain_sym": (2, 3, 4),
    "gain_rot": (5, 6, 7),
}


def _gain_group_block(name: str) -> ParamBlock:
    indices = GAIN_GROUP_BASIS_INDICES[name]
    unit_basis = GAIN_BASIS[list(indices)].reshape(len(indices), 9).T  # (9, k)
    return ParamBlock(
        name=name, target="gain_offset",
        basis=_replicate_per_unit(unit_basis, N_SENSORS),
        target_shape=(N_SENSORS, 3, 3),
    )


# --- magnet_tilt: 2 DOF per magnet, z always zero ---------------------------
#
# The magnet is a solid of revolution about its own polarization axis, so
# spin about that axis changes nothing measurable - carrying it as a
# parameter would add an exactly-dead column. Padding the always-zero third
# (z, i.e. spin) component into the basis itself means the assembled offset
# already has the right shape with no separate padding step at the call site.
TILT_UNIT_BASIS: NDArray[np.float64] = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])

# --- magnet_strength: common mode + traceless differential ------------------
#
# Split for their priors, not their math: magnets cut from one batch are
# graded to ~1% of each other (a real belief, kept as a tight prior on the
# differential), while their common absolute remanence is not pinned at all
# (the nominal 600mT figure in local_field.py is a round guess, so the mean
# gets no prior - see priors.py). offset = mean*[1,1,1] + d0*[1,0,-1] +
# d1*[0,1,-1]: any 3-vector splits uniquely into a mean plus a zero-sum
# remainder, so this loses nothing relative to 3 independent strengths.
STRENGTH_MEAN_BASIS: NDArray[np.float64] = np.array([[1.0], [1.0], [1.0]])
STRENGTH_DIFF_BASIS: NDArray[np.float64] = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, -1.0]])


BLOCKS: tuple[ParamBlock, ...] = (
    ParamBlock("magnet_pos", "magnet_pos_offset", MAGNET_POS_BASIS, (N_MAGNETS, 3)),
    ParamBlock("magnet_tilt", "magnet_tilt_offset",
               _replicate_per_unit(TILT_UNIT_BASIS, N_MAGNETS), (N_MAGNETS, 3)),
    ParamBlock("magnet_strength_mean", "magnet_strength_offset", STRENGTH_MEAN_BASIS, (3,)),
    ParamBlock("magnet_strength_diff", "magnet_strength_offset", STRENGTH_DIFF_BASIS, (3,)),
    ParamBlock("sensor_offset", "sensor_offset",
               _replicate_per_unit(np.eye(3), N_SENSORS), (N_SENSORS, 3)),
    _gain_group_block("gain_aniso"),
    _gain_group_block("gain_sym"),
    _gain_group_block("gain_rot"),
)

GROUP_SLICES: dict[str, slice] = {}
_offset = 0
for _block in BLOCKS:
    _n_free = _block.basis.shape[1]
    GROUP_SLICES[_block.name] = slice(_offset, _offset + _n_free)
    _offset += _n_free
N_SHARED_PARAMS = _offset

# Every target's physical shape, taken from its blocks (all blocks sharing a
# target must agree - checked once here rather than trusted).
TARGET_SHAPES: dict[str, tuple[int, ...]] = {}
for _block in BLOCKS:
    prev = TARGET_SHAPES.setdefault(_block.target, _block.target_shape)
    assert prev == _block.target_shape, (
        f"blocks targeting {_block.target!r} disagree on shape: {prev} vs {_block.target_shape}"
    )


def assemble(x_shared: NDArray[np.float64]) -> dict[str, NDArray[np.float64]]:
    """Flat vector -> named physical offsets. The one implementation of
    "what does this vector mean" that everything else builds on."""
    x = np.asarray(x_shared, dtype=float)
    raw: dict[str, NDArray[np.float64]] = {}
    for block in BLOCKS:
        contribution = block.basis @ x[GROUP_SLICES[block.name]]
        raw[block.target] = raw.get(block.target, 0.0) + contribution
    return {target: values.reshape(TARGET_SHAPES[target]) for target, values in raw.items()}


def set_offsets(x_shared: NDArray[np.float64], target: str, values: ArrayLike) -> None:
    """Test/debug convenience, the inverse of assemble(): write a physical
    `values` array for `target` back into x_shared's free parameters, in
    place.

    Least-squares-projects onto whichever block(s) share that target - exact
    whenever the combined basis is square-invertible or has orthonormal
    columns, which covers every block in this package (magnet_pos's gauge
    basis is orthonormal by construction; magnet_strength's mean+diff basis
    is a square invertible 3x3). Any component of `values` outside a block's
    reachable subspace (e.g. a rigid-body component of a magnet_pos offset)
    is silently dropped, which is correct - that component was never
    observable, and dropping it is exactly what choosing a gauge
    representative means.
    """
    blocks = [b for b in BLOCKS if b.target == target]
    combined_basis = np.hstack([b.basis for b in blocks])
    coeffs, *_ = np.linalg.lstsq(
        combined_basis, np.asarray(values, dtype=float).ravel(), rcond=None
    )
    offset = 0
    for block in blocks:
        n = block.basis.shape[1]
        x_shared[GROUP_SLICES[block.name]] = coeffs[offset : offset + n]
        offset += n
