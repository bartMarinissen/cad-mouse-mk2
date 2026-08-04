"""Bundle geometry: the calibration parameter vector, the forward model, and
its analytic Jacobian.

This is the layer the calibration solver talks to. Three things live here:

1. **Nominal geometry** - sensor and magnet placement, mirroring
   firmware/include/magnet_model/positions.h.
2. **The flat shared-parameter vector** (`x_shared`) - what is being
   calibrated, grouped so that a hierarchical solve can enable one group at
   a time (see PARAM_GROUPS).
3. **predict_and_jacobians()** - the forward model *and* its exact analytic
   derivative with respect to both the per-frame pose and every shared
   parameter.

Why analytic: the previous version finite-differenced the whole shared
vector per frame, which cost 2*N_SHARED magpylib evaluations *per frame per
Jacobian* - tens of thousands of calls per solver iteration. Every derivative
here is instead the same closed-form chain rule the firmware already uses in
firmware/src/magnet_model/sensor.cpp, on top of the local field gradient from
local_field.py. That makes the Jacobian roughly as cheap as one residual
evaluation, and it is checked against finite differences in
tests/test_jacobian.py (mirroring firmware/test/test_jacobian.cpp).

Frame conventions
-----------------
- World frame: origin at bundle center, +X right, +Y towards USB-C, sensors
  in the z=0 plane. Sensor positions are FIXED, not calibrated - they define
  the world frame, and any real sensor placement error is absorbed by the
  magnet position offsets (the two are related by a per-frame pose anyway).
- Knob frame: origin at the knob's pivot. A frame's pose is (t, rotvec)
  mapping knob -> world.
- Magnet local frame: origin at the magnet's BOTTOM FACE (see local_field.py
  for why), +z along its polarization axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from .local_field import MAGNET_HALF_HEIGHT_MM, local_field_and_gradient

N_MAGNETS = 3
N_SENSORS = 3
N_POSE_PARAMS = 6  # translation (3) + rotation vector (3)

# --- Nominal bundle geometry, mirrors firmware/include/magnet_model/positions.h ---
_SQRT3 = math.sqrt(3.0)
_TRIANGLE_SIDE_MM = 28.58
_TRIANGLE_R_MM = _TRIANGLE_SIDE_MM / _SQRT3
_MAGNET_Z_FROM_PIVOT_MM = 14.0
_MAGNET_REST_DISTANCE_MM = 6.0

SENSOR_POS: NDArray[np.float64] = np.array([
    [0.0, -_TRIANGLE_R_MM, 0.0],                                # sensor 1: bottom center
    [-_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],  # sensor 2: top left
    [_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],   # sensor 3: top right
])

# Knob-frame magnet positions (bottom-face reference, matching positions.h).
MAGNET_POS_NOMINAL_KNOB: NDArray[np.float64] = SENSOR_POS.copy()
MAGNET_POS_NOMINAL_KNOB[:, 2] = -_MAGNET_Z_FROM_PIVOT_MM

# Seed for the per-frame pose solve: the pivot sits this far above the sensor
# plane so each magnet's bottom face is ~_MAGNET_REST_DISTANCE_MM above its
# sensor. Only an initial guess, never assumed exact.
APPROX_REST_T_MM: NDArray[np.float64] = np.array(
    [0.0, 0.0, _MAGNET_Z_FROM_PIVOT_MM + _MAGNET_REST_DISTANCE_MM]
)

# The raw sensors read the opposite sign to the modelled field (the capture
# path streams readUncorrected(), i.e. before any correction). The firmware
# carries this as Config::magnet_gains ~ {-0.96, -1.2, -0.98}; here it is the
# nominal gain, so a *fitted* gain offset of 0 means "exactly -I" and the
# priors below are centred on something physically reachable.
NOMINAL_GAIN_SIGN = -1.0


# --- Shared parameter vector layout -----------------------------------------
#
# Grouped (not interleaved) so a hierarchical solve can free one group at a
# time with a simple boolean mask - see bundle_params.SolveStage.
#
#   magnet_pos   (3 per magnet)  knob-frame bottom-face position offset, mm
#   magnet_tilt  (2 per magnet)  magnet axis tilt, rad. Only 2 DOF: the magnet
#                                is a solid of revolution about its own
#                                polarization axis, so spin about that axis
#                                changes nothing measurable - carrying it
#                                would add an exactly-dead Jacobian column.
#   gain_iso     (1 per sensor)  isotropic gain error (the dominant term)
#   gain_aniso   (2 per sensor)  traceless diagonal, i.e. per-axis sensitivity
#                                spread at fixed overall scale
#   gain_sym     (3 per sensor)  symmetric off-diagonal (cross-axis skew)
#   gain_rot     (3 per sensor)  antisymmetric, i.e. sensor frame misalignment
#
# Gain is assembled as  G_i = NOMINAL_GAIN_SIGN * (I + A_i), so every entry is
# an offset from nominal with expected value 0 - which is what lets the ridge
# priors in bundle_params.py read as "how many prior stddevs from nominal".
PARAM_GROUPS: tuple[tuple[str, int], ...] = (
    ("magnet_pos", 3 * N_MAGNETS),
    ("magnet_tilt", 2 * N_MAGNETS),
    ("gain_iso", 1 * N_SENSORS),
    ("gain_aniso", 2 * N_SENSORS),
    ("gain_sym", 3 * N_SENSORS),
    ("gain_rot", 3 * N_SENSORS),
)

GROUP_SLICES: dict[str, slice] = {}
_offset = 0
for _name, _size in PARAM_GROUPS:
    GROUP_SLICES[_name] = slice(_offset, _offset + _size)
    _offset += _size
N_SHARED_PARAMS = _offset  # 42


def _skew(v: NDArray[np.float64]) -> NDArray[np.float64]:
    """Skew-symmetric matrix (or stack of them) from vector(s) of shape (..., 3)."""
    v = np.asarray(v, dtype=float)
    out = np.zeros(v.shape[:-1] + (3, 3))
    out[..., 0, 1] = -v[..., 2]
    out[..., 0, 2] = v[..., 1]
    out[..., 1, 0] = v[..., 2]
    out[..., 1, 2] = -v[..., 0]
    out[..., 2, 0] = -v[..., 1]
    out[..., 2, 1] = v[..., 0]
    return out


# The 9 gain basis matrices, in the order the parameter groups list them
# (iso, aniso x2, sym x3, rot x3). A_i is the sum of these weighted by the
# fitted parameters, so d(A_i)/d(param_k) is just GAIN_BASIS[k] - which keeps
# the gain Jacobian free of hand-rolled index arithmetic.
def _gain_basis() -> NDArray[np.float64]:
    basis = []
    basis.append(np.eye(3))                                    # iso
    basis.append(np.diag([1.0, 0.0, -1.0]))                    # aniso 0
    basis.append(np.diag([0.0, 1.0, -1.0]))                    # aniso 1
    for a, b in ((0, 1), (0, 2), (1, 2)):                      # sym off-diagonal
        m = np.zeros((3, 3))
        m[a, b] = m[b, a] = 1.0
        basis.append(m)
    for k in range(3):                                         # antisymmetric
        e = np.zeros(3)
        e[k] = 1.0
        basis.append(_skew(e))
    return np.array(basis)


GAIN_BASIS: NDArray[np.float64] = _gain_basis()
# Which gain basis index each gain group's per-sensor entries map to.
_GAIN_GROUP_BASIS: dict[str, tuple[int, ...]] = {
    "gain_iso": (0,),
    "gain_aniso": (1, 2),
    "gain_sym": (3, 4, 5),
    "gain_rot": (6, 7, 8),
}


def so3_left_jacobian(rotvec: NDArray[np.float64]) -> NDArray[np.float64]:
    """Left Jacobian of SO(3): d(exp(phi)) corresponds to a left perturbation
    J_l(phi) @ d(phi).

    The analytic pose/tilt derivatives below are naturally expressed against a
    *left perturbation* of the rotation (which is what the firmware's
    sensor.cpp derivation gives). scipy optimizes over the rotation vector
    itself, so the two are related by this factor. Ignoring it would still
    converge to the same optimum but with degraded steps and a wrong
    covariance - at 15 degrees it is already a ~7% effect.
    """
    rotvec = np.asarray(rotvec, dtype=float)
    theta = np.linalg.norm(rotvec, axis=-1)
    k = _skew(rotvec)
    k2 = k @ k
    small = theta < 1e-8
    theta_safe = np.where(small, 1.0, theta)
    c1 = np.where(small, 0.5, (1.0 - np.cos(theta_safe)) / theta_safe**2)
    c2 = np.where(small, 1.0 / 6.0, (theta_safe - np.sin(theta_safe)) / theta_safe**3)
    eye = np.broadcast_to(np.eye(3), k.shape)
    return eye + c1[..., None, None] * k + c2[..., None, None] * k2


@dataclass(frozen=True)
class BundleGeometry:
    """A concrete bundle geometry: nominal values with calibration offsets applied."""

    magnet_pos_knob: NDArray[np.float64]  # (3, 3) knob-frame bottom-face positions, mm
    magnet_tilt: NDArray[np.float64]      # (3, 3) magnet tilt rotation vectors (z component always 0)
    gain: NDArray[np.float64]             # (3, 3, 3) per-sensor gain matrix, nominal -I

    @property
    def magnet_rotation(self) -> Rotation:
        return Rotation.from_rotvec(self.magnet_tilt)

    def _geometry_terms(self, ts, rotvecs):
        """Shared intermediates for predict() and predict_and_jacobians()."""
        ts = np.atleast_2d(np.asarray(ts, dtype=float))
        rotvecs = np.atleast_2d(np.asarray(rotvecs, dtype=float))
        n = len(ts)

        r_knob = Rotation.from_rotvec(rotvecs).as_matrix()          # (n,3,3)
        r_mag = self.magnet_rotation.as_matrix()                    # (3,3,3)

        # v[n,i] = sensor_i - t_n ; p_knob[n,i] = R^T v
        v = SENSOR_POS[None, :, :] - ts[:, None, :]                 # (n,3,3)
        p_knob = np.einsum("nba,nib->nia", r_knob, v)               # R^T v

        # d[n,i,j] = p_knob[n,i] - m_j ; p_mag = R_j^T d
        d = p_knob[:, :, None, :] - self.magnet_pos_knob[None, None, :, :]   # (n,i,j,3)
        p_mag = np.einsum("jba,nijb->nija", r_mag, d)

        b_loc, j_loc = local_field_and_gradient(p_mag)              # (n,i,j,3), (n,i,j,3,3)

        b_knob = np.einsum("jab,nijb->nija", r_mag, b_loc)          # R_j B_loc
        b_world = np.einsum("nab,nijb->nija", r_knob, b_knob)       # R B_knob
        s = b_world.sum(axis=2)                                     # (n,i,3) summed over magnets
        pred = np.einsum("iab,nib->nia", self.gain, s)
        return dict(n=n, r_knob=r_knob, r_mag=r_mag, v=v, d=d, j_loc=j_loc,
                    b_knob=b_knob, b_world=b_world, s=s,
                    pred=pred.reshape(n, 3 * N_SENSORS), rotvecs=rotvecs)

    def predict(self, ts, rotvecs) -> NDArray[np.float64]:
        """Predicted sensor readings, (n_frames, 9), ordered s1xyz s2xyz s3xyz."""
        return self._geometry_terms(ts, rotvecs)["pred"]

    def predict_and_jacobians(
        self, ts, rotvecs
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """Prediction plus its exact derivatives.

        Returns (pred, j_pose, j_shared) with shapes
        (n, 9), (n, 9, 6) and (n, 9, N_SHARED_PARAMS).

        Each frame's pose block is independent of every other frame's - that
        block structure is what bundle_params.py turns into a sparse Jacobian
        (and what would let a future port eliminate poses frame-by-frame via
        the Schur complement, as covariance_shared() already does).
        """
        g = self._geometry_terms(ts, rotvecs)
        n, r_knob, r_mag = g["n"], g["r_knob"], g["r_mag"]
        v, d, j_loc = g["v"], g["d"], g["j_loc"]
        b_knob, b_world, s = g["b_knob"], g["b_world"], g["s"]

        # M[n,i,j] = R R_j J_loc R_j^T R^T : the field gradient in world coords.
        rrj = np.einsum("nab,jbc->njac", r_knob, r_mag)             # (n,j,3,3)
        m = np.einsum("njab,nijbc,njdc->nijad", rrj, j_loc, rrj)

        # --- pose derivatives (left perturbation, then to rotvec) ---
        # d(field)/dt = -sum_j M ; d(field)/d(delta) = sum_j (M skew(v) - skew(B_world))
        dt = -m.sum(axis=2)                                          # (n,i,3,3)
        skew_v = _skew(v)                                            # (n,i,3,3)
        drot = np.einsum("nijab,nibc->nijac", m, skew_v) - _skew(b_world)
        drot = drot.sum(axis=2)                                      # (n,i,3,3)
        drot = np.einsum("niab,nbc->niac", drot, so3_left_jacobian(g["rotvecs"]))

        gain = self.gain
        j_pose = np.empty((n, N_SENSORS, 3, N_POSE_PARAMS))
        j_pose[..., 0:3] = np.einsum("iab,nibc->niac", gain, dt)
        j_pose[..., 3:6] = np.einsum("iab,nibc->niac", gain, drot)

        j_shared = np.zeros((n, N_SENSORS, 3, N_SHARED_PARAMS))

        # --- magnet position: d(field)/dm_j = -M R ---
        dm = -np.einsum("nijab,nbc->nijac", m, r_knob)               # (n,i,j,3,3)
        dm = np.einsum("iab,nijbc->nijac", gain, dm)
        sl = GROUP_SLICES["magnet_pos"]
        for j in range(N_MAGNETS):
            j_shared[..., sl.start + 3 * j : sl.start + 3 * j + 3] = dm[:, :, j]

        # --- magnet tilt: d(field)/d(eps_j) = M R skew(d) - R skew(B_knob) ---
        mr = np.einsum("nijab,nbc->nijac", m, r_knob)
        dtilt = np.einsum("nijab,nijbc->nijac", mr, _skew(d)) - np.einsum(
            "nab,nijbc->nijac", r_knob, _skew(b_knob)
        )
        dtilt = np.einsum("iab,nijbc->nijac", gain, dtilt)
        sl = GROUP_SLICES["magnet_tilt"]
        for j in range(N_MAGNETS):
            jl = so3_left_jacobian(self.magnet_tilt[j])
            # only the x/y tilt columns are free; spin about the magnet axis is dropped
            j_shared[..., sl.start + 2 * j : sl.start + 2 * j + 2] = np.einsum(
                "niab,bc->niac", dtilt[:, :, j], jl[:, :2]
            )

        # --- gain: pred_i = sign * (I + A_i) s_i, so d/d(param) = sign * basis @ s ---
        for group, basis_idx in _GAIN_GROUP_BASIS.items():
            sl = GROUP_SLICES[group]
            width = len(basis_idx)
            for k, bi in enumerate(basis_idx):
                contrib = NOMINAL_GAIN_SIGN * np.einsum("ab,nib->nia", GAIN_BASIS[bi], s)
                for i in range(N_SENSORS):
                    j_shared[:, i, :, sl.start + width * i + k] = contrib[:, i]

        return (
            g["pred"],
            j_pose.reshape(n, 3 * N_SENSORS, N_POSE_PARAMS),
            j_shared.reshape(n, 3 * N_SENSORS, N_SHARED_PARAMS),
        )


def unpack_shared(x_shared: NDArray[np.float64]) -> BundleGeometry:
    """Build a concrete BundleGeometry from the flat shared-parameter vector.

    The single source of truth for what each index of x_shared means.
    """
    x = np.asarray(x_shared, dtype=float)

    magnet_pos_offset = x[GROUP_SLICES["magnet_pos"]].reshape(N_MAGNETS, 3)
    tilt_xy = x[GROUP_SLICES["magnet_tilt"]].reshape(N_MAGNETS, 2)
    magnet_tilt = np.concatenate([tilt_xy, np.zeros((N_MAGNETS, 1))], axis=1)

    a = np.zeros((N_SENSORS, 3, 3))
    for group, basis_idx in _GAIN_GROUP_BASIS.items():
        vals = x[GROUP_SLICES[group]].reshape(N_SENSORS, len(basis_idx))
        a += np.einsum("ik,kab->iab", vals, GAIN_BASIS[list(basis_idx)])
    gain = NOMINAL_GAIN_SIGN * (np.eye(3)[None, :, :] + a)

    return BundleGeometry(
        magnet_pos_knob=MAGNET_POS_NOMINAL_KNOB + magnet_pos_offset,
        magnet_tilt=magnet_tilt,
        gain=gain,
    )


NOMINAL_GEOMETRY = unpack_shared(np.zeros(N_SHARED_PARAMS))
