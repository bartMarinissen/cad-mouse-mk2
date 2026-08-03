"""Physical geometry of the 3-magnet / 3-sensor bundle, and the forward
model that predicts sensor readings for a given knob pose.

Nominal geometry mirrors firmware/include/magnet_model/positions.h. The
magnet itself (6mm x 6mm cylinder, ~N42 remanence) mirrors the one built
in field_approximation.ipynb. Both are kept in sync by hand, not code -
if either changes, this needs updating too.

This module also owns the flat calibration-parameter vector (`x_shared`)
that scipy.optimize.least_squares works with: unpack_shared() is the single
place that decides what each index of that vector means, and
one_frame_jacobian()/full_jacobian() differentiate directly against that
same vector (rather than re-deriving per-quantity derivatives), so the
ordering only ever has to be defined once.
"""

from __future__ import annotations

import functools
import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass

import magpylib as magpy
import numpy as np
import scipy.sparse as sp
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

N_MAGNETS = 3
N_POSE_PARAMS = 6  # translation (3) + rotation vector (3)

# --- The physical magnet, from field_approximation.ipynb ---
MAGNET_DIMENSION_MM = (6.0, 6.0)  # (diameter, height)
MAGNET_POLARIZATION_MT = (0.0, 0.0, -600.0)  # ~N42 remanence, along the magnet's own local +z

# --- Nominal bundle geometry, mirrors firmware/include/magnet_model/positions.h ---
_SQRT3 = math.sqrt(3.0)
_TRIANGLE_SIDE_MM = 28.58
_TRIANGLE_R_MM = _TRIANGLE_SIDE_MM / _SQRT3
_MAGNET_Z_FROM_PIVOT_MM = 14.0
_MAGNET_REST_DISTANCE_MM = 6.0

# World frame: origin at bundle center, +X right, +Y towards USB-C, sensors in the z=0 plane.
# Fixed - not a calibration parameter. Manufacturing error here is assumed to
# show up as (and be absorbed by) the magnet position/rotation offsets instead.
SENSOR_POS_NOMINAL: NDArray[np.float64] = np.array([
    [0.0, -_TRIANGLE_R_MM, 0.0],                                # sensor 1: bottom center
    [-_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],  # sensor 2: top left
    [_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],   # sensor 3: top right
])

# Knob frame: magnets assumed to sit directly above their sensor at rest.
MAGNET_POS_NOMINAL_KNOB: NDArray[np.float64] = SENSOR_POS_NOMINAL.copy()
MAGNET_POS_NOMINAL_KNOB[:, 2] = -_MAGNET_Z_FROM_PIVOT_MM

# A reasonable initial guess for the knob's rest pose in world coordinates.
# Not a firmware constant verbatim - derived from the two constants above
# (the knob pivot sits this far above the sensor plane so that, upright,
# each magnet ends up ~_MAGNET_REST_DISTANCE_MM above its sensor). Only
# used to seed the per-frame pose solve, never assumed to be exact.
APPROX_REST_T_MM: NDArray[np.float64] = np.array(
    [0.0, 0.0, _MAGNET_Z_FROM_PIVOT_MM + _MAGNET_REST_DISTANCE_MM]
)

# --- Flat calibration-parameter vector (x_shared) layout ---
#
# 3 per-magnet/sensor-pair blocks (same order as SENSOR_POS_NOMINAL /
# MAGNET_POS_NOMINAL_KNOB), each 11 entries:
#   0:3  magnet position offset (mm)
#   3:5  magnet axis tilt (rad) - only 2 DOF, not a full 3-parameter
#        rotation. The magnet is a uniformly axially-polarized cylinder
#        (a solid of revolution), so rotating it about its own symmetry
#        axis (local z, the polarization direction) changes neither its
#        shape nor its magnetization - predict_field's sensitivity to that
#        component is exactly zero, for every pose, at every parameter
#        value, not just near nominal. Carrying it as a free parameter
#        would add a dead/gauge direction with a near-zero (numerical-noise
#        level) Jacobian column that only the ridge prior anchors, so it's
#        dropped from x_shared entirely rather than merely regularized -
#        see unpack_shared, which pads the implicit 3rd (spin) component
#        back in as a hardcoded 0 before building the Rotation.
#   5:10 gain free entries, upper-triangular order [0,1] [0,2] [1,1] [1,2] [2,2]
#        (gain[0,0] is fixed at 1, lower off-diagonal fixed at 0 - see
#        unpack_shared. Fixing [0,0] gives magnet strength, below, sole
#        ownership of scale, rather than splitting it across two free
#        parameters that can trade off against each other.)
#   10:11 magnet strength offset (dimensionless, multiplies polarization)
N_GAIN_FREE_PER_SENSOR = 5
N_ROTATION_TILT_PARAMS = 2
N_SHARED_PARAMS_PER_PAIR = 3 + N_ROTATION_TILT_PARAMS + N_GAIN_FREE_PER_SENSOR + 1  # = 11
N_SHARED_PARAMS = N_SHARED_PARAMS_PER_PAIR * N_MAGNETS  # = 33


@dataclass(frozen=True)
class BundleGeometry:
    """A concrete bundle geometry (nominal values with calibration offsets applied)."""

    magnet_pos_knob: NDArray[np.float64]  # (3, 3) knob-frame magnet positions, mm
    magnet_rotation: Rotation             # length-3 Rotation: magnet axis offset from the knob frame
    gain: NDArray[np.float64]             # (3, 3, 3) per-sensor 3x3 gain matrix (upper-triangular)
    magnet_strength: NDArray[np.float64]  # (3,) per-magnet multiplier on the nominal polarization

    def _magnets(self, ts: NDArray[np.float64], rotvecs: NDArray[np.float64]) -> list:
        """Build the 3 magnet sources, each with a length-n_frames path."""
        r_knob = Rotation.from_rotvec(rotvecs)
        magnets = []
        for j in range(N_MAGNETS):
            magnet_world_pos = ts + r_knob.apply(self.magnet_pos_knob[j])
            magnet_world_rot = r_knob * self.magnet_rotation[j]
            magnets.append(
                magpy.magnet.Cylinder(
                    polarization=self.magnet_strength[j] * np.array(MAGNET_POLARIZATION_MT),
                    dimension=MAGNET_DIMENSION_MM,
                    position=magnet_world_pos,
                    orientation=magnet_world_rot,
                )
            )
        return magnets

    def predict_fields(self, ts: NDArray[np.float64], rotvecs: NDArray[np.float64]) -> NDArray[np.float64]:
        """Predict the 9 field components (sensor1 xyz, sensor2 xyz, sensor3 xyz) for many knob poses at once.

        ts: (n_frames, 3) knob pivot translations in world coordinates (mm);
        rotvecs: (n_frames, 3) knob orientations as rotation vectors (rad).
        Returns (n_frames, 9).

        Vectorized per magpylib's own guidance (avoid per-frame Python loops):
        each magnet gets one Cylinder object carrying the full per-frame
        path, wrapped in a Collection so magpylib sums all 3 magnets'
        contributions at each sensor in one call - this is what actually
        gives the model cross-magnet interference, not just each sensor's
        nominally-paired magnet.
        """
        n_frames = len(ts)
        collection = magpy.Collection(*self._magnets(ts, rotvecs))
        # magpylib squeezes any length-1 axis (notably the path axis when
        # n_frames == 1), so reshape explicitly rather than trust the
        # returned shape directly.
        raw = collection.getB(SENSOR_POS_NOMINAL).reshape(n_frames, N_MAGNETS, 3)
        gained = np.einsum("sij,nsj->nsi", self.gain, raw)
        return gained.reshape(n_frames, 3 * N_MAGNETS)

    def predict_field(self, t: NDArray[np.float64], rotvec: NDArray[np.float64]) -> NDArray[np.float64]:
        """Single-frame convenience wrapper around predict_fields."""
        return self.predict_fields(t[np.newaxis, :], rotvec[np.newaxis, :])[0]


def unpack_shared(x_shared: NDArray[np.float64]) -> BundleGeometry:
    """Build a concrete BundleGeometry from the flat calibration-parameter vector.

    The single source of truth for x_shared's layout - see the module
    docstring / the comment above N_SHARED_PARAMS_PER_PAIR.
    """
    pairs = x_shared.reshape(N_MAGNETS, N_SHARED_PARAMS_PER_PAIR)
    magnet_pos_offset = pairs[:, 0:3]
    magnet_rotvec_tilt = pairs[:, 3:5]
    gain_free = pairs[:, 5:10]
    strength_offset = pairs[:, 10]

    # Pad the (dropped) spin-about-own-axis component back in as a
    # hardcoded 0 - see the x_shared layout comment above.
    magnet_rotvec_offset = np.concatenate([magnet_rotvec_tilt, np.zeros((N_MAGNETS, 1))], axis=1)

    gain = np.tile(np.eye(3), (N_MAGNETS, 1, 1))
    gain[:, 0, 1] = gain_free[:, 0]
    gain[:, 0, 2] = gain_free[:, 1]
    gain[:, 1, 1] = 1.0 + gain_free[:, 2]
    gain[:, 1, 2] = gain_free[:, 3]
    gain[:, 2, 2] = 1.0 + gain_free[:, 4]
    # gain[:, 0, 0] stays 1 (fixed, not a free parameter); lower
    # off-diagonal stays 0 (from np.eye) - also fixed, not free.

    return BundleGeometry(
        magnet_pos_knob=MAGNET_POS_NOMINAL_KNOB + magnet_pos_offset,
        magnet_rotation=Rotation.from_rotvec(magnet_rotvec_offset),
        gain=gain,
        magnet_strength=1.0 + strength_offset,
    )


def _central_diff_jacobian(
    fn: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    x0: NDArray[np.float64],
    rel_step: float = 1e-6,
) -> NDArray[np.float64]:
    """A small, explicit central-difference Jacobian - not time-critical, so
    no need to reach for scipy's internal (private) numdiff machinery."""
    n = len(x0)
    f0 = fn(x0)
    jac = np.empty((len(f0), n))
    for i in range(n):
        h = rel_step * max(1.0, abs(x0[i]))
        x_plus, x_minus = x0.copy(), x0.copy()
        x_plus[i] += h
        x_minus[i] -= h
        jac[:, i] = (fn(x_plus) - fn(x_minus)) / (2 * h)
    return jac


def one_frame_jacobian(
    t: NDArray[np.float64],
    rotvec: NDArray[np.float64],
    x_shared: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Finite-difference Jacobian of one frame's 9-vector field prediction,
    w.r.t. (a) that frame's own 6 pose params and (b) the full x_shared
    vector. Returned as two separate (9x6, 9xN_SHARED_PARAMS) matrices for
    full_jacobian() to assemble.
    """
    geometry0 = unpack_shared(x_shared)

    def pose_fn(pose: NDArray[np.float64]) -> NDArray[np.float64]:
        return geometry0.predict_field(pose[:3], pose[3:])

    def shared_fn(x: NDArray[np.float64]) -> NDArray[np.float64]:
        return unpack_shared(x).predict_field(t, rotvec)

    pose_jac = _central_diff_jacobian(pose_fn, np.concatenate([t, rotvec]))
    shared_jac = _central_diff_jacobian(shared_fn, x_shared)
    return pose_jac, shared_jac


def _one_frame_jacobian_task(
    pose: tuple[NDArray[np.float64], NDArray[np.float64]],
    x_shared: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    t, rotvec = pose
    return one_frame_jacobian(t, rotvec, x_shared)


def full_jacobian(
    ts: NDArray[np.float64],
    rotvecs: NDArray[np.float64],
    x_shared: NDArray[np.float64],
    *,
    workers: Callable[[Callable, Iterable], Iterable] = map,
) -> sp.spmatrix:
    """Assemble the field-residual Jacobian for all frames at once, as a
    sparse (9*n_frames, N_SHARED_PARAMS + 6*n_frames) matrix: dense in the
    shared-parameter columns, block-diagonal in the per-frame pose columns.

    This is what avoids the O(n_frames^2) blowup of a plain dense
    finite-difference Jacobian (perturbing one frame's pose no longer
    requires re-evaluating every other frame's residual). Each frame's
    local Jacobian is independent of the others, so `workers` (e.g. a
    multiprocessing.Pool's .map) can parallelize across frames.
    """
    n_frames = len(ts)
    task = functools.partial(_one_frame_jacobian_task, x_shared=x_shared)
    blocks = list(workers(task, zip(ts, rotvecs)))

    rows: list[list] = []
    for k, (pose_jac_k, shared_jac_k) in enumerate(blocks):
        row = [shared_jac_k] + [None] * n_frames
        row[1 + k] = pose_jac_k
        rows.append(row)
    return sp.bmat(rows, format="csr")
