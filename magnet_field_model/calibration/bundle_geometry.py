"""The physical bundle geometry and its analytic forward model / Jacobian.

This module owns exactly one thing: `BundleGeometry` - a concrete geometry
(nominal values plus calibration offsets applied) and its two methods,
`predict()` and `predict_and_jacobians()`. It does NOT own the calibration
parameter *layout* - that is `parameterization.py`'s job (see its module
docstring for why that split exists). `BundleGeometry.from_shared()` is a
thin consumer of `parameterization.assemble()`, not a second implementation
of it.

Why the Jacobian is analytic: finite-differencing the whole shared vector per
frame cost 2*N_SHARED magpylib evaluations *per frame per Jacobian* - tens of
thousands of calls per solver iteration. Every derivative here is instead the
same closed-form chain rule the firmware already uses in
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

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial.transform import Rotation

from .local_field import local_field_and_gradient
from .nominal_geometry import (
    APPROX_REST_T_MM,
    MAGNET_POS_NOMINAL_KNOB,
    N_MAGNETS,
    N_POSE_PARAMS,
    N_SENSORS,
    SENSOR_POS,
)
from .parameterization import (
    GAIN_BASIS,
    GAIN_GROUP_BASIS_INDICES,
    GROUP_SLICES,
    MAGNET_POS_BASIS,
    N_SHARED_PARAMS,
    TILT_UNIT_BASIS,
    assemble,
    set_offsets,
)

__all__ = [
    "APPROX_REST_T_MM", "MAGNET_POS_NOMINAL_KNOB", "N_MAGNETS", "N_POSE_PARAMS",
    "N_SENSORS", "SENSOR_POS", "GROUP_SLICES", "N_SHARED_PARAMS", "MAGNET_POLARITY",
    "NOMINAL_GAIN_SIGN", "BundleGeometry", "NOMINAL_GEOMETRY", "magnet_pos_offsets",
    "set_magnet_pos_offsets", "strength_vector", "set_strength_vector", "so3_left_jacobian",
    "PAIRED_ONLY", "ALL_MAGNETS", "SENSOR_MAGNET_COUPLING",
]

# The raw sensors read the opposite sign to the field local_field.py models
# (the capture path streams readUncorrected(), i.e. before any correction).
# The firmware carries that flip in Config::magnet_gains ~ {-0.96, -1.2, -0.98}.
#
# Here it is attributed to the *magnet* instead: the magnets are installed with
# the opposite polarity to the one field_approximation.ipynb assumed, so
# local_field.py stays a byte-for-byte counterpart of the firmware's table
# (polarization -600) and this constant flips it. The payoff is that both
# fitted quantities then read naturally - sensor gain sits near +I and magnet
# strength near +1, instead of a sign hiding inside the gain.
MAGNET_POLARITY = -1.0
NOMINAL_GAIN_SIGN = +1.0

# --- Which magnets each sensor is modelled as seeing -------------------------
#
# (N_SENSORS, N_MAGNETS) weights, applied to the per-(sensor, magnet) local
# field and its gradient at the one point they enter the model.
#
# ALL_MAGNETS is both the physically complete model and the one the firmware
# runs: ForwardModel::evaluate() evaluates every sensor against every magnet,
# its own through the bicubic table and the other two (~28.58mm away) through
# dipole_field(). PAIRED_ONLY is what the firmware used to do, kept because
# tests use it to isolate one magnet's contribution.
#
# The two differ by 1.7-4.5% of the field on the captured runs, growing with
# knob-to-sensor distance: the paired magnet's field falls off fast while the
# far ones barely change, so their share grows as the knob lifts.
#
# This was PAIRED_ONLY for a while, deliberately matching a firmware that
# could not model cross-talk. That cost the absolute field scale: cross-talk
# is what separates magnet *strength* from magnet *distance* (a strength
# change scales the near and far contributions equally, a z-shift changes them
# at very different rates), so without it the two are near-degenerate over the
# ~3mm of heave the hardware gives. See
# test_absolute_strength_needs_cross_magnet_coupling. With the firmware
# modelling the term, the fit no longer has to absorb it into
# gain/offset/strength, and the fitted numbers mean what they say again.
#
# The two models are not identical even under ALL_MAGNETS, and it is worth
# knowing which way: this computes the cross terms with the same exact
# magpylib cylinder solution it uses for the paired magnet, while the firmware
# approximates them as point dipoles to keep them cheap. That approximation is
# 0.14-0.40% of the cross term at the geometry it is used at, i.e. under 0.02%
# of the total field - far below the ~0.3% residual either model achieves, so
# it is not a discrepancy the fit can see. Do not "fix" it by approximating
# here too: the exact solution is free on a PC and this is the reference the
# firmware's approximation gets judged against.
PAIRED_ONLY: NDArray[np.float64] = np.eye(N_SENSORS, N_MAGNETS)
ALL_MAGNETS: NDArray[np.float64] = np.ones((N_SENSORS, N_MAGNETS))

SENSOR_MAGNET_COUPLING: NDArray[np.float64] = ALL_MAGNETS


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
    magnet_tilt: NDArray[np.float64]      # (3, 3) magnet tilt rotation vectors (z always 0)
    magnet_strength: NDArray[np.float64]  # (3,) polarization multiplier, nominal 1
    gain: NDArray[np.float64]             # (3, 3, 3) per-sensor gain matrix, nominal +I
    sensor_offset: NDArray[np.float64]    # (3, 3) per-sensor DC offset in raw units (mT)
    # (3, 3) sensor-magnet visibility weights; see SENSOR_MAGNET_COUPLING.
    # A field rather than a bare module lookup so tests can exercise both
    # models side by side without mutating global state.
    coupling: NDArray[np.float64] = field(
        default_factory=lambda: SENSOR_MAGNET_COUPLING
    )

    @staticmethod
    def from_shared(
        x_shared: NDArray[np.float64],
        coupling: NDArray[np.float64] | None = None,
    ) -> BundleGeometry:
        """Build a concrete BundleGeometry from the flat shared-parameter vector.

        Every offset comes from parameterization.assemble() - this method
        does not itself decide what any index of x_shared means.

        coupling defaults to SENSOR_MAGNET_COUPLING; pass it explicitly only to
        compare the single-magnet and cross-talk models against each other.
        """
        offsets = assemble(x_shared)
        gain = NOMINAL_GAIN_SIGN * (np.eye(3)[None, :, :] + offsets["gain_offset"])
        return BundleGeometry(
            magnet_pos_knob=MAGNET_POS_NOMINAL_KNOB + offsets["magnet_pos_offset"],
            magnet_tilt=offsets["magnet_tilt_offset"],
            magnet_strength=1.0 + offsets["magnet_strength_offset"],
            gain=gain,
            sensor_offset=offsets["sensor_offset"],
            coupling=SENSOR_MAGNET_COUPLING if coupling is None else coupling,
        )

    @property
    def magnet_rotation(self) -> Rotation:
        return Rotation.from_rotvec(self.magnet_tilt)

    def _geometry_terms(self, ts: ArrayLike, rotvecs: ArrayLike) -> dict[str, NDArray[np.float64]]:
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

        # Drop the (sensor, magnet) pairs this model says are not coupled. This
        # is the ONLY place the coupling is applied, and it is deliberately
        # here: everything downstream - b_knob, b_world, b_world_unit, s, and
        # the m gradient tensor in predict_and_jacobians - is linear in b_loc
        # and j_loc, so masking at the source propagates exactly, to the
        # prediction and to every analytic derivative alike, with no second
        # place to keep in sync. See SENSOR_MAGNET_COUPLING.
        b_loc = b_loc * self.coupling[None, :, :, None]
        j_loc = j_loc * self.coupling[None, :, :, None, None]

        # Strength (and the installed polarity) scale the field linearly, so
        # they scale the gradient identically - fold them in here and every
        # downstream derivative stays correct. The unscaled world field is kept
        # because it *is* d(prediction)/d(strength), up to the polarity factor.
        factor = MAGNET_POLARITY * self.magnet_strength             # (3,)
        b_loc = b_loc * factor[None, None, :, None]
        j_loc = j_loc * factor[None, None, :, None, None]

        b_knob = np.einsum("jab,nijb->nija", r_mag, b_loc)          # R_j B_loc
        b_world = np.einsum("nab,nijb->nija", r_knob, b_knob)       # R B_knob
        b_world_unit = b_world / factor[None, None, :, None]
        s = b_world.sum(axis=2)                                     # (n,i,3) summed over magnets
        pred = np.einsum("iab,nib->nia", self.gain, s) + self.sensor_offset[None, :, :]
        return dict(n=n, r_knob=r_knob, r_mag=r_mag, v=v, d=d, j_loc=j_loc,
                    b_knob=b_knob, b_world=b_world, b_world_unit=b_world_unit, s=s,
                    pred=pred.reshape(n, 3 * N_SENSORS), rotvecs=rotvecs)

    def predict(self, ts: ArrayLike, rotvecs: ArrayLike) -> NDArray[np.float64]:
        """Predicted sensor readings, (n_frames, 9), ordered s1xyz s2xyz s3xyz."""
        return np.asarray(self._geometry_terms(ts, rotvecs)["pred"], dtype=np.float64)

    def predict_and_jacobians(
        self, ts: ArrayLike, rotvecs: ArrayLike
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """Prediction plus its exact derivatives.

        Returns (pred, j_pose, j_shared) with shapes
        (n, 9), (n, 9, 6) and (n, 9, N_SHARED_PARAMS).

        Each frame's pose block is independent of every other frame's - that
        block structure is what bundle_params.py turns into a sparse Jacobian.
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

        # --- magnet position: d(field)/dm_j = -M R, then projected onto the
        # gauge-fixed shape basis (the 6 rigid-body directions are dropped
        # rather than fitted - see parameterization.MAGNET_POS_BASIS).
        dm = -np.einsum("nijab,nbc->nijac", m, r_knob)               # (n,i,j,3,3)
        dm = np.einsum("iab,nijbc->nijac", gain, dm)
        dm_raw = np.empty((n, N_SENSORS, 3, 3 * N_MAGNETS))
        for j in range(N_MAGNETS):
            dm_raw[..., 3 * j : 3 * j + 3] = dm[:, :, j]
        j_shared[..., GROUP_SLICES["magnet_pos"]] = np.einsum(
            "nirc,ck->nirk", dm_raw, MAGNET_POS_BASIS
        )

        # --- magnet tilt: d(field)/d(eps_j) = M R skew(d) - R skew(B_knob) ---
        mr = np.einsum("nijab,nbc->nijac", m, r_knob)
        dtilt = np.einsum("nijab,nijbc->nijac", mr, _skew(d)) - np.einsum(
            "nab,nijbc->nijac", r_knob, _skew(b_knob)
        )
        dtilt = np.einsum("iab,nijbc->nijac", gain, dtilt)
        sl = GROUP_SLICES["magnet_tilt"]
        for j in range(N_MAGNETS):
            jl = so3_left_jacobian(self.magnet_tilt[j])
            # TILT_UNIT_BASIS drops the always-dead spin column - jl @ TILT_UNIT_BASIS
            # is exactly jl's first two columns, named instead of sliced positionally.
            j_shared[..., sl.start + 2 * j : sl.start + 2 * j + 2] = np.einsum(
                "niab,bc->niac", dtilt[:, :, j], jl @ TILT_UNIT_BASIS
            )

        # --- magnet strength: linear in the field, so d/d(ds_j) = G polarity B_j.
        # The mean moves all three magnets together; each differential moves one
        # magnet against the third (the triple is traceless by construction).
        dstrength = MAGNET_POLARITY * np.einsum(
            "iab,nijb->nija", gain, g["b_world_unit"]
        )
        j_shared[..., GROUP_SLICES["magnet_strength_mean"].start] = dstrength.sum(axis=2)
        sl = GROUP_SLICES["magnet_strength_diff"]
        for k in range(2):
            j_shared[..., sl.start + k] = dstrength[:, :, k] - dstrength[:, :, 2]

        # --- sensor DC offset: added straight onto the prediction ---
        sl = GROUP_SLICES["sensor_offset"]
        for i in range(N_SENSORS):
            j_shared[:, i, :, sl.start + 3 * i : sl.start + 3 * i + 3] = np.eye(3)

        # --- gain: pred_i = sign * (I + A_i) s_i, so d/d(param) = sign * basis @ s ---
        for group, basis_idx in GAIN_GROUP_BASIS_INDICES.items():
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


def magnet_pos_offsets(x_shared: NDArray[np.float64]) -> NDArray[np.float64]:
    """(3, 3) per-magnet position offsets implied by the 3 shape parameters."""
    return assemble(x_shared)["magnet_pos_offset"]


def set_magnet_pos_offsets(x_shared: NDArray[np.float64], offsets: ArrayLike) -> None:
    """Inverse of magnet_pos_offsets(): see parameterization.set_offsets()."""
    set_offsets(x_shared, "magnet_pos_offset", offsets)


def strength_vector(x_shared: NDArray[np.float64]) -> NDArray[np.float64]:
    """The 3 per-magnet strength *offsets* implied by (mean, diff) parameters."""
    return assemble(x_shared)["magnet_strength_offset"]


def set_strength_vector(x_shared: NDArray[np.float64], strength_offsets: ArrayLike) -> None:
    """Inverse of strength_vector(): see parameterization.set_offsets()."""
    set_offsets(x_shared, "magnet_strength_offset", strength_offsets)


NOMINAL_GEOMETRY = BundleGeometry.from_shared(np.zeros(N_SHARED_PARAMS))
