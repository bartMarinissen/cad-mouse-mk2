"""The axisymmetric field of one magnet, in that magnet's own local frame.

This is the innermost layer of the calibration forward model, and the direct
Python counterpart of firmware/src/magnet_model/magnet_local_model.cpp: given
a point in the magnet's local frame, return the field there *and* the 3x3
gradient of that field, exploiting the fact that a uniformly axially-polarized
cylinder is a solid of revolution (so the field only depends on (r, z), and
the full 3D gradient can be rebuilt from four (r, z) partials).

Frame convention - the one thing most worth getting right here:

    The local origin is the magnet's BOTTOM FACE, not its center.

That is the convention field_approximation.ipynb baked into the firmware's
bicubic table (it places the cylinder at z=+3 so the bottom face lands on
z=0, then samples z in [-20, -0.5] below it), and therefore the convention
firmware/include/magnet_model/positions.h's magnet positions are expressed
in. magpylib instead positions a Cylinder by its center, so
`_CENTER_OFFSET_MM` is added when the source is built - see MAGNET_HALF_HEIGHT_MM.
"""

from __future__ import annotations

import magpylib as magpy
import numpy as np
from numpy.typing import NDArray

# --- The physical magnet, mirroring field_approximation.ipynb ---
MAGNET_DIAMETER_MM = 6.0
MAGNET_HEIGHT_MM = 6.0
MAGNET_HALF_HEIGHT_MM = MAGNET_HEIGHT_MM / 2.0
MAGNET_DIMENSION_MM = (MAGNET_DIAMETER_MM, MAGNET_HEIGHT_MM)

# Polarization along the magnet's own local -z, matching the notebook. The
# measured hardware reads the opposite sign, which is carried by the nominal
# sensor gain (-I, see bundle_geometry.NOMINAL_GAIN_SIGN) rather than by
# flipping this - keeping this identical to the notebook is what keeps the
# Python model and the firmware's generated table describing the same magnet.
MAGNET_POLARIZATION_MT = (0.0, 0.0, -600.0)

# Step used for the (r, z) partials. The underlying field is analytic and
# smooth away from the magnet surface, so a plain central difference at this
# step is accurate to ~1e-9 relative in float64 - far below any model error.
_FD_STEP_MM = 1e-4

# Below this radius the cylindrical decomposition is singular and the on-axis
# limit is used instead - mirrors the EPSILON branch in magnet_local_model.cpp.
_R_EPSILON_MM = 1e-6


def _make_source(strength: float = 1.0) -> magpy.magnet.Cylinder:
    """A single magnet at the canonical local pose: bottom face at the origin."""
    return magpy.magnet.Cylinder(
        polarization=strength * np.asarray(MAGNET_POLARIZATION_MT),
        dimension=MAGNET_DIMENSION_MM,
        position=(0.0, 0.0, MAGNET_HALF_HEIGHT_MM),
    )


_SOURCE = _make_source()


def _b_rz(
    r: NDArray[np.float64], z: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """(Br, Bz) at cylindrical coordinates (r, z), evaluated on the +x half-plane.

    Axisymmetry means the field at (r, 0, z) has By == 0 and Bx == Br, so one
    magpylib call on the half-plane gives both components for any azimuth.

    Negative r is accepted and handled as the analytic continuation through
    the axis: Br is odd in r, Bz is even. That matters for the central
    differences in local_field_and_gradient() - clamping r-h to 0 instead
    would silently halve dBr/dr for any point within one step of the axis.
    """
    sign = np.where(r < 0.0, -1.0, 1.0)
    pts = np.stack([np.abs(r), np.zeros_like(r), z], axis=-1)
    b = _SOURCE.getB(pts.reshape(-1, 3)).reshape(*r.shape, 3)
    return sign * b[..., 0], b[..., 2]


def local_field_and_gradient(
    p: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Field and its 3x3 spatial gradient at local-frame points `p`.

    p: (..., 3) points in the magnet's local frame (origin = bottom face,
    +z = the magnet's symmetry/polarization axis).

    Returns (B, J) with shapes (..., 3) and (..., 3, 3), where
    J[..., a, b] = dB_a / dp_b.

    The decomposition mirrors MagnetModel::evaluate in the firmware: evaluate
    (Br, Bz) and their (r, z) partials, then rotate that 2D result back out to
    3D using the azimuth of p.
    """
    p = np.asarray(p, dtype=float)
    px, py, pz = p[..., 0], p[..., 1], p[..., 2]
    r = np.hypot(px, py)

    h = _FD_STEP_MM
    # One batched evaluation for the value and all four partials.
    br, bz = _b_rz(r, pz)
    br_rp, bz_rp = _b_rz(r + h, pz)
    br_rm, bz_rm = _b_rz(r - h, pz)
    br_zp, bz_zp = _b_rz(r, pz + h)
    br_zm, bz_zm = _b_rz(r, pz - h)

    dbr_dr = (br_rp - br_rm) / (2 * h)
    dbz_dr = (bz_rp - bz_rm) / (2 * h)
    dbr_dz = (br_zp - br_zm) / (2 * h)
    dbz_dz = (bz_zp - bz_zm) / (2 * h)

    on_axis = r < _R_EPSILON_MM
    r_safe = np.where(on_axis, 1.0, r)
    cx = np.where(on_axis, 0.0, px / r_safe)
    cy = np.where(on_axis, 0.0, py / r_safe)
    br_over_r = np.where(on_axis, dbr_dr, br / r_safe)

    b = np.stack([br * cx, br * cy, bz], axis=-1)
    # On the axis Br vanishes linearly in r, so the field there is purely
    # axial and the transverse response is isotropic with slope dBr/dr.
    b = np.where(on_axis[..., None], np.stack([px * dbr_dr, py * dbr_dr, bz], axis=-1), b)

    cx2, cy2, cxcy = cx * cx, cy * cy, cx * cy
    br_dr_minus_br_r = dbr_dr - br_over_r

    j = np.empty(p.shape[:-1] + (3, 3))
    j[..., 0, 0] = dbr_dr * cx2 + br_over_r * cy2
    j[..., 0, 1] = br_dr_minus_br_r * cxcy
    j[..., 0, 2] = dbr_dz * cx
    j[..., 1, 0] = br_dr_minus_br_r * cxcy
    j[..., 1, 1] = dbr_dr * cy2 + br_over_r * cx2
    j[..., 1, 2] = dbr_dz * cy
    j[..., 2, 0] = dbz_dr * cx
    j[..., 2, 1] = dbz_dr * cy
    j[..., 2, 2] = dbz_dz

    if np.any(on_axis):
        axis_j = np.zeros_like(j)
        axis_j[..., 0, 0] = dbr_dr
        axis_j[..., 1, 1] = dbr_dr
        axis_j[..., 2, 2] = dbz_dz
        j = np.where(on_axis[..., None, None], axis_j, j)

    return b, j
