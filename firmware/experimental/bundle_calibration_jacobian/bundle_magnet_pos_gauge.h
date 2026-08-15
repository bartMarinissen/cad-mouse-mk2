#pragma once
// PROTOTYPE / SKETCH -- companion to bundle_shared_jacobian.h.
//
// MAGNET_POS_BASIS: fixed 9x3 constant, pasted from the PC-side SVD
// (magnet_field_model/calibration/parameterization.py's
// _magnet_pos_gauge_basis(), run against this bundle's real nominal
// geometry). Depends only on the nominal triangle layout, not on any
// calibration data, so it's computed once and pasted in here -- same
// pattern as BICUBIC_INTERPOLATION_TABLE. Regenerate these 9 numbers if
// Positions::magnet_knob's nominal layout ever changes.
//
// Row 3*j+c is magnet j's raw coordinate c (x=0,y=1,z=2). Column k is one of
// the bundle's 3 real (gauge-fixed) shape parameters. Every z row is exactly
// zero: the two rigid-tilt gauge directions this basis removes force the
// magnet triangle's plane to stay horizontal, so no free parameter can move
// any magnet's z at all -- see parameterization.py's constraint derivation
// for why, not just that it happens to come out zero.
inline const Eigen::Matrix<float, 9, 3> MAGNET_POS_BASIS = (Eigen::Matrix<float, 9, 3>() <<
    -0.2133f, -0.5149f,  0.1508f,
    -0.3015f, -0.0956f, -0.7528f,
     0.0000f,  0.0000f,  0.0000f,
    -0.4964f,  0.5664f,  0.1268f,
    -0.0340f, -0.3981f,  0.5070f,
     0.0000f,  0.0000f,  0.0000f,
     0.7096f, -0.0516f, -0.2776f,
     0.3354f,  0.4937f,  0.2458f,
     0.0000f,  0.0000f,  0.0000f
).finished();

// Projects sensor i's raw d_magnet_pos (from bundle_shared_jacobian.h) onto
// the bundle's 3 free shape parameters, using magnet i's own 3-row slice of
// MAGNET_POS_BASIS. No cross-magnet sum needed here: each sensor only ever
// sees its own magnet (PAIRED_ONLY coupling, the only mode ForwardModel
// implements), so a shape parameter's effect on the *other* two magnets
// shows up in *their* own sensor's block instead, via their own slice.
inline Eigen::Matrix<float, 3, 3> project_magnet_pos(
    const Mat3& d_magnet_pos_i, int magnet_index /* 0, 1, or 2 */
) {
    return d_magnet_pos_i * MAGNET_POS_BASIS.block<3, 3>(3 * magnet_index, 0);
}

// --- Tilt does NOT get a basis here. See bundle_gnomonic_chart.h. ----------
//
// The PC side's equivalent, TILT_UNIT_BASIS, is [[1,0],[0,1],[0,0]] -- it
// drops the third COORDINATE column, the response to eps = e_z. The direction
// actually dead is R_mag.col(2), the magnet's own current polarization axis:
// the field is invariant under R_mag -> R_mag exp(theta [e_z]_x) (a spin in
// the magnet's LOCAL frame), and R exp(theta [v]_x) = exp(theta [Rv]_x) R
// turns that into a left perturbation along R_mag e_z. Those coincide only at
// zero tilt, so that basis is a nominal-frame gauge choice rather than an
// exact annihilator -- fine in practice at tolerance-sized tilts, but only
// approximately the thing it claims to be.
//
// On-device this is done differently, and exactly: bundle_gnomonic_chart.h
// parameterizes the axis DIRECTION itself, so spin is unrepresentable rather
// than projected away, and its chart_jacobian()'s columns are perpendicular
// to the magnet's current axis at any tilt by construction. That also settles
// the so3_left_jacobian question this comment used to leave open -- with an
// explicit chart there is no convention to reconcile, you differentiate your
// own map, and chart_jacobian() is that derivative.
//
// The nominal-frame basis is kept in the PC fit, so the two sides do differ
// here; see TODO/on-device-calibration.md on what that means for
// transferring RegularizationSigmas.
