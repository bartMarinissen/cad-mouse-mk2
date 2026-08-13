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

// TILT_UNIT_BASIS's projection is simpler still -- no cross-magnet mixing at
// all, just dropping the always-dead spin column. Not worth a named
// constant: `d_magnet_tilt_i.leftCols<2>()` (used directly in the test) IS
// the projection.
//
// NOT reproduced here: the so3_left_jacobian factor bundle_geometry.py
// applies on top of TILT_UNIT_BASIS. That factor exists there because scipy
// parameterizes each magnet's tilt as a rotation vector relative to nominal.
// It only belongs on-device if the bundle solver represents R_mag the same
// way -- if it instead keeps R_mag as a persistent matrix and steps it with
// R_mag = exp(d_eps) * R_mag each iteration (consistent with how
// solve_pose.cpp already treats the pose rotation R), no correction is
// needed and adding one would be the bug. This is a solver design decision,
// not a fact about the Jacobian -- make it explicitly when the solver
// exists, don't inherit whichever answer this comment defaulted to.
