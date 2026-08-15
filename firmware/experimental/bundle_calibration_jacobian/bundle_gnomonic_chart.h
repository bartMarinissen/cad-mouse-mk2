#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (see README.md).
//
// The 2-parameter chart the bundle solver uses for magnet tilt, replacing
// bundle_magnet_pos_gauge.h's leftCols<2>() projection.
//
// --- What is being parameterized, and why 2 numbers ------------------------
//
// A magnet is a solid of revolution about its own polarization axis, so spin
// about that axis is an exact symmetry of the field (see
// test_bundle_shared_jacobian.cpp, which checks this directly rather than by
// finite difference). What is measurable is therefore not the magnet's
// orientation in SO(3) but only the DIRECTION of its axis -- a point on S^2,
// 2 degrees of freedom, SO(3)/SO(2).
//
// This chart names that point by where the axis LINE crosses a plane one unit
// below the magnet, in the nominal magnet's own frame: the gnomonic (central,
// "pinhole") projection. Nominal is (0,0).
//
// Sign convention, stated once: the polarization axis is local +z and the
// sensor sits at negative local z (Math.md 3.3 -- z_l is always negative), so
// the plane is z_l = -1 and the crossing point of the line through the origin
// along n is (-n_x/n_z, -n_y/n_z). Hence the minus signs in axis_from_tilt()
// below. Tipping the magnet's +z toward +x swings the axis's downward end
// toward -x, so u goes negative -- that is the geometry, not a sign error.
// Choosing the plane on the other side would flip both signs, which is a 180
// degree relabel of the parameter plane and immaterial to an isotropic prior.
//
// --- Why gnomonic rather than two Euler angles -----------------------------
//
// Both are 2-parameter charts on S^2 and neither can be global (S^2 admits no
// global 2-chart). What differs is WHERE the chart degenerates. Spherical /
// Euler angles (polar theta, azimuth phi) degenerate at the pole: the phi
// column scales as sin(theta), so at nominal -- theta = 0, exactly where the
// magnets sit -- the two columns collapse to one. With real tilts being a
// manufacturing tolerance of a degree or two, the azimuth column's
// contribution to the normal equations would be ~sin^2(theta) ~ 1e-3 of the
// polar one, and the ridge prior would swallow it: 2 nominal DOF, ~1 fitted.
//
// The gnomonic chart's derivative at nominal is exactly an orthonormal basis
// of the tangent plane (see the identity noted in chart_jacobian below), and
// its singularity is at theta = 90 degrees, where the axis lies IN the plane
// and the crossing runs to infinity. n_z = 1/sqrt(1+u^2+v^2) > 0 for every
// finite (u,v), so the chart cannot reach it -- it covers the whole open
// hemisphere and a magnet would have to tip 90 degrees to leave it.
//
// --- Why this is exact where leftCols<2>() was only a gauge ----------------
//
// bundle_magnet_pos_gauge.h documents that dropping d_magnet_tilt's third
// column removes the response to eps = e_z, while the direction actually dead
// is R_mag.col(2) -- equal only at zero tilt. chart_jacobian() below builds
// its columns as skew(n_knob) * (something), and skew(n)x is perpendicular to
// n for any x, so both columns are perpendicular to the magnet's CURRENT axis
// by construction, at any tilt. Spin is not projected away here; it is
// unrepresentable.

#include "math3D.h"

// The solver's persistent per-magnet tilt state. Two floats, not a Mat3: a
// spin bug is not possible in a representation with no spin in it. R_mag
// becomes a derived quantity, rebuilt by rotation_from_tilt() once per magnet
// per iteration (the outer loop -- see bundle_shared_jacobian.h's cost note),
// so it also never accumulates orthogonality drift the way a matrix stepped
// in place would.
struct MagnetTilt {
    float u = 0.0f;
    float v = 0.0f;
};

// The magnet's polarization axis, as a unit vector in the NOMINAL magnet's
// local frame. (0,0) gives exactly e_z.
inline Vec3 axis_from_tilt(const MagnetTilt& t) {
    return Vec3(-t.u, -t.v, 1.0f).normalized();
}

// The lift S^2 -> SO(3): builds a full R_mag whose third column is the axis
// above, expressed in the knob frame.
//
// The forward model needs a rotation, not just an axis, so one has to be
// chosen -- and it must be chosen SMOOTHLY in (u,v) or the derivatives below
// are meaningless. This uses the shortest-arc rotation from nominal, via
// Rodrigues built from the cross and dot products directly (no trig, no
// angle extraction). Its only singularity is the antipodal case c = -1, and
// c = n.z() > 0 always here, so it cannot be hit.
//
// Do NOT substitute "rotate about x by a, then about y by b" -- that
// reintroduces an azimuth-dependent spin, which puts a third, unwanted degree
// of freedom back into a 2-parameter state where it shows up as a badly
// conditioned direction rather than an honest error.
inline Mat3 rotation_from_tilt(const Mat3& R_nominal, const MagnetTilt& t) {
    const Vec3 n = axis_from_tilt(t);
    const Vec3 k(-n.y(), n.x(), 0.0f);   // e_z x n
    const float c = n.z();                // e_z . n, > 0 for all finite (u,v)
    const Mat3 K = skew_matrix(k);
    return R_nominal * (Mat3::Identity() + K + (K * K) / (1.0f + c));
}

// Inverse chart: recovers (u,v) from an existing rotation. Needed to seed the
// solver from a stored calibration, and to express "offset from nominal" for
// the ridge prior (SharedNormalEquations::add_prior) without having to invert
// anything at prior time.
inline MagnetTilt tilt_from_rotation(const Mat3& R_nominal, const Mat3& R_mag) {
    const Vec3 n = R_nominal.transpose() * R_mag.col(2);
    return MagnetTilt{ -n.x() / n.z(), -n.y() / n.z() };
}

// d(eps)/d(u,v): the 3x2 that turns evaluate_bundle_jacobian's raw 3x3
// d_magnet_tilt (a derivative against a knob-frame LEFT perturbation eps of
// R_mag) into the 2 columns the solver actually fits.
//
// Derivation. A left perturbation moves the knob-frame axis by
//   dn = eps x n = -skew(n) eps,
// and eps components along n (spin) move it not at all. So for a desired dn
// (necessarily perpendicular to n) the canonical, spin-free choice is
//   eps = n x dn = skew(n) dn,
// which is self-consistent: -skew(n)(skew(n) dn) = dn when n.dn = 0, |n| = 1.
// Any other eps producing the same dn differs only by spin and gives the same
// field, exactly -- so this loses nothing.
//
// Chaining, with n_knob = R_nominal * axis_from_tilt(t):
//   d(eps)/d(u,v) = skew(n_knob) * R_nominal * d(axis_local)/d(u,v)
//
// At (0,0) this is the identity onto the tangent plane -- the two columns come
// out orthonormal -- which is the conditioning property the Euler chart lacks.
inline Eigen::Matrix<float, 3, 2> chart_jacobian(
    const Mat3& R_nominal, const MagnetTilt& t
) {
    const Vec3 w(-t.u, -t.v, 1.0f);
    const float s = w.norm();                  // sqrt(1 + u^2 + v^2)
    const float inv_s = 1.0f / s;
    const float inv_s3 = inv_s * inv_s * inv_s;

    // d(w/|w|)/dp = (dw/dp)/s - w * (ds/dp)/s^2, with ds/du = u/s, ds/dv = v/s
    Eigen::Matrix<float, 3, 2> d_axis_local;
    d_axis_local.col(0) = Vec3(-inv_s, 0.0f, 0.0f) - w * (t.u * inv_s3);
    d_axis_local.col(1) = Vec3(0.0f, -inv_s, 0.0f) - w * (t.v * inv_s3);

    const Vec3 n_knob = R_nominal * (w * inv_s);
    return skew_matrix(n_knob) * (R_nominal * d_axis_local);
}

// The projection itself: raw 3x3 -> the 2 fitted columns. Kept as a named
// function for the same reason project_magnet_pos() is -- so the call site
// reads as "project", and so the physics in evaluate_bundle_jacobian stays
// free of any chart decision. See SCHUR_SOLVER_DESIGN.md on why the physics
// layer deliberately stays 3-DOF.
inline Eigen::Matrix<float, 3, 2> project_magnet_tilt(
    const Mat3& d_magnet_tilt, const Eigen::Matrix<float, 3, 2>& d_eps_d_uv
) {
    return d_magnet_tilt * d_eps_d_uv;
}
