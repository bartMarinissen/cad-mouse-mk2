// test_jacobians.cpp
//
// Finite-difference validation of the analytic Jacobians used in the
// magnetic 6DOF pose solver:
//   1. BicubicField    - d_dr / d_dz returned by evaluate()
//   2. MagnetModel      - J_local = dB_l/dv_l
//   3. ForwardModel     - full 9x6 J (translation block + rotation block)
//
// ---------------------------------------------------------------------
// ASSUMPTIONS / THINGS YOU NEED TO CHECK BEFORE TRUSTING RESULTS
// ---------------------------------------------------------------------
//
//  FD_STEP_LINEAR / FD_STEP_ANGULAR assume position units of mm and
//  angle units of radians. If your state vector uses meters, scale
//  FD_STEP_LINEAR down accordingly (see comment at declaration).
// ---------------------------------------------------------------------

#include <unity.h>
#include "math3D.h"
#include "magnet_model/BicubicField.h"
#include "magnet_model/magnet_local_model.h"   // declares CALCULATED_BICUBIC_FIELD + MagnetModel
#include "magnet_model/forward_model.h"        // declares ForwardModel  (adjust filename if different)
#include "magnet_model/positions.h"            // declares Positions:: sensor_i_world / Magnet_i_knob (adjust filename if different)

// ======================================================================
// Config: fill in with real values
// ======================================================================

// Step sizes for central differences.
// Optimal FD step for float32 central differences is roughly
// cbrt(machine_eps) ~ 5e-3 in relative terms. Pick these relative to the
// physical scale of your problem (magnet is 6mm, so mm-scale steps of
// 1e-3 mm are ~1/6000 of the part size - reasonable).
static constexpr float FD_STEP_LINEAR  = 5.0e-4f;   // mm (or your length unit)
static constexpr float FD_STEP_ANGULAR = 5.0e-4f;   // radians

// (r, z) points to probe the BicubicField / MagnetModel derivatives at.
// read BICUBIC_ORIGIN / BICUBIC_FAR in magnet_model_table.h, for the actual
// domain. This file is generated. 
// z is always negative -- the sensor plane sits below
// the magnet. The points below are chosen to sit well inside the current domain, 
// with margin >> FD_STEP_LINEAR, and deliberately cluster in the region real knob
// poses actually reach rather than spanning the whole table.
struct RZSample { float r; float z; };
static constexpr RZSample BICUBIC_TEST_POINTS[] = {
    { 1.0f, -1.0f },
    { 3.0f, -6.0f },
    { 5.0f, -1.0f },
    { 0.5f, -11.0f },
    // On the r=0 symmetry axis and just inside the first cell: exercises
    // the axis_patch mirror stencil in BicubicField::evaluate (and, via
    // the central-difference d_dr probe at r=0, the graceful handling of
    // a slightly negative r from the FD step landing just past the axis).
    { 0.0f, -6.0f },
    { 0.05f, -6.0f },
};

// PCB / knob geometry, taken from Positions:: rather than placeholders.
// NOTE: sensor_i_world and Magnet_i_knob use identical formulas in the
// header you sent, which means at rest the local vector v_l between a
// magnet and its own sensor is the 6mm z-standoff (sensor plane sits
// 6mm below the magnet plane) plus whatever t.x/t.y and rotation you
// apply.
static const Vec3 SENSOR_POS[3] = {
    Positions::sensor_1_world,
    Positions::sensor_2_world,
    Positions::sensor_3_world,
};
static const Vec3 MAGNET_LOCAL[3] = {
    Positions::Magnet_1_knob,
    Positions::Magnet_2_knob,
    Positions::Magnet_3_knob,
};

// Base pose used for the ForwardModel tests.
// x/y chosen so r = sqrt(t.x^2 + t.y^2) ~ 2.5mm at rest (comfortably
// inside [0,6], away from both 0 and the outer edge).
// z = 6.0f is the real standoff: at rest the sensor plane sits 6mm
// below the magnet plane.
static const Vec3 BASE_T(2.0f, -1.5f, 6.0f);

// NOTE: physically, the two frames are rotationally ALIGNED at rest -
// R=Identity, with only the 6mm z-standoff as an offset. The non-zero
// axis below is a synthetic test pose, not an attempt to model rest.
// It's kept non-identity deliberately: the analytic Jacobian needs to
// be correct for any R the solver encounters while tracking, not just
// R=Identity, and testing only at Identity risks masking a bug that
// only shows up once R^T actually does something (e.g. a transpose or
// sign error in how R feeds into the local-frame conversion). Kept
// small purely so the resulting v_l for all three magnet/sensor pairs
// stays inside the field's r<=6, -12<=z<=-0.5 domain.
static const Vec3 BASE_ROTATION_AXIS(0.05f, -0.03f, 0.02f);

// ======================================================================
// Helpers
// ======================================================================



// Max relative error over a matrix, column by column (as Vec3s), generic size.
template <int ROWS, int COLS>
static float max_rel_error_mat(const Eigen::Matrix<float, ROWS, COLS>& A,
                                const Eigen::Matrix<float, ROWS, COLS>& N,
                                float floor_ = 1.0e-5f) {
    // || J_analytic - J_numeric ||_F
    float error_norm = (A - N).norm();
    
    // || J_analytic ||_F
    float numeric_norm = N.norm();

    // Prevent division by zero if the target matrix is exactly zero
    if (numeric_norm < 1e-8f) {
        return error_norm; // Fallback to absolute error
    }
    
    return error_norm / numeric_norm;
}

// Max relative error over a 3-vector.
static float max_rel_error(const Vec3& a, const Vec3& n, float floor_ = 1.0e-5f) {
    return max_rel_error_mat<3, 1>(a, n);
}

// Exact SO(3) exponential map (Rodrigues' formula), used to perturb R.
// This matters: R_new = exp([w]_x) R_old is the actual definition used
// to derive J_rot, so the FD test must use the exact exponential, not
// the first-order approximation (I + [w]_x) R, or the FD estimate picks
// up an O(h) contamination from the approximation itself (rather than
// being a clean O(h^2) central-difference estimate).
static Mat3 exp_so3(const Vec3& w) {
    float theta = w.norm();
    Mat3 K = skew_matrix(w);
    if (theta < 1.0e-8f) {
        // Small-angle fallback (also avoids 0/0); accurate to O(theta^2).
        return Mat3::Identity() + K + 0.5f * (K * K);
    }
    float s = std::sin(theta) / theta;
    float c = (1.0f - std::cos(theta)) / (theta * theta);
    return Mat3::Identity() + s * K + c * (K * K);
}

// ======================================================================
// 1. BicubicField: check d_dr / d_dz against central differences of
//    evaluate() itself. This validates the interpolator's own analytic
//    partials are self-consistent with its own values, independent of
//    everything downstream.
// ======================================================================

static void check_bicubic_point(float r0, float z0) {
    Mat2 jac0;
    CALCULATED_BICUBIC_FIELD.evaluate(r0, z0, jac0);
    const Vec2 ddr0 = jac0.col(0);
    const Vec2 ddz0 = jac0.col(1);

    Mat2 dummy_jac;
    Vec2 val_rp = CALCULATED_BICUBIC_FIELD.evaluate(r0 + FD_STEP_LINEAR, z0, dummy_jac);
    Vec2 val_rm = CALCULATED_BICUBIC_FIELD.evaluate(r0 - FD_STEP_LINEAR, z0, dummy_jac);
    Vec2 ddr_num = (val_rp - val_rm) / (2.0f * FD_STEP_LINEAR);

    Vec2 val_zp = CALCULATED_BICUBIC_FIELD.evaluate(r0, z0 + FD_STEP_LINEAR, dummy_jac);
    Vec2 val_zm = CALCULATED_BICUBIC_FIELD.evaluate(r0, z0 - FD_STEP_LINEAR, dummy_jac);
    Vec2 ddz_num = (val_zp - val_zm) / (2.0f * FD_STEP_LINEAR);

    char msg[128];
    float e_dr = max_rel_error_mat(ddr0, ddr_num);
    snprintf(msg, sizeof(msg), "d_dr mismatch at r=%.3f z=%.3f (rel err %.5f)", r0, z0, e_dr);
    TEST_ASSERT_TRUE_MESSAGE(e_dr < 0.01f, msg);

    float e_dz = max_rel_error_mat(ddz0, ddz_num);
    snprintf(msg, sizeof(msg), "d_dz mismatch at r=%.3f z=%.3f (rel err %.5f)", r0, z0, e_dz);
    TEST_ASSERT_TRUE_MESSAGE(e_dz < 0.01f, msg);
}

void test_bicubic_field_derivatives(void) {
    for (const auto& p : BICUBIC_TEST_POINTS) {
        check_bicubic_point(p.r, p.z);
    }
}

// ======================================================================
// 2. MagnetModel: check J_local = dB_l/dv_l against central differences
//    of MagnetModel::evaluate() over v_l.
// ======================================================================
static void check_magnet_model_at(const MagnetModel& model, const Vec3& v_l) {
    Mat3 J_analytic;
    Vec3 B0 = model.evaluate(v_l, J_analytic);
    (void)B0;

    Mat3 J_numeric;
    for (int i = 0; i < 3; ++i) {
        Vec3 dv = Vec3::Zero();
        dv[i] = FD_STEP_LINEAR;

        Mat3 J_dummy;
        Vec3 Bp = model.evaluate(v_l + dv, J_dummy);
        Vec3 Bm = model.evaluate(v_l - dv, J_dummy);
        J_numeric.col(i) = (Bp - Bm) / (2.0f * FD_STEP_LINEAR);
    }

    // Assuming you switched to norm_rel_error, otherwise use max_rel_error_mat
    float e = max_rel_error_mat(J_analytic, J_numeric); 

    // Buffer expanded to easily fit 18 floats plus formatting
    char msg[1024]; 
    snprintf(msg, sizeof(msg), 
             "J_local mismatch at v_l=(%.3f,%.3f,%.3f) rel err %.5f\n"
             "J_analytic:\n"
             "  %9.5f %9.5f %9.5f\n"
             "  %9.5f %9.5f %9.5f\n"
             "  %9.5f %9.5f %9.5f\n"
             "J_numeric:\n"
             "  %9.5f %9.5f %9.5f\n"
             "  %9.5f %9.5f %9.5f\n"
             "  %9.5f %9.5f %9.5f\n",
             v_l.x(), v_l.y(), v_l.z(), e,
             J_analytic(0,0), J_analytic(0,1), J_analytic(0,2),
             J_analytic(1,0), J_analytic(1,1), J_analytic(1,2),
             J_analytic(2,0), J_analytic(2,1), J_analytic(2,2),
             J_numeric(0,0), J_numeric(0,1), J_numeric(0,2),
             J_numeric(1,0), J_numeric(1,1), J_numeric(1,2),
             J_numeric(2,0), J_numeric(2,1), J_numeric(2,2));
             
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);
}

void test_magnet_model_jacobian_generic(void) {
    MagnetModel model(CALCULATED_BICUBIC_FIELD, Vec3::Zero());
    // Generic points away from r=0 (in the local frame v_l = [x_l,y_l,z_l]).
    // z_l must stay in [-12,-0.5] (z=0 is NOT valid - it's the boundary
    // BICUBIC_FAR sits at -0.5, i.e. sensor plane is below the magnet).
    check_magnet_model_at(model, Vec3(2.0f,  0.0f, -1.0f));
    check_magnet_model_at(model, Vec3(1.4f,  1.4f, -3.0f));
    check_magnet_model_at(model, Vec3(0.0f,  3.0f, -6.0f));
    check_magnet_model_at(model, Vec3(-2.0f, -2.0f, -8.0f));
}

void test_magnet_model_jacobian_at_origin(void) {
    // r = sqrt(x_l^2 + y_l^2) = 0 exercises the L'Hopital branch in
    // J_local's top-left 2x2 block. Perturbing x_l or y_l off zero
    // gives r = FD_STEP_LINEAR > 0 on both sides (never crosses back
    // through the singularity), so central differences are well-defined
    // even though the *base* point requires the L'Hopital limit.
    // z_l chosen well away from the z=-0.5 domain edge for margin.
    MagnetModel model(CALCULATED_BICUBIC_FIELD, Vec3::Zero());
    check_magnet_model_at(model, Vec3(0.0f, 0.0f, -1.0f));
    check_magnet_model_at(model, Vec3(0.0f, 0.0f, -6.0f));
}

void test_magnet_strength_scales_field_and_jacobian(void) {
    // magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT multiplies the six
    // cylindrical quantities inside evaluate() rather than the assembled
    // outputs, on the argument that the field enters everything downstream
    // linearly. That argument is exactly what this asserts: B and J must both
    // come out scaled by that ratio, with no residual shape change. If someone
    // later scales only B, or scales after the r->0 L'Hopital branch, this
    // catches it.
    //
    // Expressed as ratios (not raw mT) so the assertion below reads directly
    // as "B_s should be ratio * B_unit" regardless of what
    // BICUBIC_FIELD_REFERENCE_MT itself happens to be.
    const float ratios[] = { 0.87f, 1.0f, 1.23f };
    const Vec3 probes[] = {
        Vec3( 1.4f,  1.4f, -3.0f),
        Vec3( 0.0f,  3.0f, -6.0f),
        Vec3(-2.0f, -2.0f, -8.0f),
        Vec3( 0.0f,  0.0f, -6.0f),   // the r = 0 branch
    };

    MagnetModel unit(CALCULATED_BICUBIC_FIELD, Vec3::Zero());
    char msg[192];

    for (const Vec3& p : probes) {
        Mat3 J_unit;
        const Vec3 B_unit = unit.evaluate(p, J_unit);

        for (float ratio : ratios) {
            MagnetModel scaled(CALCULATED_BICUBIC_FIELD, Vec3::Zero(), Mat3::Identity(),
                                ratio * BICUBIC_FIELD_REFERENCE_MT);
            Mat3 J_s;
            const Vec3 B_s = scaled.evaluate(p, J_s);

            const float eB = max_rel_error_mat<3, 1>(B_s, (ratio * B_unit).eval());
            const float eJ = max_rel_error_mat<3, 3>(J_s, (ratio * J_unit).eval());

            snprintf(msg, sizeof(msg),
                     "strength ratio %.2f (%.0f mT) at p=(%.2f, %.2f, %.2f): B err %.2e, J err %.2e",
                     ratio, ratio * BICUBIC_FIELD_REFERENCE_MT, p[0], p[1], p[2], eB, eJ);
            TEST_ASSERT_TRUE_MESSAGE(eB < 1e-5f && eJ < 1e-5f, msg);
        }
    }
}

// ======================================================================
// 3. dipole_field: the far-field model used for the two magnets a sensor
//    is NOT paired with. Same finite-difference treatment as the
//    interpolated path -- it is a hand-derived Jacobian like every other
//    link in the chain, and fails just as silently.
// ======================================================================

// Displacements at the geometry this model is actually used at: magnets sit a
// triangle side apart (Positions::triangle_sidelength_mm = 28.58), a few mm to
// a couple of cm above the sensor plane. Deliberately NOT probed near the
// source -- dipole_field has no r->0 branch because nothing ever calls it
// there, and probing close would test an approximation the design never
// relies on.
static const Vec3 DIPOLE_TEST_POINTS[] = {
    Vec3( 28.58f,   0.0f,  -6.0f),
    Vec3( 28.58f,   0.0f,  -9.0f),
    Vec3( 20.0f,   20.6f, -15.0f),   // generic azimuth, no zero component
    Vec3(  0.0f,   28.58f, -26.0f),
    Vec3(-15.0f,   24.3f,  -3.0f),   // asymmetric
};

// A moment that is not axis-aligned, so a term that happens to vanish for
// m = (0, 0, -M) cannot hide. The real caller passes the magnet's own axis,
// but the formula is general and the test should exercise it as such.
static const Vec3 DIPOLE_TEST_MOMENT(0.21f * DIPOLE_MOMENT_AT_REFERENCE_MT_MM3,
                                     -0.34f * DIPOLE_MOMENT_AT_REFERENCE_MT_MM3,
                                     -0.91f * DIPOLE_MOMENT_AT_REFERENCE_MT_MM3);

static void check_dipole_at(const Vec3& m, const Vec3& r) {
    Mat3 J_analytic;
    dipole_field(m, r, J_analytic);

    // Column j is dB/dr_j by central difference.
    Mat3 J_numeric;
    for (int j = 0; j < 3; ++j) {
        Vec3 dr = Vec3::Zero();
        dr[j] = FD_STEP_LINEAR;
        Mat3 dummy;
        const Vec3 B_plus  = dipole_field(m, (r + dr).eval(), dummy);
        const Vec3 B_minus = dipole_field(m, (r - dr).eval(), dummy);
        J_numeric.col(j) = (B_plus - B_minus) / (2.0f * FD_STEP_LINEAR);
    }

    char msg[192];
    const float e = max_rel_error_mat<3, 3>(J_analytic, J_numeric);
    snprintf(msg, sizeof(msg),
             "dipole J mismatch at r=(%.2f, %.2f, %.2f), |r|=%.2f (rel err %.5f)",
             r[0], r[1], r[2], r.norm(), e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);

    // Symmetry is not incidental: the implementation computes six entries and
    // mirrors three, which is only valid because the field is curl-free and J
    // is therefore minus the Hessian of a scalar potential. Asserted so that
    // an asymmetric edit fails here rather than producing a plausible-looking
    // solver that drifts.
    const float asym = (J_analytic - J_analytic.transpose()).norm() / J_analytic.norm();
    snprintf(msg, sizeof(msg),
             "dipole J not symmetric at r=(%.2f, %.2f, %.2f): rel asymmetry %.3e",
             r[0], r[1], r[2], asym);
    TEST_ASSERT_TRUE_MESSAGE(asym < 1e-6f, msg);
}

void test_dipole_field_jacobian(void) {
    for (const Vec3& r : DIPOLE_TEST_POINTS) {
        check_dipole_at(DIPOLE_TEST_MOMENT, r);
    }
}

void test_dipole_matches_the_magnet_the_table_models(void) {
    // The two models have to describe the SAME magnet: the table is generated
    // from a 6x6mm cylinder at BICUBIC_FIELD_REFERENCE_MT, and the dipole is
    // supposed to be that cylinder's far-field limit. Nothing else in the
    // suite would notice a moment that is off by a constant factor, or a
    // dipole placed at the magnet's bottom face instead of its centre -- both
    // produce a smooth, self-consistent, finite-difference-clean field that is
    // simply the wrong size.
    //
    // The check is against MagnetModel::evaluate at the far edge of the
    // table's domain, where both models are valid at once. Agreement there is
    // limited by the dipole approximation itself (the cylinder is not a point
    // at 10mm), so the tolerance is percent-scale on purpose -- this is a
    // units-and-placement check, not a precision one. A wrong 1/mu0 factor
    // would show up here as a factor of ~8e5, and the bottom-face-vs-centre
    // error as tens of percent.
    MagnetModel magnet(CALCULATED_BICUBIC_FIELD, Vec3::Zero());

    // On the magnet's axis, at the bottom of the table's z range. Far enough
    // out for the dipole limit to be close, still inside the interpolated
    // domain. Local frame: origin at the bottom face, +z along polarization.
    const Vec3 v_l(0.0f, 0.0f, BICUBIC_ORIGIN.z);

    Mat3 J_table;
    const Vec3 B_table = magnet.evaluate(v_l, J_table);

    // Polarization is along local -z, so the moment vector is -|m| * z_hat.
    const Vec3 m(0.0f, 0.0f, -magnet.moment_mT_mm3);
    // Displacement from the DIPOLE (at the centre) to the field point, which
    // is what makes MAGNET_HALF_HEIGHT_MM load-bearing here.
    const Vec3 r = v_l - Vec3(0.0f, 0.0f, MAGNET_HALF_HEIGHT_MM);

    Mat3 J_dipole;
    const Vec3 B_dipole = dipole_field(m, r, J_dipole);

    char msg[224];
    const float e = max_rel_error_mat<3, 1>(B_dipole, B_table);
    snprintf(msg, sizeof(msg),
             "dipole vs table at z=%.1f: table (%.4f, %.4f, %.4f), dipole (%.4f, %.4f, %.4f), rel err %.4f",
             v_l[2], B_table[0], B_table[1], B_table[2],
             B_dipole[0], B_dipole[1], B_dipole[2], e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.05f, msg);
}

void test_cross_magnet_terms_are_actually_present(void) {
    // Everything else here is a self-consistency check: the finite-difference
    // tests compare the model against its own derivative, so they would pass
    // just as happily if the cross-magnet contribution were silently zero.
    // TODO/resolved/cross-magnet-interference.md measured that contribution at
    // 1.7-4.5% of the field on captured data, so this asserts it is present
    // and of that order.
    //
    // Paired-only is obtained by zeroing the cross magnets' moments rather
    // than by a second code path: dipole_field returns zero field and zero
    // gradient for m = 0, so this is the same arithmetic with the term
    // switched off, not an independent reimplementation to disagree with.
    MagnetModel magnets[3] = {
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[0]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[1]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[2]),
    };
    VirtualSensor sensors[3] = {
        VirtualSensor(SENSOR_POS[0]),
        VirtualSensor(SENSOR_POS[1]),
        VirtualSensor(SENSOR_POS[2]),
    };
    ForwardModel fm(sensors, magnets);

    const Mat3 R = Mat3::Identity();
    char msg[224];

    // Across the knob's heave range: the share grows with lift, because the
    // paired magnet's field falls off fast while the cross magnets, a fixed
    // triangle side away, barely change.
    const float standoffs[] = { 4.3f, 6.0f, 8.0f };

    for (float standoff : standoffs) {
        const Vec3 t(0.0f, 0.0f, Positions::magnet_z_pos_from_pivot + standoff);

        Eigen::Matrix<float, 9, 1> B_all;
        Eigen::Matrix<float, 9, 6> J_unused;
        fm.evaluate(t, R, B_all, J_unused);

        // The same sensors, with the cross magnets present but inert.
        MagnetPlacement placements[3] = {
            magnets[0].place(t, R), magnets[1].place(t, R), magnets[2].place(t, R),
        };
        MagnetPlacement inert[3] = { placements[0], placements[1], placements[2] };
        for (int j = 0; j < 3; ++j) inert[j].moment_world = Vec3::Zero();

        Eigen::Matrix<float, 9, 1> B_paired;
        for (int i = 0; i < 3; ++i) {
            Vec3 B_i;
            Eigen::Matrix<float, 3, 6> J_i;
            sensors[i].evaluate(placements[i],
                                inert[(i + 1) % 3], inert[(i + 2) % 3],
                                t, B_i, J_i);
            B_paired.block<3, 1>(i * 3, 0) = B_i;
        }

        const float share = (B_all - B_paired).norm() / B_paired.norm();
        snprintf(msg, sizeof(msg),
                 "cross-magnet share at standoff %.1fmm is %.2f%%, expected roughly 1-5%%",
                 standoff, 100.0f * share);
        TEST_ASSERT_TRUE_MESSAGE(share > 0.005f && share < 0.10f, msg);
        TEST_MESSAGE(msg);
    }
}

// ======================================================================
// 4. ForwardModel: Grid Sweep & Calibration State Validation
// ======================================================================

// Updated to accept hardware calibration states.
// Sensor gain is no longer part of ForwardModel/VirtualSensor - it's applied by
// SensorController on raw readings before they ever reach the solver - so
// this exercises the two per-magnet states that *are* still in the model:
// axis tilt and polarization strength.
static void compute_forward_model_jacobians(
        const Vec3& t, const Mat3& R,
        const Mat3 magnet_rotations[3],
        const float magnet_strengths[3],
        Eigen::Matrix<float, 9, 6>& J_analytic,
        Eigen::Matrix<float, 9, 6>& J_numeric) {

    MagnetModel magnets[3] = {
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[0], magnet_rotations[0], magnet_strengths[0]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[1], magnet_rotations[1], magnet_strengths[1]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[2], magnet_rotations[2], magnet_strengths[2]),
    };

    VirtualSensor sensors[3] = {
        VirtualSensor(SENSOR_POS[0]),
        VirtualSensor(SENSOR_POS[1]),
        VirtualSensor(SENSOR_POS[2]),
    };
    ForwardModel fm(sensors, magnets);

    Eigen::Matrix<float, 9, 1> residual0;
    fm.evaluate(t, R, residual0, J_analytic);

    J_numeric = Eigen::Matrix<float, 9, 6>::Zero();

    // Translation columns (0..2)
    for (int i = 0; i < 3; ++i) {
        Vec3 dt = Vec3::Zero();
        dt[i] = FD_STEP_LINEAR;

        Eigen::Matrix<float, 9, 1> res_p, res_m;
        Eigen::Matrix<float, 9, 6> J_dummy;
        fm.evaluate(t + dt, R, res_p, J_dummy);
        fm.evaluate(t - dt, R, res_m, J_dummy);

        J_numeric.col(i) = (res_p - res_m) / (2.0f * FD_STEP_LINEAR);
    }

    // Rotation columns (3..5)
    for (int i = 0; i < 3; ++i) {
        Vec3 dw = Vec3::Zero();
        dw[i] = FD_STEP_ANGULAR;

        Mat3 R_p = exp_so3(dw) * R;
        Mat3 R_m = exp_so3(-dw) * R;

        Eigen::Matrix<float, 9, 1> res_p, res_m;
        Eigen::Matrix<float, 9, 6> J_dummy;
        fm.evaluate(t, R_p, res_p, J_dummy);
        fm.evaluate(t, R_m, res_m, J_dummy);

        J_numeric.col(3 + i) = (res_p - res_m) / (2.0f * FD_STEP_ANGULAR);
    }
}

// The Massive Grid Test
void test_forward_model_jacobian_grid(void) {
    // 1. Define Calibration Scenarios
    // Sensor gain/skew is no longer modeled here - see the comment on
    // compute_forward_model_jacobians(). Only magnet tilt/orientation varies.
    // Scenario A: Perfect Hardware
    Mat3 tilts_perfect[3] = { Mat3::Identity(), Mat3::Identity(), Mat3::Identity() };

    float strengths_perfect[3] = {
        BICUBIC_FIELD_REFERENCE_MT, BICUBIC_FIELD_REFERENCE_MT, BICUBIC_FIELD_REFERENCE_MT
    };

    // Scenario B: Realistic Manufacturing Tolerances (magnet tilt)
    Mat3 tilts_real[3] = {
        exp_so3(Vec3( 0.03f, -0.02f,  0.01f)), // ~2 deg tilt
        exp_so3(Vec3(-0.01f,  0.04f,  0.00f)),
        exp_so3(Vec3( 0.02f,  0.01f, -0.03f))
    };
    // Per-magnet polarization spread, expressed as a ratio of
    // BICUBIC_FIELD_REFERENCE_MT so the scenario means the same thing
    // regardless of what that reference value is. The fit's own prior on how
    // much magnets from one batch differ is ~1%; these are deliberately wider,
    // plus a common-mode offset, since the common mode is the absolute field
    // scale and is left unregularized by the fit.
    float strengths_real[3] = {
        0.94f * BICUBIC_FIELD_REFERENCE_MT,
        1.08f * BICUBIC_FIELD_REFERENCE_MT,
        1.01f * BICUBIC_FIELD_REFERENCE_MT,
    };

    struct HardwareState {
        const Mat3* tilts;
        const float* strengths;
        const char* name;
    };
    HardwareState hw_states[] = {
        { tilts_perfect, strengths_perfect, "Ideal Hardware" },
        { tilts_real, strengths_real, "Distorted Hardware" }
    };

    // 2. Define Pose Grid Bounds
    // Translations (mm) - kept small to avoid pushing local coordinates out of the [0, 6] r-bounds
    float t_x_steps[] = { -3.0f, -1.5f, 0.0f, 1.2f };
    float t_y_steps[] = {  -1.5f, 0.0f, 0.5f, 1.5f };
    // Z is the vertical standoff (rest is 6.0mm)
    float t_z_steps[] = { 8.0f,  6.0f, 4.3f, 2.5f }; 
    
    // Rotations (axis-angle vectors)
    Vec3 rot_steps[] = {
        Vec3(0.0f, 0.0f, 0.0f),         // Rest
        Vec3(0.08f, -0.05f, 0.02f),     // Tilted X/Y, slight Yaw
        Vec3(-0.04f, 0.07f, -0.06f)     // Opposite tilt
    };

    int total_tests = 0;
    int passed_tests = 0;
    char msg[256];

    // 3. Run the Sweep
    for (const auto& hw : hw_states) {
        for (float tx : t_x_steps) {
            for (float ty : t_y_steps) {
                for (float tz : t_z_steps) {
                    for (const Vec3& r_vec : rot_steps) {
                        
                        Vec3 t(tx, ty, tz);
                        Mat3 R = exp_so3(r_vec);
                        
                        Eigen::Matrix<float, 9, 6> J_analytic, J_numeric;
                        compute_forward_model_jacobians(t, R, hw.tilts, hw.strengths, J_analytic, J_numeric);

                        float e = max_rel_error_mat<9, 6>(J_analytic, J_numeric);
                        
                        snprintf(msg, sizeof(msg), 
                                 "[%s] J mismatch at t=(%.2f, %.2f, %.2f). r=(%.2f, %.2f, %.2f) Max rel err: %.5f", 
                                 hw.name, tx, ty, tz, r_vec[0], r_vec[1], r_vec[2], e);
                        
                        TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg); // slightly looser tolerance for highly skewed combos
                        
                        total_tests++;
                        passed_tests++;
                    }
                }
            }
        }
    }
    
    // Optional: Print a summary to the serial monitor so you know it actually ran all of them
    snprintf(msg, sizeof(msg), "Grid sweep complete: %d/%d points passed.", passed_tests, total_tests);
    TEST_MESSAGE(msg);
}

// ======================================================================
// Unity entry points
// ======================================================================

void setUp(void) {}
void tearDown(void) {}

// The suite itself, kept in one place because there are two entry points into
// it: the board runs setup()/loop(), the host runs main(). Adding a test to
// one and not the other is exactly the kind of divergence that would make the
// two environments quietly disagree about what "passing" means.
// Returns UNITY_END()'s failure count.
static int run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_bicubic_field_derivatives);
    RUN_TEST(test_magnet_model_jacobian_generic);
    RUN_TEST(test_magnet_model_jacobian_at_origin);
    RUN_TEST(test_magnet_strength_scales_field_and_jacobian);
    RUN_TEST(test_dipole_field_jacobian);
    RUN_TEST(test_dipole_matches_the_magnet_the_table_models);
    RUN_TEST(test_cross_magnet_terms_are_actually_present);
    RUN_TEST(test_forward_model_jacobian_grid);
    return UNITY_END();
}

#ifdef ARDUINO

void setup() {
    delay(2000); // let serial monitor attach
    run_all_tests();
}

void loop() {}

#else

// env:native_test (platformio.ini) builds without a framework, so there is no
// setup()/loop() for anything to call -- the host runner starts at main(), and
// the failure count has to come back as the exit status for `pio test` to
// notice a failure at all.
int main() {
    return run_all_tests();
}

#endif
