// test_jacobians.cpp
//
// Finite-difference validation of the analytic Jacobians used in the
// magnetic 6DOF pose solver:
//   1. BicubicField    - d_dr / d_dz returned by evaluate()
//   2. MagnetModel      - J_local = dB_l/dv_l
//   3. ForwardModel     - full 9x6 J (translation block + rotation block)
//
// Run with Unity on-device (RP2040 / PlatformIO).
//
// ---------------------------------------------------------------------
// ASSUMPTIONS / THINGS YOU NEED TO CHECK BEFORE TRUSTING RESULTS
// ---------------------------------------------------------------------
// (a) ForwardModel::evaluate is assumed to take `residual` and `J` by
//     REFERENCE. The signature you sent has them by value, which cannot
//     return anything to the caller - almost certainly a typo. If it's
//     not, this file won't compile against your real header and the
//     signature below needs to change back.
//
// (b) SENSOR_POS[], MAGNET_LOCAL[], and the (r,z) sample points in
//     BICUBIC_TEST_POINTS are PLACEHOLDERS. Replace them with your real
//     PCB geometry and with (r,z) pairs you know are safely inside
//     BICUBIC_ORIGIN..BICUBIC_FAR (not on the boundary, since the FD
//     stencil pokes FD_STEP_LINEAR outside the sample point in both
//     directions).
//
// (c) FD_STEP_LINEAR / FD_STEP_ANGULAR assume position units of mm and
//     angle units of radians. If your state vector uses meters, scale
//     FD_STEP_LINEAR down accordingly (see comment at declaration).
// ---------------------------------------------------------------------

#include <unity.h>
#include "math3D.h"
#include "magnet_model/BicubicField.h"
#include "magnet_model/magnet_local_model.h"   // declares CALCULATED_BICUBIC_FIELD + MagnetModel
#include "magnet_model/forward_model.h"        // declares ForwardModel  (adjust filename if different)

// ======================================================================
// Config: fill in with real values
// ======================================================================

// Step sizes for central differences.
// Optimal FD step for float32 central differences is roughly
// cbrt(machine_eps) ~ 5e-3 in relative terms. Pick these relative to the
// physical scale of your problem (magnet is 6mm, so mm-scale steps of
// 1e-3 mm are ~1/6000 of the part size - reasonable).
static constexpr float FD_STEP_LINEAR  = 1.0e-3f;   // mm (or your length unit)
static constexpr float FD_STEP_ANGULAR = 1.0e-3f;   // radians

// (r, z) points to probe the BicubicField / MagnetModel derivatives at.
// MUST be interior to [BICUBIC_ORIGIN, BICUBIC_FAR] with margin >= FD_STEP_LINEAR.
struct RZSample { float r; float z; };
static constexpr RZSample BICUBIC_TEST_POINTS[] = {
    { 2.0f,  0.0f },
    { 2.0f,  1.5f },
    { 4.0f, -1.5f },
    { 1.0f,  2.5f },
};

// Placeholder PCB / knob geometry for the ForwardModel test.
// Replace with your real sensor positions and magnet resting positions.
static const Vec3 SENSOR_POS[3] = {
    Vec3( 10.0f,   0.0f, 5.0f),
    Vec3( -5.0f,  8.66f, 5.0f),
    Vec3( -5.0f, -8.66f, 5.0f),
};
static const Vec3 MAGNET_LOCAL[3] = {
    Vec3( 10.0f,   0.0f, 0.0f),
    Vec3( -5.0f,  8.66f, 0.0f),
    Vec3( -5.0f, -8.66f, 0.0f),
};

// Base pose used for the ForwardModel test.
static const Vec3 BASE_T(0.5f, -0.3f, 20.0f);

// ======================================================================
// Helpers
// ======================================================================

// Relative error with an absolute floor so we don't divide by ~0.
static float rel_error(float analytic, float numeric, float floor_ = 1.0e-5f) {
    float denom = std::max(std::fabs(analytic), std::max(std::fabs(numeric), floor_));
    return std::fabs(analytic - numeric) / denom;
}

// Max relative error over a 3-vector.
static float max_rel_error(const Vec3& a, const Vec3& n, float floor_ = 1.0e-5f) {
    float e = 0.0f;
    e = std::max(e, rel_error(a.x(), n.x(), floor_));
    e = std::max(e, rel_error(a.y(), n.y(), floor_));
    e = std::max(e, rel_error(a.z(), n.z(), floor_));
    return e;
}

// Max relative error over a matrix, column by column (as Vec3s), generic size.
template <int ROWS, int COLS>
static float max_rel_error_mat(const Eigen::Matrix<float, ROWS, COLS>& A,
                                const Eigen::Matrix<float, ROWS, COLS>& N,
                                float floor_ = 1.0e-5f) {
    float e = 0.0f;
    for (int c = 0; c < COLS; ++c) {
        for (int r = 0; r < ROWS; ++r) {
            e = std::max(e, rel_error(A(r, c), N(r, c), floor_));
        }
    }
    return e;
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
    Vec3 val0, ddr0, ddz0;
    CALCULATED_BICUBIC_FIELD.evaluate(r0, z0, val0, ddr0, ddz0);

    Vec3 val_rp, val_rm, dummy_a, dummy_b;
    CALCULATED_BICUBIC_FIELD.evaluate(r0 + FD_STEP_LINEAR, z0, val_rp, dummy_a, dummy_b);
    CALCULATED_BICUBIC_FIELD.evaluate(r0 - FD_STEP_LINEAR, z0, val_rm, dummy_a, dummy_b);
    Vec3 ddr_num = (val_rp - val_rm) / (2.0f * FD_STEP_LINEAR);

    Vec3 val_zp, val_zm;
    CALCULATED_BICUBIC_FIELD.evaluate(r0, z0 + FD_STEP_LINEAR, val_zp, dummy_a, dummy_b);
    CALCULATED_BICUBIC_FIELD.evaluate(r0, z0 - FD_STEP_LINEAR, val_zm, dummy_a, dummy_b);
    Vec3 ddz_num = (val_zp - val_zm) / (2.0f * FD_STEP_LINEAR);

    char msg[128];
    float e_dr = max_rel_error(ddr0, ddr_num);
    snprintf(msg, sizeof(msg), "d_dr mismatch at r=%.3f z=%.3f (rel err %.5f)", r0, z0, e_dr);
    TEST_ASSERT_TRUE_MESSAGE(e_dr < 0.01f, msg);

    float e_dz = max_rel_error(ddz0, ddz_num);
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

    char msg[160];
    float e = max_rel_error_mat<3, 3>(J_analytic, J_numeric);
    snprintf(msg, sizeof(msg), "J_local mismatch at v_l=(%.3f,%.3f,%.3f) rel err %.5f",
             v_l.x(), v_l.y(), v_l.z(), e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);
}

void test_magnet_model_jacobian_generic(void) {
    MagnetModel model(CALCULATED_BICUBIC_FIELD, Vec3::Zero());
    // Generic points away from r=0 (in the local frame v_l = [x_l,y_l,z_l]).
    check_magnet_model_at(model, Vec3(2.0f, 0.0f, 0.0f));
    check_magnet_model_at(model, Vec3(1.4f, 1.4f, 1.0f));
    check_magnet_model_at(model, Vec3(0.0f, 3.0f, -1.0f));
    check_magnet_model_at(model, Vec3(-2.0f, -2.0f, 2.0f));
}

void test_magnet_model_jacobian_at_origin(void) {
    // r = sqrt(x_l^2 + y_l^2) = 0 exercises the L'Hopital branch in
    // J_local's top-left 2x2 block. Perturbing x_l or y_l off zero
    // gives r = FD_STEP_LINEAR > 0 on both sides (never crosses back
    // through the singularity), so central differences are well-defined
    // even though the *base* point requires the L'Hopital limit.
    MagnetModel model(CALCULATED_BICUBIC_FIELD, Vec3::Zero());
    check_magnet_model_at(model, Vec3(0.0f, 0.0f, 0.5f));
    check_magnet_model_at(model, Vec3(0.0f, 0.0f, -0.5f));
}

// ======================================================================
// 3. ForwardModel: check the full 9x6 J against central differences of
//    the residual itself (not the predicted field). Differencing the
//    residual directly means we don't need to know or assume the sign
//    convention (residual = measured - predicted vs predicted - measured)
//    - whatever convention evaluate() uses internally, J should match
//    d(residual)/d(pose) exactly.
// ======================================================================

void test_forward_model_jacobian_translation(void) {
    MagnetModel magnets[3] = {
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[0]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[1]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[2]),
    };
    ForwardModel fm(SENSOR_POS, magnets);

    Mat3 R = Mat3::Identity();

    // Use a self-consistent "measured" field: whatever the model predicts
    // at the base pose. The residual's *value* at the base pose doesn't
    // matter for a Jacobian check, only how it changes - but evaluate()
    // needs some measured_fields[3] input, so any fixed vector works as
    // long as it's held constant across the +/- perturbations.
    Vec3 measured[3] = { Vec3(10.0f, 5.0f, -3.0f),
                         Vec3(-4.0f, 8.0f,  2.0f),
                         Vec3( 1.0f,-6.0f,  7.0f) };

    Eigen::Matrix<float, 9, 1> residual0;
    Eigen::Matrix<float, 9, 6> J;
    fm.evaluate(BASE_T, R, measured, residual0, J);

    Eigen::Matrix<float, 9, 6> J_numeric = Eigen::Matrix<float, 9, 6>::Zero();

    for (int i = 0; i < 3; ++i) {
        Vec3 dt = Vec3::Zero();
        dt[i] = FD_STEP_LINEAR;

        Eigen::Matrix<float, 9, 1> res_p, res_m;
        Eigen::Matrix<float, 9, 6> J_dummy;
        fm.evaluate(BASE_T + dt, R, measured, res_p, J_dummy);
        fm.evaluate(BASE_T - dt, R, measured, res_m, J_dummy);

        J_numeric.col(i) = (res_p - res_m) / (2.0f * FD_STEP_LINEAR);
    }

    char msg[128];
    // Only check the translation columns (0..2) here.
    float e = 0.0f;
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 9; ++r)
            e = std::max(e, rel_error(J(r, c), J_numeric(r, c)));
    snprintf(msg, sizeof(msg), "J_trans mismatch, max rel err %.5f", e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);
}

void test_forward_model_jacobian_rotation(void) {
    MagnetModel magnets[3] = {
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[0]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[1]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[2]),
    };
    ForwardModel fm(SENSOR_POS, magnets);

    // Use a non-identity base rotation so we're not accidentally testing
    // only a degenerate case.
    Vec3 base_axis(0.3f, -0.5f, 0.2f);
    Mat3 R = exp_so3(base_axis);

    Vec3 measured[3] = { Vec3(10.0f, 5.0f, -3.0f),
                         Vec3(-4.0f, 8.0f,  2.0f),
                         Vec3( 1.0f,-6.0f,  7.0f) };

    Eigen::Matrix<float, 9, 1> residual0;
    Eigen::Matrix<float, 9, 6> J;
    fm.evaluate(BASE_T, R, measured, residual0, J);

    Eigen::Matrix<float, 9, 6> J_numeric = Eigen::Matrix<float, 9, 6>::Zero();

    for (int i = 0; i < 3; ++i) {
        Vec3 dw = Vec3::Zero();
        dw[i] = FD_STEP_ANGULAR;

        Mat3 R_p = exp_so3(dw) * R;    // exact exponential perturbation
        Mat3 R_m = exp_so3(-dw) * R;

        Eigen::Matrix<float, 9, 1> res_p, res_m;
        Eigen::Matrix<float, 9, 6> J_dummy;
        fm.evaluate(BASE_T, R_p, measured, res_p, J_dummy);
        fm.evaluate(BASE_T, R_m, measured, res_m, J_dummy);

        J_numeric.col(3 + i) = (res_p - res_m) / (2.0f * FD_STEP_ANGULAR);
    }

    char msg[128];
    float e = 0.0f;
    for (int c = 3; c < 6; ++c)
        for (int r = 0; r < 9; ++r)
            e = std::max(e, rel_error(J(r, c), J_numeric(r, c)));
    snprintf(msg, sizeof(msg), "J_rot mismatch, max rel err %.5f", e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);
}

// ======================================================================
// Unity entry points
// ======================================================================

void setUp(void) {}
void tearDown(void) {}

void setup() {
    delay(2000); // let serial monitor attach
    UNITY_BEGIN();
    RUN_TEST(test_bicubic_field_derivatives);
    RUN_TEST(test_magnet_model_jacobian_generic);
    RUN_TEST(test_magnet_model_jacobian_at_origin);
    RUN_TEST(test_forward_model_jacobian_translation);
    RUN_TEST(test_forward_model_jacobian_rotation);
    UNITY_END();
}

void loop() {}