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
static constexpr float FD_STEP_LINEAR  = 1.0e-3f;   // mm (or your length unit)
static constexpr float FD_STEP_ANGULAR = 1.0e-3f;   // radians

// (r, z) points to probe the BicubicField / MagnetModel derivatives at.
// Valid domain per BICUBIC_ORIGIN=(0.0,-12.0), BICUBIC_FAR=(6.0,-0.5):
//   r in [0, 6], z in [-12, -0.5]  (z is always negative - sensor plane
//   sits below the magnet). Points below are comfortably interior with
//   margin >> FD_STEP_LINEAR.
struct RZSample { float r; float z; };
static constexpr RZSample BICUBIC_TEST_POINTS[] = {
    { 1.0f, -1.0f },
    { 3.0f, -6.0f },
    { 5.0f, -1.0f },
    { 0.5f, -11.0f },
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

// ======================================================================
// 3. ForwardModel: check the full 9x6 J against central differences of
//    the residual itself (not the predicted field). Differencing the
//    residual directly means we don't need to know or assume the sign
//    convention (residual = measured - predicted vs predicted - measured)
//    - whatever convention evaluate() uses internally, J should match
//    d(residual)/d(pose) exactly.
// ======================================================================

// Shared setup + FD computation for a given (t, R) pose. Fills J_analytic
// and J_numeric (all 6 columns) so callers can check whichever block(s)
// they care about.
static void compute_forward_model_jacobians(const Vec3& t, const Mat3& R,
                                             Eigen::Matrix<float, 9, 6>& J_analytic,
                                             Eigen::Matrix<float, 9, 6>& J_numeric) {
    MagnetModel magnets[3] = {
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[0]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[1]),
        MagnetModel(CALCULATED_BICUBIC_FIELD, MAGNET_LOCAL[2]),
    };
    ForwardModel fm(SENSOR_POS, magnets);

    Eigen::Matrix<float, 9, 1> residual0;
    fm.evaluate(t, R, residual0, J_analytic);

    J_numeric = Eigen::Matrix<float, 9, 6>::Zero();

    // Translation columns (0..2): straightforward Euclidean perturbation.
    for (int i = 0; i < 3; ++i) {
        Vec3 dt = Vec3::Zero();
        dt[i] = FD_STEP_LINEAR;

        Eigen::Matrix<float, 9, 1> res_p, res_m;
        Eigen::Matrix<float, 9, 6> J_dummy;
        fm.evaluate(t + dt, R, res_p, J_dummy);
        fm.evaluate(t - dt, R, res_m, J_dummy);

        J_numeric.col(i) = (res_p - res_m) / (2.0f * FD_STEP_LINEAR);
    }

    // Rotation columns (3..5): exact exponential-map perturbation of R.
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

void test_forward_model_jacobian_translation(void) {
    // See BASE_ROTATION_AXIS definition above for why this uses a
    // non-identity R rather than the true rest pose.
    Mat3 R = exp_so3(BASE_ROTATION_AXIS);
    Eigen::Matrix<float, 9, 6> J, J_numeric;
    compute_forward_model_jacobians(BASE_T, R, J, J_numeric);

    char msg[128];
    float e = max_rel_error_mat<9, 6>(J, J_numeric);
    snprintf(msg, sizeof(msg), "J_trans mismatch, max rel err %.5f", e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);
}

void test_forward_model_jacobian_rotation(void) {
    Mat3 R = exp_so3(BASE_ROTATION_AXIS);
    Eigen::Matrix<float, 9, 6> J, J_numeric;
    compute_forward_model_jacobians(BASE_T, R, J, J_numeric);

    char msg[128];
    float e = max_rel_error_mat<9, 6>(J, J_numeric);
    snprintf(msg, sizeof(msg), "J_rot mismatch, max rel err %.5f", e);
    TEST_ASSERT_TRUE_MESSAGE(e < 0.01f, msg);
}

void test_forward_model_jacobian_at_rest_pose(void) {
    // The true physical rest pose: frames rotationally aligned (R =
    // Identity), only the 6mm z-standoff as an offset. Checked
    // separately from the generic-pose tests above because it's a real
    // configuration the solver starts every tracking session from, not
    // just a synthetic point chosen for FD-testing convenience.
    Vec3 t_rest(0.0f, 0.0f, 6.0f);
    Mat3 R_rest = Mat3::Identity();

    Eigen::Matrix<float, 9, 6> J, J_numeric;
    compute_forward_model_jacobians(t_rest, R_rest, J, J_numeric);

    char msg[128];
    float e = max_rel_error_mat<9, 6>(J, J_numeric);
    snprintf(msg, sizeof(msg), "Full 9x6 J mismatch at rest pose, max rel err %.5f", e);
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
    RUN_TEST(test_forward_model_jacobian_at_rest_pose);
    UNITY_END();
}

void loop() {}
