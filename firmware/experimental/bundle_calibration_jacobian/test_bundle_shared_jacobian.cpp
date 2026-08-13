// test_bundle_shared_jacobian.cpp -- PROTOTYPE, not wired into the build.
//
// Finite-difference validation of bundle_shared_jacobian.h /
// bundle_linear_jacobian.h, in exactly the style firmware/test/test_jacobian.cpp
// already uses for the pose Jacobian: perturb the real physical quantity,
// rebuild the (const-membered) model from scratch, central-difference,
// compare. No autodiff, no symbolic cross-check -- finite differences against
// the real evaluate() calls is the bedrock here, same as everywhere else in
// this test suite.
//
// Structured in two tiers:
//   1. Per-quantity checks (check_shared_jacobian_at): does each raw
//      derivative (d_magnet_pos, d_magnet_tilt, d_strength) match perturbing
//      that one magnet's own state directly?
//   2. End-to-end (test_magnet_pos_gauge_projection): does the *assembled,
//      gauge-projected* free-parameter column -- what the solver would
//      actually use -- match perturbing the real physical shape parameter
//      (moving all three magnets at once via MAGNET_POS_BASIS)? Tier 1 can
//      pass while the wiring between magnets/sensors is still wrong (right
//      formula, wrong slice, transposed basis, off-by-one magnet index) --
//      tier 2 is the one that actually catches that class of bug.
//
// This file assumes it will eventually be merged into test_jacobian.cpp
// (sharing its FD_STEP constants, max_rel_error_mat, exp_so3, SENSOR_POS/
// MAGNET_LOCAL) rather than shipping as a second Unity binary long-term --
// duplicated here only so it can be read and run standalone while this is
// still a sketch.

#include <unity.h>
#include "math3D.h"
#include "magnet_model/BicubicField.h"
#include "magnet_model/magnet_local_model.h"
#include "magnet_model/forward_model.h"
#include "magnet_model/positions.h"
#include "bundle_shared_jacobian.h"
#include "bundle_linear_jacobian.h"
#include "bundle_magnet_pos_gauge.h"   // MAGNET_POS_BASIS, project_magnet_pos

static constexpr float FD_STEP_LINEAR  = 5.0e-4f;   // mm
static constexpr float FD_STEP_ANGULAR = 5.0e-4f;   // radians
static constexpr float REL_TOL         = 0.01f;     // matches test_jacobian.cpp's bar

static const Vec3 SENSOR_POS[3] = {
    Positions::sensor_1_world, Positions::sensor_2_world, Positions::sensor_3_world,
};
static const Vec3 MAGNET_LOCAL[3] = {
    Positions::Magnet_1_knob, Positions::Magnet_2_knob, Positions::Magnet_3_knob,
};
static const Vec3 BASE_T(2.0f, -1.5f, 6.0f);

static Mat3 exp_so3(const Vec3& w) {
    float theta = w.norm();
    Mat3 K = skew_matrix(w);
    if (theta < 1.0e-8f) return Mat3::Identity() + K + 0.5f * (K * K);
    float s = std::sin(theta) / theta;
    float c = (1.0f - std::cos(theta)) / (theta * theta);
    return Mat3::Identity() + s * K + c * (K * K);
}

template <int ROWS, int COLS>
static float max_rel_error_mat(const Eigen::Matrix<float, ROWS, COLS>& A,
                                const Eigen::Matrix<float, ROWS, COLS>& N) {
    float error_norm = (A - N).norm();
    float numeric_norm = N.norm();
    if (numeric_norm < 1e-8f) return error_norm;
    return error_norm / numeric_norm;
}

// ======================================================================
// Tier 1: per-quantity checks against evaluate_shared_jacobian()
// ======================================================================

static void check_shared_jacobian_at(
        const Vec3& magnet_pos, const Mat3& magnet_rot, float strength,
        const Vec3& sensor_pos, const Vec3& t, const Mat3& R, const char* label) {

    Sensor sensor(sensor_pos);
    // MagnetState, not a directly-built MagnetModel: this is the mutable
    // trial state a solver would hold and perturb, exercised through the
    // real entry point (evaluate_bundle_jacobian) exactly as a solver would
    // call it -- not a hand-rolled construct-and-call-separately sequence
    // that only resembles what production code would do.
    MagnetState state0{magnet_pos, magnet_rot, strength};

    Vec3 B0;
    Eigen::Matrix<float, 3, 6> J_pose;
    SharedJacobianBlock J_analytic;
    evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, state0, t, R, B0, J_pose, J_analytic);

    char msg[256];

    // --- magnet position: perturb magnet_pos_knob directly ---
    Mat3 J_pos_numeric;
    for (int i = 0; i < 3; ++i) {
        Vec3 dm = Vec3::Zero(); dm[i] = FD_STEP_LINEAR;
        MagnetState sp{magnet_pos + dm, magnet_rot, strength};
        MagnetState sm{magnet_pos - dm, magnet_rot, strength};
        Vec3 Bp, Bm; Eigen::Matrix<float, 3, 6> Jd; SharedJacobianBlock Jsd;
        evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, sp, t, R, Bp, Jd, Jsd);
        evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, sm, t, R, Bm, Jd, Jsd);
        J_pos_numeric.col(i) = (Bp - Bm) / (2.0f * FD_STEP_LINEAR);
    }
    float e_pos = max_rel_error_mat<3, 3>(J_analytic.d_magnet_pos, J_pos_numeric);
    snprintf(msg, sizeof(msg), "[%s] d_magnet_pos mismatch, rel err %.5f", label, e_pos);
    TEST_ASSERT_TRUE_MESSAGE(e_pos < REL_TOL, msg);

    // --- magnet tilt: left-perturb magnet_rot; column 2 (spin) is dead ---
    Mat3 J_tilt_numeric;
    for (int i = 0; i < 3; ++i) {
        Vec3 dw = Vec3::Zero(); dw[i] = FD_STEP_ANGULAR;
        MagnetState sp{magnet_pos, exp_so3(dw) * magnet_rot, strength};
        MagnetState sm{magnet_pos, exp_so3(-dw) * magnet_rot, strength};
        Vec3 Bp, Bm; Eigen::Matrix<float, 3, 6> Jd; SharedJacobianBlock Jsd;
        evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, sp, t, R, Bp, Jd, Jsd);
        evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, sm, t, R, Bm, Jd, Jsd);
        J_tilt_numeric.col(i) = (Bp - Bm) / (2.0f * FD_STEP_ANGULAR);
    }
    // Full 3x3, not just columns 0-1: the raw Jacobian is a real 3x3
    // (derivative against a left perturbation in all 3 world/knob-frame
    // directions), and column 2 is not a don't-care -- an earlier version of
    // this test asserted the *world-frame* e_2 direction should be dead,
    // which is only true at magnet_rot == Identity (see the body-frame check
    // below for why) and false in general, silently masking whether column 2
    // is actually being computed correctly. Comparing the full matrix here
    // means there is nowhere for a sign/axis bug in that column to hide.
    float e_tilt = max_rel_error_mat<3, 3>(J_analytic.d_magnet_tilt, J_tilt_numeric);
    snprintf(msg, sizeof(msg), "[%s] d_magnet_tilt mismatch, rel err %.5f", label, e_tilt);
    TEST_ASSERT_TRUE_MESSAGE(e_tilt < REL_TOL, msg);

    // "Spin about the magnet's own axis does nothing" is not really a
    // derivative claim -- it's an EXACT symmetry (the local field model is
    // manifestly axisymmetric about the magnet's own current polarization
    // axis: MagnetModel::evaluate's (Br*cx, Br*cy, Bz) form is invariant
    // under any rotation about that axis by construction). Testing it via a
    // finite difference means differencing two large near-equal floats to
    // extract a value that should be ~0 -- exactly where central differences
    // are worst (roundoff, not truncation, dominates; shrinking the step
    // makes the estimate WORSE, not better -- confirmed by hand: 0.94 at
    // step=5e-4, growing to 12.7 at step=2e-5). So don't difference it: spin
    // the magnet by real, non-infinitesimal angles about its own current
    // axis (R_mag's own z column, not the fixed world-frame e_2, which only
    // coincides with it at zero tilt -- R exp(theta v x) = exp(theta (Rv) x)
    // R is why) and check Sensor::evaluate's OUTPUT is unchanged directly.
    // No subtraction of comparable quantities, no near-zero target -- this
    // resolves to float32 machine precision (~2e-6 relative, measured),
    // not the ~1e-3 the differencing approach was capped at.
    Vec3 own_axis = magnet_rot.col(2);
    float field_scale = B0.norm();
    for (float theta : {0.01f, 1.0f, 3.0f}) {
        Mat3 spin = exp_so3(theta * own_axis);
        MagnetState spun_state{magnet_pos, spin * magnet_rot, strength};
        Vec3 B_spun; Eigen::Matrix<float, 3, 6> spin_Jd; SharedJacobianBlock spin_Jsd;
        evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, spun_state, t, R, B_spun, spin_Jd, spin_Jsd);
        float e = (B_spun - B0).norm() / field_scale;
        snprintf(msg, sizeof(msg),
                 "[%s] spinning the magnet %.2f rad about its OWN axis changed the field by rel %.2e (should be ~machine eps)",
                 label, theta, e);
        TEST_ASSERT_TRUE_MESSAGE(e < 1e-4f, msg);
    }

    // --- strength: relative step, since strength is O(hundreds of mT) ---
    float ds = strength * 1.0e-3f;
    MagnetState sp{magnet_pos, magnet_rot, strength + ds};
    MagnetState sm{magnet_pos, magnet_rot, strength - ds};
    Vec3 Bp, Bm; Eigen::Matrix<float, 3, 6> Jd; SharedJacobianBlock Jsd;
    evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, sp, t, R, Bp, Jd, Jsd);
    evaluate_bundle_jacobian(sensor, CALCULATED_BICUBIC_FIELD, sm, t, R, Bm, Jd, Jsd);
    Vec3 d_strength_numeric = (Bp - Bm) / (2.0f * ds);
    float e_strength = max_rel_error_mat<3, 1>(J_analytic.d_strength, d_strength_numeric);
    snprintf(msg, sizeof(msg), "[%s] d_strength mismatch, rel err %.5f", label, e_strength);
    TEST_ASSERT_TRUE_MESSAGE(e_strength < REL_TOL, msg);
}

void test_shared_jacobian_grid(void) {
    // Mirrors test_forward_model_jacobian_grid's two hardware scenarios and
    // a small pose sweep -- not the full grid (this is a sketch), enough to
    // exercise off-identity R/R_mag together, which is where a transpose or
    // sign error in the M@R / skew(d) chain would actually show up.
    struct Pose { Vec3 t; Vec3 rot; const char* name; };
    Pose poses[] = {
        { BASE_T, Vec3(0, 0, 0), "rest" },
        { Vec3(3.5f, -2.0f, 4.3f), Vec3(0.08f, -0.05f, 0.02f), "off-axis" },
        { Vec3(-1.0f, 1.2f, 8.0f), Vec3(-0.04f, 0.07f, -0.06f), "opposite tilt" },
    };
    struct Magnet { Vec3 pos; Vec3 tilt_axis; float strength; const char* name; };
    Magnet magnets[] = {
        { MAGNET_LOCAL[0], Vec3(0, 0, 0), BICUBIC_FIELD_REFERENCE_MT, "ideal" },
        { MAGNET_LOCAL[0], Vec3(0.03f, -0.02f, 0.0f), 0.94f * BICUBIC_FIELD_REFERENCE_MT, "tilted+weak" },
    };

    char label[128];
    for (const auto& p : poses) {
        for (const auto& m : magnets) {
            snprintf(label, sizeof(label), "%s / %s", p.name, m.name);
            check_shared_jacobian_at(
                m.pos, exp_so3(m.tilt_axis), m.strength,
                SENSOR_POS[0], p.t, exp_so3(p.rot), label);
        }
    }
}

// ======================================================================
// Tier 2: end-to-end check of the gauge-projected free-parameter column,
// against perturbing the real physical shape parameter (all 3 magnets move
// together via MAGNET_POS_BASIS's column, exactly as the fit would).
// ======================================================================
void test_magnet_pos_gauge_projection(void) {
    // MagnetState[3], not MagnetModel[3]: same reasoning as
    // check_shared_jacobian_at -- this is the mutable trial state a solver
    // holds, fed through the real per-iteration entry point.
    MagnetState states0[3] = {
        {MAGNET_LOCAL[0], Mat3::Identity(), BICUBIC_FIELD_REFERENCE_MT},
        {MAGNET_LOCAL[1], Mat3::Identity(), BICUBIC_FIELD_REFERENCE_MT},
        {MAGNET_LOCAL[2], Mat3::Identity(), BICUBIC_FIELD_REFERENCE_MT},
    };
    Sensor sensors[3] = { Sensor(SENSOR_POS[0]), Sensor(SENSOR_POS[1]), Sensor(SENSOR_POS[2]) };
    const Vec3 t = BASE_T;
    const Mat3 R = Mat3::Identity();

    for (int k = 0; k < 3; ++k) {
        // Analytic: each sensor's own project_magnet_pos(), stacked.
        Eigen::Matrix<float, 9, 1> J_analytic_col;
        for (int i = 0; i < 3; ++i) {
            Vec3 B0; Eigen::Matrix<float, 3, 6> Jp; SharedJacobianBlock Js;
            evaluate_bundle_jacobian(sensors[i], CALCULATED_BICUBIC_FIELD, states0[i], t, R, B0, Jp, Js);
            Mat3 proj = project_magnet_pos(Js.d_magnet_pos, i);
            J_analytic_col.block<3, 1>(3 * i, 0) = proj.col(k);
        }

        // Numeric: move all 3 magnets by delta * MAGNET_POS_BASIS[:, k] --
        // the actual physical motion this free parameter produces -- and
        // central-difference the real 9-vector.
        Eigen::Matrix<float, 9, 1> B_plus, B_minus;
        for (int i = 0; i < 3; ++i) {
            Vec3 dm(MAGNET_POS_BASIS(3 * i + 0, k),
                    MAGNET_POS_BASIS(3 * i + 1, k),
                    MAGNET_POS_BASIS(3 * i + 2, k));
            MagnetState sp{states0[i].pos + FD_STEP_LINEAR * dm, states0[i].rotation, states0[i].strength_mT};
            MagnetState sm{states0[i].pos - FD_STEP_LINEAR * dm, states0[i].rotation, states0[i].strength_mT};
            Vec3 bp, bm; Eigen::Matrix<float, 3, 6> Jd; SharedJacobianBlock Jsd;
            evaluate_bundle_jacobian(sensors[i], CALCULATED_BICUBIC_FIELD, sp, t, R, bp, Jd, Jsd);
            evaluate_bundle_jacobian(sensors[i], CALCULATED_BICUBIC_FIELD, sm, t, R, bm, Jd, Jsd);
            B_plus.block<3, 1>(3 * i, 0) = bp;
            B_minus.block<3, 1>(3 * i, 0) = bm;
        }
        Eigen::Matrix<float, 9, 1> J_numeric_col = (B_plus - B_minus) / (2.0f * FD_STEP_LINEAR);

        float e = max_rel_error_mat<9, 1>(J_analytic_col, J_numeric_col);
        char msg[128];
        snprintf(msg, sizeof(msg), "shape parameter %d: assembled column mismatch, rel err %.5f", k, e);
        TEST_ASSERT_TRUE_MESSAGE(e < REL_TOL, msg);
    }
}

// ======================================================================
// Unity entry points
// ======================================================================
void setUp(void) {}
void tearDown(void) {}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_shared_jacobian_grid);
    RUN_TEST(test_magnet_pos_gauge_projection);
    UNITY_END();
}

void loop() {}
