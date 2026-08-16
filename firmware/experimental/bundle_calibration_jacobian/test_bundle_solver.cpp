// test_bundle_solver.cpp -- PROTOTYPE, not wired into the build.
//
// End-to-end check of bundle_solver.h: generate synthetic sensor readings from
// a KNOWN parameter vector using the real forward model, hand the solver only
// the readings, and see whether it fits them.
//
// What this can and cannot assert. The data is generated noise-free by the
// same model the solver uses, so a correct solver must drive the residual to
// roughly float32 zero -- that is a real, sharp claim and it is what is
// asserted below. It must NOT assert that the recovered parameter vector
// equals the generating one: several directions are genuinely degenerate at
// these pose ranges (TODO/cross-magnet-interference.md documents that magnet
// strength and distance are near-inseparable without cross-magnet coupling,
// and the position gauge has its own nullspace), so a different parameter
// vector explaining the same data is a correct answer, not a failure. Testing
// for parameter recovery would be testing identifiability, which is a property
// of the experiment design rather than of this code.
//
// Frame count is not arbitrary: each frame adds 9 observations and 6 pose
// unknowns, so the shared 45 are only determined once 3*N > 45, i.e. N > 15.
// 30 frames leaves real margin.

#include <unity.h>
#include <random>
#include "math3D.h"
#include "magnet_model/BicubicField.h"
#include "magnet_model/positions.h"
#include "bundle_solver.h"

static constexpr int N_FRAMES = 30;

static Mat3 test_exp_so3(const Vec3& w) {
    const float theta = w.norm();
    const Mat3 K = skew_matrix(w);
    if (theta < 1.0e-8f) return Mat3::Identity() + K + 0.5f * (K * K);
    const float s = std::sin(theta) / theta;
    const float c = (1.0f - std::cos(theta)) / (theta * theta);
    return Mat3::Identity() + s * K + c * (K * K);
}

// Statically allocated, not a local: sizeof is ~18 KB of accumulator plus the
// frames, against this device's 4 KB per-core stack. The host would tolerate
// it on the stack; the target would not, and the test should model the usage
// the target requires. See bundle_solver.h's memory note.
static BundleSolver<N_FRAMES> solver;

static BundleGeometry make_geometry() {
    BundleGeometry g;
    g.sensor_pos[0] = Positions::sensor_1_world;
    g.sensor_pos[1] = Positions::sensor_2_world;
    g.sensor_pos[2] = Positions::sensor_3_world;
    g.magnet_nominal_pos[0] = Positions::Magnet_1_knob;
    g.magnet_nominal_pos[1] = Positions::Magnet_2_knob;
    g.magnet_nominal_pos[2] = Positions::Magnet_3_knob;
    g.magnet_nominal_rot[0] = test_exp_so3(Vec3(0.02f, -0.01f, 0.0f));
    g.magnet_nominal_rot[1] = test_exp_so3(Vec3(-0.01f, 0.02f, 0.0f));
    g.magnet_nominal_rot[2] = test_exp_so3(Vec3(0.01f, 0.01f, 0.0f));
    g.magnet_nominal_strength_mT = BICUBIC_FIELD_REFERENCE_MT;
    return g;
}

// Poses spread over the range the knob actually reaches, so the frames carry
// independent information rather than 30 near-copies of one pose.
//
// Centred on Positions::approx_rest_pos (~21mm above the sensor plane), NOT
// on a hand-picked heave. The first version of this used z in [5,8], which
// puts the magnet-local query at z_l around +9 -- entirely outside the bicubic
// table's domain, whose bounds live in magnet_model_table.h. The generated
// "measurements" were then extrapolation artefacts of 500-2500 mT against a
// 1000 mT reference, and while the fit is still self-consistent out there
// (same model both sides), the extrapolated surface is nonlinear enough that
// LM stalled: 21 iterations, 5 accepted steps, cost down only 4x. Inside the
// table it converges. Worth remembering that a solver failing to converge can
// be a statement about the data it was given rather than about the solver.
static void make_poses(BundleFrame (&frames)[N_FRAMES]) {
    std::mt19937 rng(20240815);
    std::uniform_real_distribution<float> lateral(-2.5f, 2.5f);
    std::uniform_real_distribution<float> heave(-3.0f, 2.0f);   // about rest
    std::uniform_real_distribution<float> tilt(-0.10f, 0.10f);
    for (int k = 0; k < N_FRAMES; ++k) {
        frames[k].t = Positions::approx_rest_pos
            + Vec3(lateral(rng), lateral(rng), heave(rng));
        frames[k].R = test_exp_so3(Vec3(tilt(rng), tilt(rng), tilt(rng)));
    }
}

// Ground truth: tolerance-sized perturbations in every group, so no group is
// silently exercised at zero.
static Eigen::Matrix<float, N_SHARED_PARAMS, 1> make_truth() {
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> x =
        Eigen::Matrix<float, N_SHARED_PARAMS, 1>::Zero();
    x.segment<3>(COL_MAGNET_POS)    << 0.08f, -0.05f, 0.06f;
    x[COL_STRENGTH_MEAN]            = 0.06f;    // +6% field scale
    x.segment<2>(COL_STRENGTH_DIFF) << 0.03f, -0.02f;
    for (int m = 0; m < N_MAGNETS; ++m) {
        x[COL_MAGNET_TILT + 2 * m]     = 0.015f * (m + 1);
        x[COL_MAGNET_TILT + 2 * m + 1] = -0.012f * (m + 1);
    }
    for (int i = 0; i < N_SENSORS; ++i) {
        const int base = unit_block_start(i);
        x.segment<3>(base + UNIT_OFFSET_SENSOR_OFFSET) << 0.3f, -0.2f, 0.25f;
        for (int k = 0; k < 8; ++k) x[base + UNIT_OFFSET_GAIN + k] = 0.004f * (k - 3);
    }
    return x;
}

void test_solver_fits_synthetic_data(void) {
    solver.geometry = make_geometry();
    make_poses(solver.frames);

    // Generate measurements at the truth, through the solver's own forward
    // path, then discard the truth. rebuild_from_params + the prediction is
    // exactly what cost() consumes, so this cannot drift from what is fitted.
    const Eigen::Matrix<float, N_SHARED_PARAMS, 1> x_true = make_truth();
    solver.rebuild_from_params(x_true);
    for (int k = 0; k < N_FRAMES; ++k) {
        for (int i = 0; i < N_SENSORS; ++i) {
            VirtualSensor sensor(solver.geometry.sensor_pos[i]);
            MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[i]);
            Vec3 B; Eigen::Matrix<float, 3, 6> J_pose;
            sensor.evaluate(magnet, solver.frames[k].t, solver.frames[k].R, B, J_pose);
            solver.frames[k].measured[i] = solver.gains[i] * B + solver.offsets[i];
        }
    }

    // Start from nominal. Poses stay at truth so this isolates the shared
    // parameter fit; the pose back-substitution still runs every iteration and
    // has to not damage them.
    solver.x.setZero();
    solver.prior_sigma.setConstant(1.0e3f);   // effectively uninformative
    solver.observation_sigma = 1.0f;

    BundleSolverOptions opt;
    opt.max_iterations = 60;
    BundleSolverReport report = solver.solve(opt);

    char msg[256];
    snprintf(msg, sizeof(msg),
             "iterations %d, accepted %d, cost %.6e -> %.6e, converged %d",
             report.iterations, report.accepted_steps,
             report.initial_cost, report.final_cost, (int)report.converged);
    TEST_MESSAGE(msg);

    TEST_ASSERT_TRUE_MESSAGE(report.accepted_steps > 0, "no step was ever accepted");

    // Noise-free data from the same model: the residual has an exact zero to
    // find, so anything but a large drop means the step is wrong, not that the
    // problem is hard.
    snprintf(msg, sizeof(msg), "cost only fell from %.6e to %.6e",
             report.initial_cost, report.final_cost);
    TEST_ASSERT_TRUE_MESSAGE(report.final_cost < report.initial_cost * 1.0e-4f, msg);

    // Per-observation RMS residual, in mT -- the number that means something
    // physically. 270 observations.
    const float rms = std::sqrt(report.final_cost / (N_FRAMES * N_SENSORS * 3));
    snprintf(msg, sizeof(msg), "RMS residual %.5f mT (want < 0.01)", rms);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(rms < 0.01f, msg);
}

// The poses must survive the fit. Pass 2's back-substitution runs every
// iteration whether or not the poses need moving, so a sign error or a
// mismatched linearization point there would show up as poses drifting away
// from a starting point that was already correct -- while the shared fit
// absorbed the damage and the cost still fell.
void test_solver_leaves_correct_poses_alone(void) {
    solver.geometry = make_geometry();
    make_poses(solver.frames);

    BundleFrame truth_poses[N_FRAMES];
    for (int k = 0; k < N_FRAMES; ++k) truth_poses[k] = solver.frames[k];

    const Eigen::Matrix<float, N_SHARED_PARAMS, 1> x_true = make_truth();
    solver.rebuild_from_params(x_true);
    for (int k = 0; k < N_FRAMES; ++k) {
        for (int i = 0; i < N_SENSORS; ++i) {
            VirtualSensor sensor(solver.geometry.sensor_pos[i]);
            MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[i]);
            Vec3 B; Eigen::Matrix<float, 3, 6> J_pose;
            sensor.evaluate(magnet, solver.frames[k].t, solver.frames[k].R, B, J_pose);
            solver.frames[k].measured[i] = solver.gains[i] * B + solver.offsets[i];
        }
    }

    solver.x = x_true;                        // start AT the answer
    solver.prior_sigma.setConstant(1.0e3f);
    solver.observation_sigma = 1.0f;

    BundleSolverOptions opt;
    opt.max_iterations = 5;
    solver.solve(opt);

    char msg[224];
    float worst_t = 0.0f, worst_R = 0.0f;
    for (int k = 0; k < N_FRAMES; ++k) {
        worst_t = std::max(worst_t, (solver.frames[k].t - truth_poses[k].t).norm());
        worst_R = std::max(worst_R, (solver.frames[k].R - truth_poses[k].R).norm());
    }
    snprintf(msg, sizeof(msg),
             "starting at the answer, poses drifted: max |dt| = %.3e mm, max |dR| = %.3e",
             worst_t, worst_R);
    TEST_ASSERT_TRUE_MESSAGE(worst_t < 1.0e-2f && worst_R < 1.0e-3f, msg);
}

// Fills solver.frames[].measured from a parameter vector, through the same
// forward path cost() uses.
static void generate_measurements(const Eigen::Matrix<float, N_SHARED_PARAMS, 1>& x) {
    solver.rebuild_from_params(x);
    for (int k = 0; k < N_FRAMES; ++k) {
        for (int i = 0; i < N_SENSORS; ++i) {
            VirtualSensor sensor(solver.geometry.sensor_pos[i]);
            MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[i]);
            Vec3 B; Eigen::Matrix<float, 3, 6> J_pose;
            sensor.evaluate(magnet, solver.frames[k].t, solver.frames[k].R, B, J_pose);
            solver.frames[k].measured[i] = solver.gains[i] * B + solver.offsets[i];
        }
    }
}

// The prior has to actually do something, and in the right direction.
//
// This exists because both tests above run add_prior() on every iteration with
// a sigma so loose it contributes nothing measurable -- they would pass
// unchanged if the ridge term were dropped entirely, or if its sign were
// inverted. Fit the same data twice, once unregularized and once with a very
// tight prior on gain_sym (documented in bundle_params.py as weakly observable,
// so the prior has room to win), and check the regularized answer is pulled
// toward nominal rather than away from it.
void test_prior_pulls_toward_nominal(void) {
    solver.geometry = make_geometry();
    make_poses(solver.frames);
    const BundleFrame truth_poses_backup[1] = {solver.frames[0]};
    (void)truth_poses_backup;

    Eigen::Matrix<float, N_SHARED_PARAMS, 1> x_true = make_truth();
    const int gain_sym_col = unit_block_start(0) + UNIT_OFFSET_GAIN + 2;
    x_true[gain_sym_col] = 0.05f;
    generate_measurements(x_true);

    BundleSolverOptions opt;
    opt.max_iterations = 60;
    const float inf = std::numeric_limits<float>::infinity();

    // Fit A: no prior anywhere -- the data's own answer.
    BundleFrame saved[N_FRAMES];
    for (int k = 0; k < N_FRAMES; ++k) saved[k] = solver.frames[k];
    solver.x.setZero();
    solver.prior_sigma.setConstant(inf);
    solver.solve(opt);
    const float unregularized = solver.x[gain_sym_col];

    // Fit B: identical, but that one parameter is pinned hard to nominal.
    for (int k = 0; k < N_FRAMES; ++k) solver.frames[k] = saved[k];
    solver.x.setZero();
    solver.prior_sigma.setConstant(inf);
    solver.prior_sigma[gain_sym_col] = 1.0e-4f;
    solver.solve(opt);
    const float regularized = solver.x[gain_sym_col];

    char msg[224];
    snprintf(msg, sizeof(msg), "gain_sym: unregularized %.5f, tightly regularized %.5f",
             unregularized, regularized);
    TEST_MESSAGE(msg);

    // The data alone should find roughly the value that generated it.
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(unregularized - 0.05f) < 0.02f,
                              "unregularized fit did not recover the generating value");

    // And the prior must pull it in, not push it out. A sign error in
    // add_prior's rhs term inverts exactly this.
    snprintf(msg, sizeof(msg),
             "prior did not pull toward nominal: |%.5f| should be well under |%.5f|",
             regularized, unregularized);
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(regularized) < 0.2f * std::fabs(unregularized), msg);
    snprintf(msg, sizeof(msg),
             "prior overshot past nominal (%.5f vs %.5f) -- suggests a sign error",
             regularized, unregularized);
    TEST_ASSERT_TRUE_MESSAGE(regularized * unregularized >= 0.0f, msg);
}

// The specific claim the unregularized magnet_strength_mean rests on: leaving
// one near-degenerate parameter without a prior is safe *because* everything
// it trades against is regularized. A flat direction only survives if it lies
// entirely within unregularized coordinates, and this one does not.
//
// If that reasoning were wrong, H would be singular along the strength/
// position/gain combination and the step would come out non-finite -- which
// BundleSolver rejects, so the symptom would be a fit that never accepts a
// step rather than a crash.
void test_unregularized_strength_mean_stays_well_posed(void) {
    solver.geometry = make_geometry();
    make_poses(solver.frames);
    generate_measurements(make_truth());

    solver.x.setZero();
    solver.prior_sigma = nominal_prior_sigma();

    char msg[224];
    TEST_ASSERT_TRUE_MESSAGE(std::isinf(solver.prior_sigma[COL_STRENGTH_MEAN]),
                              "magnet_strength_mean should carry no prior");

    BundleSolverOptions opt;
    opt.max_iterations = 60;
    BundleSolverReport report = solver.solve(opt);

    snprintf(msg, sizeof(msg),
             "with nominal priors: accepted %d/%d, cost %.4e -> %.4e, strength_mean %+.4f (rel)",
             report.accepted_steps, report.iterations,
             report.initial_cost, report.final_cost, solver.x[COL_STRENGTH_MEAN]);
    TEST_MESSAGE(msg);

    TEST_ASSERT_TRUE_MESSAGE(solver.x.allFinite(), "fit produced non-finite parameters");
    TEST_ASSERT_TRUE_MESSAGE(report.accepted_steps > 0,
                              "no step accepted -- H may be singular along the "
                              "unregularized strength direction");
    TEST_ASSERT_TRUE_MESSAGE(report.final_cost < report.initial_cost * 0.01f,
                              "fit did not make substantial progress under nominal priors");

    // The stronger claim, now that it holds: the unregularized parameter is
    // not merely well-posed, it is well-DETERMINED. Its degeneracy partners
    // being pinned by their own priors is what leaves the data free to fix
    // the field scale, which is the whole argument for carrying no prior here.
    const float truth_strength_mean = make_truth()[COL_STRENGTH_MEAN];
    const float recovered = solver.x[COL_STRENGTH_MEAN];
    snprintf(msg, sizeof(msg),
             "unregularized strength_mean recovered %+.4f against truth %+.4f",
             recovered, truth_strength_mean);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(
        std::fabs(recovered - truth_strength_mean) < 0.2f * std::fabs(truth_strength_mean),
        msg);
}

// ======================================================================
// Unity entry points
// ======================================================================
void setUp(void) {}
void tearDown(void) {}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_solver_fits_synthetic_data);
    RUN_TEST(test_solver_leaves_correct_poses_alone);
    RUN_TEST(test_prior_pulls_toward_nominal);
    RUN_TEST(test_unregularized_strength_mean_stays_well_posed);
    UNITY_END();
}

void loop() {}
