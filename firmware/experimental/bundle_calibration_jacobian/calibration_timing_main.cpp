// Standalone timing harness for the on-device bundle calibration prototype.
//
//   pio run -e seeed_xiao_rp2040_caltiming -t upload
//   pio device monitor
//
// This REPLACES the normal firmware's main.cpp in that environment (see
// platformio.ini) -- it is not a unit test and does not run under `pio test`.
// It exists to answer the one question host builds cannot: how long does a
// calibration actually take on a soft-float M0+, and where does the time go.
//
// It prints a breakdown rather than a single number, because the interesting
// question is which term dominates. Per the cost model, one LM iteration is
// roughly 1.2M multiply-accumulates: ~547k accumulating H_ss, ~450k in the
// Schur folds, ~96k in pass 2, ~30k in the P x P solve, plus ~270 forward
// model evaluations. If that model is right, H_ss and the folds should be most
// of the wall time and the forward model should be a minority -- which would
// mean sparsity work matters more than making evaluate() faster. If it is
// wrong, better to find out from the device than to keep reasoning from
// instruction counts.
//
// Caveat worth knowing before trusting the total: peak stack depth is close to
// the 4 KB this core has, so a hang or a garbled reading is as likely to be a
// stack overflow as a slow solve. See design documentation/bundle-calibration.md.

#include <Arduino.h>

#include "magnet_model/BicubicField.h"
#include "magnet_model/positions.h"
#include "bundle_solver.h"

static constexpr int N_FRAMES = 30;

// Statically allocated: ~22.7 KB, far past what the stack could hold.
static BundleSolver<N_FRAMES> solver;

// Deterministic and tiny -- std::mt19937 would carry 2.5 KB of state for no
// benefit here. Any repeatable spread of poses does the job.
static uint32_t rng_state = 0x1234567u;
static float next_uniform(float lo, float hi) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return lo + (hi - lo) * (float)(rng_state >> 8) * (1.0f / 16777216.0f);
}

static Mat3 exp_so3_local(const Vec3& w) {
    const float theta = w.norm();
    const Mat3 K = skew_matrix(w);
    if (theta < 1.0e-8f) return Mat3::Identity() + K + 0.5f * (K * K);
    const float s = std::sin(theta) / theta;
    const float c = (1.0f - std::cos(theta)) / (theta * theta);
    return Mat3::Identity() + s * K + c * (K * K);
}

static void build_problem() {
    solver.geometry.sensor_pos[0] = Positions::sensor_1_world;
    solver.geometry.sensor_pos[1] = Positions::sensor_2_world;
    solver.geometry.sensor_pos[2] = Positions::sensor_3_world;
    solver.geometry.magnet_nominal_pos[0] = Positions::Magnet_1_knob;
    solver.geometry.magnet_nominal_pos[1] = Positions::Magnet_2_knob;
    solver.geometry.magnet_nominal_pos[2] = Positions::Magnet_3_knob;
    for (int m = 0; m < N_MAGNETS; ++m) solver.geometry.magnet_nominal_rot[m] = Mat3::Identity();
    solver.geometry.magnet_nominal_strength_mT = BICUBIC_FIELD_REFERENCE_MT;

    // Poses centred on the real rest pose. Off-table poses put the bicubic in
    // its extrapolation region, where the field is nonsense and the solver
    // stalls -- which would make this measure the wrong thing entirely.
    for (int k = 0; k < N_FRAMES; ++k) {
        solver.frames[k].t = Positions::approx_rest_pos
            + Vec3(next_uniform(-2.5f, 2.5f), next_uniform(-2.5f, 2.5f),
                    next_uniform(-3.0f, 2.0f));
        solver.frames[k].R = exp_so3_local(Vec3(next_uniform(-0.1f, 0.1f),
                                                 next_uniform(-0.1f, 0.1f),
                                                 next_uniform(-0.1f, 0.1f)));
    }

    // Synthetic truth, then measurements generated through the same forward
    // path the solver fits with.
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> truth;
    truth.setZero();
    truth.segment<3>(COL_MAGNET_POS) << 0.08f, -0.05f, 0.06f;
    truth[COL_STRENGTH_MEAN] = 0.06f;
    truth.segment<2>(COL_STRENGTH_DIFF) << 0.03f, -0.02f;
    for (int m = 0; m < N_MAGNETS; ++m) {
        truth[COL_MAGNET_TILT + 2 * m]     = 0.015f * (m + 1);
        truth[COL_MAGNET_TILT + 2 * m + 1] = -0.012f * (m + 1);
    }
    for (int i = 0; i < N_SENSORS; ++i) {
        const int base = unit_block_start(i);
        truth.segment<3>(base + UNIT_OFFSET_SENSOR_OFFSET) << 0.3f, -0.2f, 0.25f;
        for (int k = 0; k < 8; ++k) truth[base + UNIT_OFFSET_GAIN + k] = 0.004f * (k - 3);
    }

    solver.rebuild_from_params(truth);
    for (int k = 0; k < N_FRAMES; ++k) {
        for (int i = 0; i < N_SENSORS; ++i) {
            VirtualSensor sensor(solver.geometry.sensor_pos[i]);
            MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[i]);
            Vec3 B; Eigen::Matrix<float, 3, 6> J_pose;
            sensor.evaluate(magnet, solver.frames[k].t, solver.frames[k].R, B, J_pose);
            solver.frames[k].measured[i] = solver.gains[i] * B + solver.offsets[i];
        }
    }

    solver.x.setZero();
    solver.prior_sigma = nominal_prior_sigma();
    solver.observation_sigma = 1.0f;
}

// --- component timings ---------------------------------------------------
// Repeated and averaged: micros() has ~1us granularity and these are fast
// enough that a single call would be mostly quantisation noise.

static void time_components() {
    solver.rebuild_from_params(solver.x);
    const BundleFrame& frame0 = solver.frames[0];

    constexpr int REPS = 200;
    uint32_t t0;

    t0 = micros();
    for (int r = 0; r < REPS; ++r) {
        VirtualSensor sensor(solver.geometry.sensor_pos[0]);
        MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[0]);
        Vec3 B; Eigen::Matrix<float, 3, 6> J_pose;
        sensor.evaluate(magnet, frame0.t, frame0.R, B, J_pose);
        asm volatile("" :: "r"(B.data()) : "memory");   // don't optimise it away
    }
    const float us_forward = (float)(micros() - t0) / REPS;

    t0 = micros();
    for (int r = 0; r < REPS; ++r) {
        solver.build_frame(solver.frame_system, frame0);
        asm volatile("" :: "r"(solver.frame_system.H_ss.data()) : "memory");
    }
    const float us_build_full = (float)(micros() - t0) / REPS;

    t0 = micros();
    for (int r = 0; r < REPS; ++r) {
        solver.build_frame(solver.pose_system, frame0);
        asm volatile("" :: "r"(solver.pose_system.H_sp.data()) : "memory");
    }
    const float us_build_pose = (float)(micros() - t0) / REPS;

    solver.build_frame(solver.frame_system, frame0);
    t0 = micros();
    for (int r = 0; r < REPS; ++r) {
        solver.normal_equations.absorb_frame(solver.frame_system);
        asm volatile("" :: "r"(solver.normal_equations.H.data()) : "memory");
    }
    const float us_absorb = (float)(micros() - t0) / REPS;

    solver.normal_equations.reset();
    for (int k = 0; k < N_FRAMES; ++k) {
        solver.build_frame(solver.frame_system, solver.frames[k]);
        solver.normal_equations.absorb_frame(solver.frame_system);
    }
    solver.normal_equations.add_prior(solver.x, solver.prior_sigma);
    t0 = micros();
    for (int r = 0; r < 20; ++r) {
        solver.normal_equations.solve_into(solver.dx_shared);
        asm volatile("" :: "r"(solver.dx_shared.data()) : "memory");
    }
    const float us_solve45 = (float)(micros() - t0) / 20.0f;

    Serial.println();
    Serial.println("--- component timings (us, averaged) ---");
    Serial.printf("  VirtualSensor::evaluate (1 sensor)   %9.1f\n", us_forward);
    Serial.printf("  build_frame, pass 1 (3 sensors)      %9.1f\n", us_build_full);
    Serial.printf("  build_frame, pass 2 (3 sensors)      %9.1f\n", us_build_pose);
    Serial.printf("  absorb_frame (Schur fold)            %9.1f\n", us_absorb);
    Serial.printf("  45x45 LDLT solve                     %9.1f\n", us_solve45);
    Serial.println();
    Serial.printf("  projected per LM iteration:          %9.1f ms\n",
                  (N_FRAMES * (us_build_full + us_absorb + us_build_pose) + us_solve45) / 1000.0f);
    Serial.printf("  of which forward model:              %9.1f %%\n",
                  100.0f * (N_FRAMES * 6 * us_forward)
                      / (N_FRAMES * (us_build_full + us_absorb + us_build_pose) + us_solve45));
}

void setup() {
    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 5000) { delay(10); }
    delay(500);

    Serial.println();
    Serial.println("=== bundle calibration timing harness ===");
    Serial.printf("frames %d, shared parameters %d, sizeof(solver) %u bytes\n",
                  N_FRAMES, N_SHARED_PARAMS, (unsigned)sizeof(solver));

    build_problem();
    time_components();

    // --- the whole fit -------------------------------------------------
    solver.x.setZero();
    BundleSolverOptions opt;
    opt.max_iterations = 60;

    Serial.println();
    Serial.println("--- full solve ---");
    const uint32_t t0 = millis();
    BundleSolverReport report = solver.solve(opt);
    const uint32_t elapsed_ms = millis() - t0;

    Serial.printf("  wall time            %lu ms\n", (unsigned long)elapsed_ms);
    Serial.printf("  iterations           %d (%d accepted)\n",
                  report.iterations, report.accepted_steps);
    Serial.printf("  per iteration        %.1f ms\n",
                  (float)elapsed_ms / (report.iterations > 0 ? report.iterations : 1));
    Serial.printf("  cost                 %.6e -> %.6e\n",
                  report.initial_cost, report.final_cost);
    Serial.printf("  RMS residual         %.5f mT\n",
                  sqrtf(report.final_cost / (N_FRAMES * N_SENSORS * 3)));
    Serial.printf("  converged            %s\n", report.converged ? "yes" : "no (hit a limit)");
    Serial.printf("  strength_mean        %+.4f (relative; truth was +0.0600)\n",
                  solver.x[COL_STRENGTH_MEAN]);
    Serial.println();
    Serial.println("done.");
}

void loop() {
    delay(1000);
}
