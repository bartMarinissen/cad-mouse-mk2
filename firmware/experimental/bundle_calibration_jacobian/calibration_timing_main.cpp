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
// Frames are REAL captured readings, embedded by generate_calibration_frames.py
// from magnet_field_model/calibration_runs/. An earlier version generated them
// from the forward model at a synthetic truth vector, which makes the solver
// fit its own predictions -- no sensor noise, no model mismatch, no unmodelled
// cross-magnet term, and a data residual that can reach exactly zero. Timing
// was representative; convergence behaviour was not.
//
// The whole flow is timed, because all of it is work a real calibration does:
//
//   1. seed each frame's pose with solve_knob_pose against nominal geometry
//      (what calibration_algorithm.py does before its joint solve)
//   2. bundle-adjust the shared parameters and all poses jointly
//
// With real data there is no ground truth to compare parameters against, so
// what is reported is the residual before and after, which is the number that
// actually says whether the fit helped.

#include <Arduino.h>

#include "magnet_model/BicubicField.h"
#include "magnet_model/positions.h"
#include "magnet_model/forward_model.h"
#include "magnet_model/solve_pose.h"
#include "bundle_solver.h"
#include "calibration_frames_data.h"

static constexpr int N_FRAMES = CALIBRATION_RUN_FRAMES;

// Statically allocated: tens of KB, far past what the stack could hold.
static BundleSolver<N_FRAMES> solver;

static void set_nominal_geometry() {
    solver.geometry.sensor_pos[0] = Positions::sensor_1_world;
    solver.geometry.sensor_pos[1] = Positions::sensor_2_world;
    solver.geometry.sensor_pos[2] = Positions::sensor_3_world;
    solver.geometry.magnet_nominal_pos[0] = Positions::Magnet_1_knob;
    solver.geometry.magnet_nominal_pos[1] = Positions::Magnet_2_knob;
    solver.geometry.magnet_nominal_pos[2] = Positions::Magnet_3_knob;
    for (int m = 0; m < N_MAGNETS; ++m) {
        solver.geometry.magnet_nominal_rot[m] = Mat3::Identity();
    }
    // Signed: see MAGNET_POLARITY in bundle_param_layout.h.
    solver.geometry.magnet_nominal_strength_mT =
        MAGNET_POLARITY * BICUBIC_FIELD_REFERENCE_MT;
}

static void load_measurements() {
    for (int k = 0; k < N_FRAMES; ++k) {
        for (int i = 0; i < N_SENSORS; ++i) {
            solver.frames[k].measured[i] = Vec3(CALIBRATION_RUN_READINGS[k][3 * i + 0],
                                                 CALIBRATION_RUN_READINGS[k][3 * i + 1],
                                                 CALIBRATION_RUN_READINGS[k][3 * i + 2]);
        }
    }
}

// Nominal forward model, for seeding. Static because ForwardModel owns three
// VirtualSensors and three MagnetModels by value.
static VirtualSensor nominal_sensors[3] = {
    VirtualSensor(Positions::sensor_1_world),
    VirtualSensor(Positions::sensor_2_world),
    VirtualSensor(Positions::sensor_3_world),
};

// Seeds every frame's pose against nominal geometry, and reports how long that
// costs and how well nominal alone explains the data. That second number is
// the baseline the bundle fit has to beat.
static uint32_t seed_poses(float& rms_out, int& failed_out) {
    solver.x.setZero();
    solver.rebuild_from_params(solver.x);

    static MagnetModel nominal_magnets[3] = {
        build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[0]),
        build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[1]),
        build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[2]),
    };
    static ForwardModel nominal_model(nominal_sensors, nominal_magnets);

    float sum_sq = 0.0f;
    int failed = 0;
    const uint32_t t0 = millis();
    for (int k = 0; k < N_FRAMES; ++k) {
        solver.frames[k].t = Positions::approx_rest_pos;
        solver.frames[k].R = Mat3::Identity();
        const float residual = solve_knob_pose(solver.frames[k].t, solver.frames[k].R,
                                                nominal_model, solver.frames[k].measured);
        if (!isfinite(residual) || residual > 50.0f) ++failed;
        sum_sq += residual * residual;
    }
    const uint32_t elapsed = millis() - t0;
    rms_out = sqrtf(sum_sq / (N_FRAMES * N_SENSORS * 3));
    failed_out = failed;
    return elapsed;
}

static void time_components() {
    solver.rebuild_from_params(solver.x);
    const BundleFrame& frame0 = solver.frames[0];
    constexpr int REPS = 100;
    uint32_t t0;

    t0 = micros();
    for (int r = 0; r < REPS; ++r) {
        VirtualSensor sensor(solver.geometry.sensor_pos[0]);
        MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, solver.magnets[0]);
        Vec3 B; Eigen::Matrix<float, 3, 6> J_pose;
        sensor.evaluate(magnet, frame0.t, frame0.R, B, J_pose);
        asm volatile("" :: "r"(B.data()) : "memory");
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
    const float us_solve = (float)(micros() - t0) / 20.0f;

    const float per_iter_ms =
        (N_FRAMES * (us_build_full + us_absorb + us_build_pose) + us_solve) / 1000.0f;

    Serial.println();
    Serial.println("--- component timings (us, averaged) ---");
    Serial.printf("  VirtualSensor::evaluate (1 sensor)   %9.1f\n", us_forward);
    Serial.printf("  build_frame, pass 1 (3 sensors)      %9.1f\n", us_build_full);
    Serial.printf("  build_frame, pass 2 (3 sensors)      %9.1f\n", us_build_pose);
    Serial.printf("  absorb_frame (Schur fold)            %9.1f\n", us_absorb);
    Serial.printf("  %dx%d LDLT solve                      %9.1f\n",
                  N_SHARED_PARAMS, N_SHARED_PARAMS, us_solve);
    Serial.println();
    Serial.printf("  projected per LM iteration:          %9.1f ms\n", per_iter_ms);
    Serial.printf("  of which forward model:              %9.1f %%\n",
                  100.0f * (N_FRAMES * 6 * us_forward) / (per_iter_ms * 1000.0f));
}

void setup() {
    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 5000) { delay(10); }
    delay(500);

    Serial.println();
    Serial.println("=== bundle calibration timing harness ===");
    Serial.println("frames: REAL capture, see calibration_frames_data.h");
    Serial.printf("frames %d, shared parameters %d, sizeof(solver) %u bytes\n",
                  N_FRAMES, N_SHARED_PARAMS, (unsigned)sizeof(solver));

    set_nominal_geometry();
    load_measurements();
    solver.prior_sigma = nominal_prior_sigma();
    solver.observation_sigma = 1.0f;   // mT; sensor noise is not characterised here

    // --- stage 1: seed poses against nominal geometry --------------------
    float seed_rms = 0.0f;
    int seed_failed = 0;
    const uint32_t seed_ms = seed_poses(seed_rms, seed_failed);
    Serial.println();
    Serial.println("--- pose seeding (nominal geometry) ---");
    Serial.printf("  wall time            %lu ms  (%.1f ms/frame)\n",
                  (unsigned long)seed_ms, (float)seed_ms / N_FRAMES);
    Serial.printf("  RMS residual         %.4f mT   <-- baseline to beat\n", seed_rms);
    Serial.printf("  frames not solved    %d\n", seed_failed);

    time_components();

    // --- stage 2: the joint fit ------------------------------------------
    solver.x.setZero();
    BundleSolverOptions opt;
    opt.max_iterations = 60;

    Serial.println();
    Serial.println("--- full solve ---");
    const uint32_t t0 = millis();
    BundleSolverReport report = solver.solve(opt);
    const uint32_t elapsed_ms = millis() - t0;

    const float rms = sqrtf(report.final_cost / (N_FRAMES * N_SENSORS * 3));
    Serial.printf("  wall time            %lu ms\n", (unsigned long)elapsed_ms);
    Serial.printf("  iterations           %d  (Gauss-Newton; every step taken)\n",
                  report.iterations);
    Serial.printf("  per iteration        %.1f ms\n",
                  (float)elapsed_ms / (report.iterations > 0 ? report.iterations : 1));
    Serial.printf("  cost                 %.6e -> %.6e\n",
                  report.initial_cost, report.final_cost);
    Serial.printf("  RMS residual         %.4f mT  (from %.4f, %.2fx better)\n",
                  rms, seed_rms, seed_rms / (rms > 0.0f ? rms : 1.0f));
    Serial.printf("  converged            %s\n", report.converged ? "yes" : "no (hit a limit)");

    Serial.println();
    Serial.println("--- fitted shared parameters (offsets from nominal) ---");
    Serial.printf("  magnet_pos      %+.4f %+.4f %+.4f  mm-ish (gauge basis)\n",
                  solver.x[COL_MAGNET_POS + 0], solver.x[COL_MAGNET_POS + 1],
                  solver.x[COL_MAGNET_POS + 2]);
    Serial.printf("  strength_mean   %+.4f  (relative; unregularized)\n",
                  solver.x[COL_STRENGTH_MEAN]);
    Serial.printf("  strength_diff   %+.4f %+.4f\n",
                  solver.x[COL_STRENGTH_DIFF + 0], solver.x[COL_STRENGTH_DIFF + 1]);
    for (int m = 0; m < N_MAGNETS; ++m) {
        Serial.printf("  magnet_tilt[%d]  %+.4f %+.4f\n", m,
                      solver.x[COL_MAGNET_TILT + 2 * m], solver.x[COL_MAGNET_TILT + 2 * m + 1]);
    }
    for (int i = 0; i < N_SENSORS; ++i) {
        const int b = unit_block_start(i);
        Serial.printf("  sensor[%d] offset %+.3f %+.3f %+.3f mT\n", i,
                      solver.x[b + UNIT_OFFSET_SENSOR_OFFSET + 0],
                      solver.x[b + UNIT_OFFSET_SENSOR_OFFSET + 1],
                      solver.x[b + UNIT_OFFSET_SENSOR_OFFSET + 2]);
    }
    Serial.println();
    Serial.println("done.");
}

void loop() {
    delay(1000);
}
