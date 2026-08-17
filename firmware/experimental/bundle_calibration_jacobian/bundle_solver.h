#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (see README.md).
//
// The Levenberg-Marquardt outer loop: the piece that turns the accumulator
// (schur_normal_equations.h) and the row assembly (bundle_param_layout.h) into
// something that actually fits. Derivation of the block structure it exploits
// is design documentation/Math.md section 7.
//
// --- The linear solve is a plain dense LDLT, on purpose --------------------
//
// H_ss is itself bordered block-diagonal (a 12-wide border and three 11-wide
// per-sensor blocks -- see bundle_param_layout.h), so its pose-style Schur
// elimination could be applied a second time, reducing the 45x45 solve to a
// 12x12 one. That is deliberately NOT done: the P x P factorization is ~P^3/3
// = 30k MAC against ~1.09M for the accumulation it sits on top of, so
// eliminating it optimizes about 3% of an iteration while adding a second,
// differently-shaped elimination to get right and keep verified. Eigen's
// fixed-size LDLT is already no-malloc and already the primitive used
// elsewhere here.
//
// The block structure still earns its keep -- it is what makes the
// ACCUMULATION sparse, which is where the real cost is. Structure for
// accumulation, dense solve.
//
// --- Memory: the stack is the binding constraint, and it is small ---------
//
// Measured, because an earlier version of this comment got all three numbers
// wrong. memmap_default.ld gives core0 a stack at the top of SCRATCH_Y:
// PICO_STACK_SIZE = 0x800, so 2,048 bytes RESERVED (__StackTop 0x20042000 down
// to __StackBottom 0x20041800). SCRATCH_Y is 4 KB and .scratch_y measures 0
// bytes in the real firmware, so ~4 KB is reachable in practice -- below that
// is __StackOneTop, core1's stack. RP2040 does have an MPU (__MPU_PRESENT 1);
// what it does not have is stack guards, since PICO_USE_STACK_GUARDS defaults
// to 0 and the Arduino core does not enable it. So overflow is unguarded, and
// it corrupts core1's stack rather than faulting.
//
// sizeof(BundleSolver<30>) is ~22.7 KB, so it MUST have static storage
// duration -- a local is a 22,712-byte frame, confirmed by disassembly.
//
// But that is the easy half, and saying only that was the mistake. Frames NEST,
// and this code's own internals were far over budget while every caller was
// doing the documented thing: build_frame at 19,256 bytes (resetting an
// accumulator via `*this = T()` builds a full-size temporary) and solve at
// 12,536 (Eigen materializing P x P products; H.ldlt() putting an 8.1 KB
// factorization on the stack). Both now fixed -- see the setZero()/noalias()/
// solveInPlace notes in schur_normal_equations.h -- but the general point is
// that this cannot be maintained by comment. verify_arm.sh gates on
// -Wstack-usage; run it after touching anything here.
//
// Remaining peak is roughly 3.7-4 KB along the deepest path (absorb_frame plus
// Eigen's blocked GEMM/triangular kernels beneath it), which fits the 4 KB
// region only because core1 is idle and .scratch_y is empty. That is not
// margin, and TODO/on-device-calibration.md tracks it as open.

#include "math3D.h"
#include "bundle_param_layout.h"
#include "schur_normal_equations.h"

// One captured frame: the 9 raw sensor readings, plus the pose estimate for
// that frame, which is refined alongside the shared parameters.
struct BundleFrame {
    Vec3 measured[N_SENSORS];   // raw readings, mT
    Vec3 t = Vec3::Zero();      // knob translation for this frame
    Mat3 R = Mat3::Identity();  // knob rotation for this frame
};

// Fixed per-unit inputs the fit does not solve for.
struct BundleGeometry {
    Vec3 sensor_pos[N_SENSORS];
    Vec3 magnet_nominal_pos[N_MAGNETS];
    Mat3 magnet_nominal_rot[N_MAGNETS];
    float magnet_nominal_strength_mT = 0.0f;
};

struct BundleSolverOptions {
    int max_iterations = 30;
    float initial_lambda = 1.0e-3f;
    float lambda_up = 10.0f;
    float lambda_down = 0.1f;
    // Converged when a successful step moves the shared parameters by less
    // than this, in the parameters' own (mixed) units.
    float shared_step_tolerance = 1.0e-5f;
    // ...or when an accepted step improves the cost by less than this
    // FRACTION of it. Needed as well as the step test, not instead: once the
    // fit is essentially exact, further steps stop being accepted at all, so
    // a step-size-only criterion never fires and the loop instead terminates
    // by damping itself to a standstill -- reporting "did not converge" about
    // a fit that had already reached 2e-4 mT RMS.
    float cost_tolerance = 1.0e-6f;
};

struct BundleSolverReport {
    int iterations = 0;
    int accepted_steps = 0;
    float initial_cost = 0.0f;
    float final_cost = 0.0f;
    bool converged = false;
};

template <int N_FRAMES>
struct BundleSolver {
    // --- state the fit owns -------------------------------------------
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> x =
        Eigen::Matrix<float, N_SHARED_PARAMS, 1>::Zero();   // offsets from nominal
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> prior_sigma =
        Eigen::Matrix<float, N_SHARED_PARAMS, 1>::Constant(1.0e6f);
    float observation_sigma = 1.0f;   // mT; scales residuals and Jacobian rows

    BundleGeometry geometry;
    BundleFrame frames[N_FRAMES];

    // --- scratch, sized to the problem and deliberately not on the stack --
    SharedNormalEquations<N_SHARED_PARAMS> normal_equations;
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> dx_shared;
    FrameNormalEquations<N_SHARED_PARAMS> frame_system;   // pass 1, reused
    FramePoseBlock<N_SHARED_PARAMS> pose_system;          // pass 2, reused
    // Trial state, also members rather than locals in solve(): at 30 frames
    // these are 2.5 KB and 720 bytes, and the stack is 2 KB total.
    BundleFrame trial_frames[N_FRAMES];
    Eigen::Matrix<float, 6, 1> dx_pose[N_FRAMES];
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> x_trial;
    SharedRow shared_row;                          // one sensor's assembled row
    Mat3 delta_R;
    Eigen::Matrix<float, 3, 6> scaled_pose_jacobian;

    // Per-magnet trial state, rebuilt once per iteration from x. This is the
    // OUTER loop of the two-level structure bundle_shared_jacobian.h describes:
    // 3 model builds per iteration, not one per (frame, sensor).
    MagnetState magnets[N_MAGNETS];
    Eigen::Matrix<float, 3, 2> chart[N_MAGNETS];
    Mat3 gains[N_SENSORS];
    Vec3 offsets[N_SENSORS];

    // Rebuilds every derived per-unit quantity from the parameter vector.
    // Called once per trial point -- never inside the frame loop.
    void rebuild_from_params(const Eigen::Matrix<float, N_SHARED_PARAMS, 1>& params) {
        for (int m = 0; m < N_MAGNETS; ++m) {
            magnets[m].pos = geometry.magnet_nominal_pos[m]
                + MAGNET_POS_BASIS.block<3, 3>(3 * m, 0) * params.template segment<3>(COL_MAGNET_POS);

            // MULTIPLIER, not an mT offset: matches bundle_geometry.py's
            // `magnet_strength = 1.0 + offset`, and is the parameterization the
            // physical beliefs are actually stated in -- "magnets from one
            // batch are graded to ~1% of each other" and "the scale could be
            // 500-1500 mT" are both relative claims. Getting this wrong makes
            // bundle_params.py's prior sigmas transfer off by the nominal
            // strength, i.e. a factor of ~1000.
            const Vec2 diff = strength_diff_coefficients(m);
            magnets[m].strength_mT = geometry.magnet_nominal_strength_mT
                * (1.0f + params[COL_STRENGTH_MEAN]
                        + diff.dot(params.template segment<2>(COL_STRENGTH_DIFF)));

            const MagnetTilt tilt{params[COL_MAGNET_TILT + 2 * m],
                                   params[COL_MAGNET_TILT + 2 * m + 1]};
            magnets[m].rotation = rotation_from_tilt(geometry.magnet_nominal_rot[m], tilt);
            chart[m] = chart_jacobian(geometry.magnet_nominal_rot[m], tilt);
        }
        for (int i = 0; i < N_SENSORS; ++i) {
            const int base = unit_block_start(i);
            offsets[i] = params.template segment<3>(base + UNIT_OFFSET_SENSOR_OFFSET);
            gains[i] = Mat3::Identity();
            for (int k = 0; k < 8; ++k) {
                gains[i] += params[base + UNIT_OFFSET_GAIN + k] * GAIN_BASIS[k];
            }
        }
    }

    // Sum of squared sigma-scaled residuals over every frame and sensor, at
    // the currently-rebuilt parameters and the given poses. The quantity LM
    // accepts or rejects a step on.
    float cost(const BundleFrame (&f)[N_FRAMES]) {
        const float inv_sigma = 1.0f / observation_sigma;
        float total = 0.0f;
        for (int k = 0; k < N_FRAMES; ++k) {
            for (int i = 0; i < N_SENSORS; ++i) {
                VirtualSensor sensor(geometry.sensor_pos[i]);
                MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, magnets[i]);
                Vec3 B; Eigen::Matrix<float, 3, 6> J_pose; SharedJacobianBlock J_shared;
                sensor.evaluate(magnet, f[k].t, f[k].R, B, J_pose);
                const Vec3 r = (gains[i] * B + offsets[i] - f[k].measured[i]) * inv_sigma;
                total += r.squaredNorm();
            }
        }
        return total;
    }

    // Builds one frame's local system at the current linearization point.
    // Templated on the accumulator type so pass 1 (FrameNormalEquations, which
    // also carries the shared corner) and pass 2 (FramePoseBlock, which does
    // not and is 8x smaller) share this code rather than duplicating the
    // per-sensor assembly.
    template <typename FrameSystem>
    void build_frame(FrameSystem& out, const BundleFrame& f) {
        out.setZero();
        const float inv_sigma = 1.0f / observation_sigma;
        for (int i = 0; i < N_SENSORS; ++i) {
            VirtualSensor sensor(geometry.sensor_pos[i]);
            MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, magnets[i]);
            Vec3 B; Eigen::Matrix<float, 3, 6> J_pose; SharedJacobianBlock J_shared;
            evaluate_bundle_jacobian(sensor, magnet, f.t, f.R, B, J_pose, J_shared);

            shared_row.setZero();
            add_magnet_columns(shared_row, i, gains[i], geometry.magnet_nominal_strength_mT,
                                J_shared, chart[i]);   // PAIRED_ONLY
            add_sensor_columns(shared_row, i, B);
            shared_row *= inv_sigma;   // in place; `row * inv_sigma` as an
                                        // argument is another 3 x P temporary

            const Vec3 residual = (gains[i] * B + offsets[i] - f.measured[i]) * inv_sigma;
            // The pose Jacobian is pre-gain for the same reason the magnet
            // columns are -- the prediction is G*B, so the pose block carries
            // a factor of G too (Math.md 4.E).
            scaled_pose_jacobian.noalias() = gains[i] * J_pose;
            scaled_pose_jacobian *= inv_sigma;
            out.add_sensor(residual, scaled_pose_jacobian, shared_row);
        }
    }

    BundleSolverReport solve(const BundleSolverOptions& opt = {}) {
        BundleSolverReport report;
        float lambda = opt.initial_lambda;

        rebuild_from_params(x);
        float current_cost = cost(frames);
        report.initial_cost = current_cost;

        for (int iter = 0; iter < opt.max_iterations; ++iter) {
            report.iterations = iter + 1;

            // --- pass 1: accumulate, eliminating each frame's pose ---------
            rebuild_from_params(x);
            normal_equations.reset();
            for (int k = 0; k < N_FRAMES; ++k) {
                build_frame(frame_system, frames[k]);
                normal_equations.absorb_frame(frame_system);
            }
            normal_equations.add_prior(x, prior_sigma);

            // LM damping on the reduced system. Applied after the frames are
            // folded in, so this damps the shared parameters only; each
            // frame's pose block was already inverted during elimination.
            for (int j = 0; j < N_SHARED_PARAMS; ++j) {
                normal_equations.H(j, j) *= (1.0f + lambda);
            }

            normal_equations.solve_into(dx_shared);
            if (!dx_shared.allFinite()) { lambda *= opt.lambda_up; continue; }

            // --- pass 2: back-substitute each frame's own pose update ------
            // Rebuilt, not stored: see SCHUR_SOLVER_DESIGN.md. Must linearize
            // at the SAME point pass 1 did, which is why nothing below is
            // applied to x or to any frame until every frame is done.
            for (int k = 0; k < N_FRAMES; ++k) {
                build_frame(pose_system, frames[k]);
                dx_pose[k] = solve_frame_pose_update(pose_system, dx_shared);
            }

            // --- trial step -----------------------------------------------
            x_trial = x + dx_shared;
            for (int k = 0; k < N_FRAMES; ++k) {
                trial_frames[k] = frames[k];
                trial_frames[k].t += dx_pose[k].template head<3>();
                delta_R = exp_so3_solver(dx_pose[k].template tail<3>());
                trial_frames[k].R.noalias() = delta_R * frames[k].R;
            }

            rebuild_from_params(x_trial);
            const float trial_cost = cost(trial_frames);

            if (trial_cost < current_cost) {
                const float relative_gain =
                    (current_cost - trial_cost) / (current_cost > 0.0f ? current_cost : 1.0f);
                x = x_trial;
                for (int k = 0; k < N_FRAMES; ++k) frames[k] = trial_frames[k];
                current_cost = trial_cost;
                lambda *= opt.lambda_down;
                report.accepted_steps++;
                if (dx_shared.norm() < opt.shared_step_tolerance
                    || relative_gain < opt.cost_tolerance) {
                    report.converged = true;
                    break;
                }
            } else {
                lambda *= opt.lambda_up;
                if (lambda > 1.0e8f) break;   // damped to a standstill
            }
        }

        rebuild_from_params(x);
        report.final_cost = cost(frames);
        return report;
    }

    // Exact exponential map, matching solve_pose.cpp's convention: a finite
    // step leaves R exactly orthogonal, so no re-orthonormalization is needed
    // (Math.md section 5, step 5).
    static Mat3 exp_so3_solver(const Vec3& w) {
        const float theta = w.norm();
        const Mat3 K = skew_matrix(w);
        if (theta < 1.0e-8f) return Mat3::Identity() + K + 0.5f * (K * K);
        const float s = std::sin(theta) / theta;
        const float c = (1.0f - std::cos(theta)) / (theta * theta);
        return Mat3::Identity() + s * K + c * (K * K);
    }
};
