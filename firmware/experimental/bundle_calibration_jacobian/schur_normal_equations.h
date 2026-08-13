#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (see README.md).
// See SCHUR_SOLVER_DESIGN.md for the full derivation and the store-vs-
// recompute memory tradeoff this shape is built around. Short version: the
// bundle-calibration normal equations are arrowhead-shaped (dense P x P
// shared block, block-diagonal 6x6 per-frame pose blocks, thin P x 6
// coupling blocks) -- eliminating each frame's pose block via Schur
// complement, one frame at a time, turns an O((P+6N)^2) memory problem into
// O(P^2) persistent + O(P) transient, independent of N.
//
// Sign/scaling convention, stated once here rather than re-derived at every
// call site: callers pass rows already divided by their sigma (matching
// bundle_params.py's residual()/jacobian(), which does the same before
// forming its least-squares problem), and "rhs" throughout is defined so
// that solve() returns dx meant to be ADDED to the current iterate --
// i.e. these are Gauss-Newton normal equations H @ dx = -J^T @ r, with the
// sign already folded into how add_sensor()/add_prior() accumulate rhs.

#include "math3D.h"

// One frame's LOCAL system, before its own pose block is eliminated. Built
// via 3 calls to add_sensor() (one per sensor -- matches
// evaluate_bundle_jacobian's per-sensor 3-row output) and then consumed
// exactly once, either by SharedNormalEquations::absorb_frame() (pass 1) or
// by solve_frame_pose_update() (pass 2, on a freshly-rebuilt instance -- see
// SCHUR_SOLVER_DESIGN.md for why pass 2 rebuilds rather than reusing pass
// 1's instance). Never stored in an array across frames.
template <int P>
struct FrameNormalEquations {
    Eigen::Matrix<float, 6, 6> H_pp = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, P, 6> H_sp = Eigen::Matrix<float, P, 6>::Zero();
    Eigen::Matrix<float, P, P> H_ss = Eigen::Matrix<float, P, P>::Zero();
    Eigen::Matrix<float, 6, 1> rhs_p = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, P, 1> rhs_s = Eigen::Matrix<float, P, 1>::Zero();

    // Folds one sensor's already-sigma-scaled 3-row contribution in. Call
    // once per sensor (3x per frame). J_shared_scaled is the sensor's full
    // P-wide row -- i.e. already projected through whatever gauge basis
    // applies (MAGNET_POS_BASIS etc.) and laid out in the shared parameter
    // vector's actual column order; assembling that from
    // evaluate_bundle_jacobian's raw SharedJacobianBlock is a separate,
    // not-yet-built step (see SCHUR_SOLVER_DESIGN.md).
    void add_sensor(
        const Eigen::Matrix<float, 3, 1>& residual_scaled,
        const Eigen::Matrix<float, 3, 6>& J_pose_scaled,
        const Eigen::Matrix<float, 3, P>& J_shared_scaled
    ) {
        H_pp += J_pose_scaled.transpose() * J_pose_scaled;
        H_sp += J_shared_scaled.transpose() * J_pose_scaled;
        H_ss += J_shared_scaled.transpose() * J_shared_scaled;
        rhs_p -= J_pose_scaled.transpose() * residual_scaled;
        rhs_s -= J_shared_scaled.transpose() * residual_scaled;
    }
};

// The ONLY state that persists across the whole frame set during one solver
// iteration: the reduced P x P shared-parameter system, with every frame's
// pose already eliminated out of it. O(P^2), not O(P * n_frames).
template <int P>
struct SharedNormalEquations {
    Eigen::Matrix<float, P, P> H = Eigen::Matrix<float, P, P>::Zero();
    Eigen::Matrix<float, P, 1> rhs = Eigen::Matrix<float, P, 1>::Zero();

    void reset() { H.setZero(); rhs.setZero(); }

    // Ridge/prior regularization -- mirrors RegularizationSigmas in
    // bundle_params.py: a belief that each shared parameter's offset from
    // nominal (x_current) is 0 with the given prior stddev (sigma). Pass
    // sigma = infinity (or skip the call for that index) for an
    // unregularized parameter, matching sigma=None there.
    void add_prior(const Eigen::Matrix<float, P, 1>& x_current,
                    const Eigen::Matrix<float, P, 1>& sigma) {
        for (int i = 0; i < P; ++i) {
            const float inv_sigma2 = 1.0f / (sigma[i] * sigma[i]);
            H(i, i) += inv_sigma2;
            rhs[i] -= x_current[i] * inv_sigma2;
        }
    }

    // Pass 1: folds one frame's local system in via Schur elimination of
    // that frame's own 6-DOF pose block. f.H_pp's LDLT is the SAME
    // fixed-size solve solve_pose.cpp already does, at the same 6x6 size --
    // no new linear-algebra primitive, just reused here for a different
    // purpose (eliminating a block, not solving the whole frame's pose).
    void absorb_frame(const FrameNormalEquations<P>& f) {
        auto ldlt = f.H_pp.ldlt();
        const Eigen::Matrix<float, 6, P> Hpp_inv_HspT = ldlt.solve(f.H_sp.transpose());
        const Eigen::Matrix<float, 6, 1> Hpp_inv_rhsp = ldlt.solve(f.rhs_p);
        H += f.H_ss - f.H_sp * Hpp_inv_HspT;
        rhs += f.rhs_s - f.H_sp * Hpp_inv_rhsp;
    }

    Eigen::Matrix<float, P, 1> solve() const {
        return H.ldlt().solve(rhs);
    }
};

// Pass 2: recovers one frame's own pose update, given a FRESHLY-REBUILT
// FrameNormalEquations for that frame (see SCHUR_SOLVER_DESIGN.md for why
// pass 2 rebuilds rather than reusing pass 1's instance) and the shared
// update already solved from the accumulated system.
template <int P>
Eigen::Matrix<float, 6, 1> solve_frame_pose_update(
    const FrameNormalEquations<P>& f, const Eigen::Matrix<float, P, 1>& dx_shared
) {
    auto ldlt = f.H_pp.ldlt();
    return ldlt.solve(f.rhs_p - f.H_sp.transpose() * dx_shared);
}
