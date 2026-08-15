#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (see README.md).
// Derivation: design documentation/Math.md §7 (why the normal equations are
// arrowhead-shaped, why eliminating each frame's pose block is an exact
// identity, why it streams). Engineering tradeoffs (store vs. recompute,
// memory budget): SCHUR_SOLVER_DESIGN.md. This file is the accumulator
// those two describe -- H_pp/H_sp/H_ss/rhs_p/rhs_s below are Math.md §7's
// D_k/B_k/A/b_k/a, one frame's terms except H_ss/rhs_s which are one
// frame's ADDITIVE CONTRIBUTION to the shared corner (see FrameNormalEquations).
//
// Sign/scaling convention, stated once here rather than re-derived at every
// call site: callers pass rows already divided by their sigma (matching
// bundle_params.py's residual()/jacobian(), which does the same before
// forming its least-squares problem), and "rhs" throughout is defined so
// that solve() returns dx meant to be ADDED to the current iterate --
// i.e. these are Gauss-Newton normal equations H @ dx = -J^T @ r, with the
// sign already folded into how add_sensor()/add_prior() accumulate rhs.

#include "math3D.h"

// Exactly one frame's ROW of the arrowhead: its own 6x6 pose block (H_pp),
// its P x 6 coupling to the shared parameters (H_sp), and its pose rhs
// (rhs_p). Both things that ever touch a frame's pose -- eliminating it
// (pass 1) and recovering it (pass 2) -- need these three and nothing else.
//
// Split out from FrameNormalEquations precisely because of that: pass 2's
// back-substitution never reads H_ss/rhs_s, and H_ss is by far the most
// expensive term to accumulate (3*P^2 MACs per sensor, vs 3*P*6 for H_sp).
// A pass-2 rebuild therefore builds one of THESE, not a full
// FrameNormalEquations -- skipping the dominant cost, and skipping a P x P
// stack temporary it would never read (~8 KB at P=45, vs ~1.2 KB here).
// Making that structural rather than a rule to remember is the point: there
// is no way to accidentally pay for H_ss in pass 2, and no way to forget it
// in pass 1.
//
// Built via 3 calls to add_sensor() (one per sensor -- matches
// evaluate_bundle_jacobian's per-sensor 3-row output). Never stored in an
// array across frames.
template <int P>
struct FramePoseBlock {
    Eigen::Matrix<float, 6, 6> H_pp = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, P, 6> H_sp = Eigen::Matrix<float, P, 6>::Zero();
    Eigen::Matrix<float, 6, 1> rhs_p = Eigen::Matrix<float, 6, 1>::Zero();

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
        rhs_p -= J_pose_scaled.transpose() * residual_scaled;
    }
};

// One frame's full contribution to the normal equations: its own row
// (the FramePoseBlock above) PLUS its additive contribution to the shared
// corner. Note H_ss/rhs_s are NOT "this frame's block" the way H_pp is --
// no frame owns the shared corner; each just adds into it, which is exactly
// why they can be dropped when only the frame's own row is wanted.
//
// Consumed exactly once, by SharedNormalEquations::absorb_frame() (pass 1).
// Pass 2 uses a freshly-rebuilt FramePoseBlock instead -- see
// SCHUR_SOLVER_DESIGN.md for why pass 2 rebuilds rather than storing pass
// 1's instances.
template <int P>
struct FrameNormalEquations {
    FramePoseBlock<P> pose;
    Eigen::Matrix<float, P, P> H_ss = Eigen::Matrix<float, P, P>::Zero();
    Eigen::Matrix<float, P, 1> rhs_s = Eigen::Matrix<float, P, 1>::Zero();

    // Same contract as FramePoseBlock::add_sensor -- forwards to it for the
    // three pose-row terms rather than restating them, then adds the two
    // shared-corner terms it exists to carry.
    void add_sensor(
        const Eigen::Matrix<float, 3, 1>& residual_scaled,
        const Eigen::Matrix<float, 3, 6>& J_pose_scaled,
        const Eigen::Matrix<float, 3, P>& J_shared_scaled
    ) {
        pose.add_sensor(residual_scaled, J_pose_scaled, J_shared_scaled);
        H_ss += J_shared_scaled.transpose() * J_shared_scaled;
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
        auto ldlt = f.pose.H_pp.ldlt();
        const Eigen::Matrix<float, 6, P> Hpp_inv_HspT = ldlt.solve(f.pose.H_sp.transpose());
        const Eigen::Matrix<float, 6, 1> Hpp_inv_rhsp = ldlt.solve(f.pose.rhs_p);
        H += f.H_ss - f.pose.H_sp * Hpp_inv_HspT;
        rhs += f.rhs_s - f.pose.H_sp * Hpp_inv_rhsp;
    }

    Eigen::Matrix<float, P, 1> solve() const {
        return H.ldlt().solve(rhs);
    }
};

// Pass 2: recovers one frame's own pose update, given a FRESHLY-REBUILT
// FramePoseBlock for that frame (see SCHUR_SOLVER_DESIGN.md for why pass 2
// rebuilds rather than storing pass 1's instances) and the shared update
// already solved from the accumulated system.
//
// Taking FramePoseBlock rather than FrameNormalEquations is what makes the
// cheap rebuild expressible: the caller cannot supply H_ss here, so it has
// no reason to have spent anything computing it.
//
// The rebuild MUST linearize at the same point pass 1 did -- same
// x_shared, same pose -- so dx_shared and every dx_pose have to be applied
// to the iterate only after this pass has finished for every frame.
// Otherwise H_sp/rhs_p here come from a different linearization than the
// reduced system that produced dx_shared, and the back-substitution stops
// being the exact identity it is derived as.
template <int P>
Eigen::Matrix<float, 6, 1> solve_frame_pose_update(
    const FramePoseBlock<P>& f, const Eigen::Matrix<float, P, 1>& dx_shared
) {
    auto ldlt = f.H_pp.ldlt();
    return ldlt.solve(f.rhs_p - f.H_sp.transpose() * dx_shared);
}
