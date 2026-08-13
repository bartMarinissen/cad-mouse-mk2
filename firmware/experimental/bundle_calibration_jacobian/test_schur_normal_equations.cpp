// test_schur_normal_equations.cpp -- PROTOTYPE, not wired into the build.
//
// Exact-algebra verification, not finite differences: Schur complement is a
// linear-algebra IDENTITY (eliminating a block of a linear system is exact,
// not an approximation), so there's a ground-truth answer to check against --
// assemble the FULL dense arrowhead system directly and solve it in one
// shot, then check the frame-by-frame Schur path (schur_normal_equations.h)
// gives the identical answer, to float32 precision. If the two disagree by
// more than roundoff, the accumulator shape has a real bug, not a modeling
// approximation to argue about.
//
// Uses a small, generic P and N (not the real bundle's P=45/N=60) --
// this is testing that the ALGEBRA is right for any arrowhead system of this
// shape, not exercising the real calibration's specific column layout (which
// doesn't exist yet -- see SCHUR_SOLVER_DESIGN.md's "what's deliberately not
// here").

#include <unity.h>
#include <random>
#include "math3D.h"
#include "schur_normal_equations.h"

static constexpr int P = 5;   // shared parameters -- arbitrary, small
static constexpr int N = 4;   // frames
static constexpr int FULL = P + 6 * N;

// Fixed-seed std::mt19937, not a hand-rolled sinusoid: an earlier version of
// this test generated "arbitrary" matrix entries as sin(linear combination
// of indices), which is exactly wrong for this purpose -- sin(x + shift) is
// always a fixed linear combination of sin(x)/cos(x), so every such matrix
// lives in a 2-dimensional subspace regardless of size, no matter how many
// indices feed the shift. That made every synthetic H_pp rank-deficient
// (measured: eigenvalues -0.0, -0.0, 0.0, 0.0, 0.16, 21.0 -- 2 real degrees
// of freedom, not 6), so the comparison below was checking two different
// solve paths on a singular system and failing not because the algebra was
// wrong, but because a singular system doesn't have a unique answer to
// agree on. A real PRNG doesn't have this failure mode. This class of bug
// is exactly why "close to Path B" isn't enough for this test -- it has to
// be checked that Path B's own inputs are actually well-posed, not just
// that two computations produced *some* numbers.
static std::mt19937 rng(12345);
static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

template <int ROWS, int COLS>
static Eigen::Matrix<float, ROWS, COLS> rand_matrix() {
    Eigen::Matrix<float, ROWS, COLS> m;
    for (int r = 0; r < ROWS; ++r)
        for (int c = 0; c < COLS; ++c)
            m(r, c) = dist(rng);
    return m;
}

void test_schur_matches_dense_reference(void) {
    // Per-frame, per-sensor synthetic data: 3 sensors x N frames, each
    // contributing a 3-row block -- mirrors evaluate_bundle_jacobian's
    // per-sensor 3-row output shape exactly, even though the values here
    // are arbitrary rather than physical.
    Eigen::Matrix<float, 3, 6> J_pose[N][3];
    Eigen::Matrix<float, 3, P> J_shared[N][3];
    Eigen::Matrix<float, 3, 1> residual[N][3];
    for (int f = 0; f < N; ++f) {
        for (int s = 0; s < 3; ++s) {
            J_pose[f][s] = rand_matrix<3, 6>();
            J_shared[f][s] = rand_matrix<3, P>();
            residual[f][s] = rand_matrix<3, 1>();
        }
    }

    // Well-posedness guard: each frame's H_pp (3 sensors x 3 rows = 9
    // observations of 6 pose unknowns) must actually be full rank for
    // "the Schur path agrees with the dense reference" to mean anything --
    // on a singular H_pp, both paths can independently produce *a* answer
    // without producing the *same* answer, which looks identical to a real
    // bug from the assertions below alone. This is exactly the failure mode
    // an earlier version of this test hit silently; check it explicitly
    // rather than trust the RNG not to reintroduce it.
    for (int f = 0; f < N; ++f) {
        Eigen::Matrix<float, 6, 6> H_pp_check = Eigen::Matrix<float, 6, 6>::Zero();
        for (int s = 0; s < 3; ++s) H_pp_check += J_pose[f][s].transpose() * J_pose[f][s];
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> es(H_pp_check);
        char msg[128];
        snprintf(msg, sizeof(msg), "frame %d H_pp smallest eigenvalue %.4f -- not well-posed",
                 f, es.eigenvalues()[0]);
        TEST_ASSERT_TRUE_MESSAGE(es.eigenvalues()[0] > 0.05f, msg);
    }

    // --- Path A: the Schur accumulator under test ---
    SharedNormalEquations<P> shared_eq;
    FrameNormalEquations<P> frames[N];   // kept only so pass 2 below doesn't
                                          // need to recompute for THIS test;
                                          // real usage rebuilds in pass 2 --
                                          // see SCHUR_SOLVER_DESIGN.md. Using
                                          // the same instances here still
                                          // exercises the identical algebra,
                                          // since absorb_frame() and
                                          // solve_frame_pose_update() don't
                                          // know or care whether their input
                                          // was just-built or rebuilt.
    for (int f = 0; f < N; ++f) {
        for (int s = 0; s < 3; ++s) {
            frames[f].add_sensor(residual[f][s], J_pose[f][s], J_shared[f][s]);
        }
        shared_eq.absorb_frame(frames[f]);
    }
    Eigen::Matrix<float, P, 1> dx_shared = shared_eq.solve();

    Eigen::Matrix<float, 6, 1> dx_pose[N];
    for (int f = 0; f < N; ++f) {
        dx_pose[f] = solve_frame_pose_update(frames[f], dx_shared);
    }

    // --- Path B: the dense reference -- full (P+6N) x (P+6N) system,
    // assembled directly from the same per-sensor blocks, solved in one shot.
    Eigen::Matrix<float, FULL, FULL> H_full = Eigen::Matrix<float, FULL, FULL>::Zero();
    Eigen::Matrix<float, FULL, 1> rhs_full = Eigen::Matrix<float, FULL, 1>::Zero();

    for (int f = 0; f < N; ++f) {
        Eigen::Matrix<float, 6, 6> H_pp = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, P, 6> H_sp = Eigen::Matrix<float, P, 6>::Zero();
        Eigen::Matrix<float, 6, 1> rhs_p = Eigen::Matrix<float, 6, 1>::Zero();
        for (int s = 0; s < 3; ++s) {
            H_pp += J_pose[f][s].transpose() * J_pose[f][s];
            H_sp += J_shared[f][s].transpose() * J_pose[f][s];
            H_full.block<P, P>(0, 0) += J_shared[f][s].transpose() * J_shared[f][s];
            rhs_p -= J_pose[f][s].transpose() * residual[f][s];
            rhs_full.block<P, 1>(0, 0) -= J_shared[f][s].transpose() * residual[f][s];
        }
        const int off = P + 6 * f;
        H_full.block<6, 6>(off, off) = H_pp;
        H_full.block<P, 6>(0, off) = H_sp;
        H_full.block<6, P>(off, 0) = H_sp.transpose();
        rhs_full.block<6, 1>(off, 0) = rhs_p;
    }
    Eigen::Matrix<float, FULL, 1> dx_full = H_full.ldlt().solve(rhs_full);

    // --- Compare ---
    char msg[192];
    float e_shared = (dx_shared - dx_full.block<P, 1>(0, 0)).norm() / dx_full.block<P, 1>(0, 0).norm();
    snprintf(msg, sizeof(msg), "dx_shared vs dense reference: rel err %.2e", e_shared);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(e_shared < 1e-4f, msg);

    for (int f = 0; f < N; ++f) {
        Eigen::Matrix<float, 6, 1> ref = dx_full.block<6, 1>(P + 6 * f, 0);
        float e_pose = (dx_pose[f] - ref).norm() / ref.norm();
        snprintf(msg, sizeof(msg), "frame %d dx_pose vs dense reference: rel err %.2e", f, e_pose);
        TEST_MESSAGE(msg);
        TEST_ASSERT_TRUE_MESSAGE(e_pose < 1e-4f, msg);
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
    RUN_TEST(test_schur_matches_dense_reference);
    UNITY_END();
}

void loop() {}
