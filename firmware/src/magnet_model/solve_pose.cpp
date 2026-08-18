#include "magnet_model/solve_pose.h"
#include <magnet_model/forward_model.h>
#include "math3D.h"

// Only used in this file, so kept local rather than promoted to math3D.h
// (TODO/eigen-to-bla-migration.md's "one owner per fact" rule applies to
// shared facts, not single-use-site ones).
using Vector6f = BLA::Matrix<6, 1, float>;
using Matrix6x6f = BLA::Matrix<6, 6, float>;

float __not_in_flash_func(solve_knob_pose)(
    Vec3& t,                           // In/Out: Current translation guess
    Mat3& R,                           // In/Out: Current rotation matrix guess
    const ForwardModel& model,         // Your evaluated forward model
    const Vector9f &measured_fields,   // The 9x1 vector of Hall sensor readings
    Vector9f *residual_out
) {
    const int MAX_ITER = 10;
    const float TOLERANCE = 3e-3f; // Stop if the update step is smaller than this
    // We run a weak version of LM where we always damp a fixed amount. This is
    // the factor by which we damp. This is not a well-chosen constant. We should
    // look at this better.
    const float LM_FIXED_DAMPING = 0.02f;

    Matrix9x6f jacobian;
    // Work directly in the caller's buffer for the residual whenever it supplied them.
    Vector9f residual_local;
    Vector9f &residual = (residual_out != nullptr) ? *residual_out : residual_local;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // 1. Evaluate forward model
        model.evaluate(t, R, residual, jacobian);
        // Calculate the residual. Both operands are genuine Vector9f lvalues here (not
        // Submatrix() sub-block views), so BLA's -= (a free function requiring a
        // non-const lvalue) binds fine -- see TODO/eigen-to-bla-migration.md's note on
        // where that restriction actually bites.
        residual -= measured_fields;

        // 2. Construct Damped Normal Equations (Levenberg-Marquardt).
        // H = J^T J. This project builds at -O3 project-wide, where GCC fully
        // unrolls this multiply and value-numbers H(i,j)/H(j,i) as the same
        // expression -- landing on the exact same soft-float call count as a
        // hand-written 21-dot-product lower-triangle-only version (see
        // TODO/Performance.md's "-O3" pass, re-verified byte-for-byte in the
        // cross-magnet-refactor pass -- unaffected since jacobian's 9x6 shape
        // never changed), so the general form is kept.
        Matrix6x6f H = ~jacobian * jacobian;
        for (int i = 0; i < 6; ++i) H(i, i) += LM_FIXED_DAMPING;
        Vector6f g = -(~jacobian * residual);

        // 3. Solve the 6x6 linear system. CholeskyDecompose factorizes H
        // in place (it's rebuilt fresh every iteration, so nothing downstream
        // reads the pre-factorization H again) and reports positive_definite
        // explicitly rather than degrading silently the way Eigen's LDLT
        // did -- H = J^T J + LM_FIXED_DAMPING*I is SPD by construction as long as
        // nothing upstream is already NaN/Inf, so treat a failure here the
        // same as the allFinite() check below.
        auto chol = BLA::CholeskyDecompose(H);
        if (!chol.positive_definite) {
            // Math collapsed (NaN or Inf, or a non-SPD H). Reject update and abort solver.
            Serial.println("NAN ERROR");
            delay(1000);
            break;
        }
        Vector6f dx = BLA::CholeskySolve(chol, g);

        if (!all_finite(dx)) {
            // Math collapsed (NaN or Inf). Reject update and abort solver.
            Serial.println("NAN ERROR");
            delay(1000);
            break;
        }

        // 4. Check for convergence - use squared norm to save a square root
        if (dot(dx, dx) < TOLERANCE * TOLERANCE) {
            break;
        }

        // 5. Extract updates
        Vec3 dt = dx.Submatrix<3, 1>(0, 0);
        Vec3 dw = dx.Submatrix<3, 1>(3, 0);

        // 6. Apply Updates
        t += dt;

        float w_norm = BLA::Norm(dw);
        if (w_norm > 1e-7f) {
            // Exactly matches your analytic Jacobian derivation: R_new = exp([w]x) * R_old
            R = exp_so3(dw) * R;
        }
    }

    // R is now caller-persisted state, hot-started back in on the next call
    // rather than reset to identity every time -- so unlike a value that's
    // rebuilt from scratch each call, small float error in the per-iteration
    // rotation update above can accumulate across many thousands of calls.
    // The whole forward-model/Jacobian chain assumes R^T == R^-1, so this
    // matters for correctness, not just cosmetics. One correction per call
    // (not per iteration -- drift within a single solve's handful of
    // iterations is negligible; it's the cross-call accumulation that isn't)
    // is cheap enough to apply unconditionally rather than track how much
    // drift has actually built up.
    R = orthonormalize_approx(R);

    return BLA::Norm(residual);
}
