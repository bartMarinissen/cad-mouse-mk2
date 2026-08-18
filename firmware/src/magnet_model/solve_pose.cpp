#include "magnet_model/solve_pose.h"
#include <ArduinoEigenDense.h>
#include <magnet_model/forward_model.h>
#include "math3D.h"


float __not_in_flash_func(solve_knob_pose)(
    Eigen::Vector3f& t,                // In/Out: Current translation guess
    Eigen::Matrix3f& R,                // In/Out: Current rotation matrix guess
    const ForwardModel& model,         // Your evaluated forward model
    const Vector9f &measured_fields, // The 9x1 vector of Hall sensor readings
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
        // Calculate the residual
        residual -= measured_fields;

        // 2. Construct Damped Normal Equations (Levenberg-Marquardt).
        // H = J^T J is symmetric, so only its lower triangle is worth computing:
        // 21 dot products of length 9 instead of a full 36-entry product, which
        // saves ~135 multiplies and ~120 adds per iteration. The upper triangle
        // is mirrored in so H stays a well-formed symmetric matrix for whatever
        // reads it next.
        Matrix6x6f H;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j <= i; ++j) {
                const float h = jacobian.col(i).dot(jacobian.col(j));
                H(i, j) = h;
                H(j, i) = h;
            }
        }
        H.diagonal().array() += LM_FIXED_DAMPING;
        Vector6f g = -jacobian.transpose() * residual;
        
        // 3. Solve the 6x6 linear system
        auto ldlt = H.ldlt();
        Vector6f dx = ldlt.solve(g);

        if (!dx.allFinite()) {
            // Math collapsed (NaN or Inf). Reject update and abort solver.
            Serial.println("NAN ERROR");
            delay(1000);
            break; 
        }

        // 4. Check for convergence - use squared norm to save a square root
        if (dx.squaredNorm() < TOLERANCE * TOLERANCE) {
            break; 
        }

        // 5. Extract updates
        Eigen::Vector3f dt = dx.head<3>();
        Eigen::Vector3f dw = dx.tail<3>();

        // 6. Apply Updates 
        t += dt;

        float w_norm = dw.norm();
        if (w_norm > 1e-7f) {
            // Recompute angle since we might have clamped dw
            float angle = w_norm;
            Eigen::AngleAxisf dR(angle, dw / angle);
            
            // Exactly matches your analytic Jacobian derivation: R_new = exp([w]x) * R_old
            R = (dR * R).eval();
        }
    }

    // R is now caller-persisted state, hot-started back in on the next call
    // rather than reset to identity every time -- so unlike a value that's
    // rebuilt from scratch each call, small float error in the per-iteration
    // AngleAxisf update above can accumulate across many thousands of calls.
    // The whole forward-model/Jacobian chain assumes R^T == R^-1, so this
    // matters for correctness, not just cosmetics. One correction per call
    // (not per iteration -- drift within a single solve's handful of
    // iterations is negligible; it's the cross-call accumulation that isn't)
    // is cheap enough to apply unconditionally rather than track how much
    // drift has actually built up.
    R = orthonormalize_approx(R);

    return residual.norm();
}
