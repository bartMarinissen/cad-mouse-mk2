#include "magnet_model/solve_pose.h"
#include <ArduinoEigenDense.h>
#include <magnet_model/forward_model.h>

using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;
using Matrix6x6f = Eigen::Matrix<float, 6, 6>;

float __not_in_flash_func(solve_knob_pose)(
    Eigen::Vector3f& t,                // In/Out: Current translation guess
    Eigen::Matrix3f& R,                // In/Out: Current rotation matrix guess
    const ForwardModel& model,         // Your evaluated forward model
    const Eigen::Vector3f measured_fields[3], // The 9x1 vector of Hall sensor readings
    Vector9f *residual_out,
    Matrix9x6f *Jacobian_out
) {
    const int MAX_ITER = 10;
    const float TOLERANCE = 3e-3f; // Stop if the update step is smaller than this
    
    // TODO don't even allocate these if we were passed non-null residual_out / Jacobian_out.
    //      instead, in that case, pass in those pointers directly
    Vector9f residual;        
    Matrix9x6f jacobian;      

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // 1. Evaluate forward model (assuming it populates predicted fields)
        model.evaluate(t, R, residual, jacobian);
        residual.block<3, 1>(0, 0) -= measured_fields[0];
        residual.block<3, 1>(3, 0) -= measured_fields[1];
        residual.block<3, 1>(6, 0) -= measured_fields[2];

        // 2. Construct Damped Normal Equations (Levenberg-Marquardt)
        Matrix6x6f H = jacobian.transpose() * jacobian;
        const float LAMBDA = 0.02f;
        H.diagonal().array() += LAMBDA;
        Vector6f g = -jacobian.transpose() * residual;

        auto ldlt = H.ldlt();
        // 3. Solve the 6x6 linear system
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
    if (residual_out != nullptr)
        *residual_out = residual;
    if (Jacobian_out != nullptr)
        *Jacobian_out = jacobian;
    // TODO check jacobian well-formedness
    // TODO deal with residual
    return residual.norm();
}
