#include <ArduinoEigenDense.h>
#include <iostream>
#include <magnet_model/forward_model.h>

// Use fixed-size types to guarantee zero heap allocation on the RP2040
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;
using Matrix6x6f = Eigen::Matrix<float, 6, 6>;

void solve_knob_pose(
    Eigen::Vector3f& t,                // In/Out: Current translation guess
    Eigen::Matrix3f& R,                // In/Out: Current rotation matrix guess
    const ForwardModel& model,         // Your evaluated forward model
    const Vec3 measured_fields[3]    // The 9x1 vector of Hall sensor readings
) {
    const int MAX_ITER = 5;
    const float TOLERANCE = 3e-3f; // Stop if the update step is smaller than this

    Vector9f r;        // Residual vector
    Matrix9x6f J;      // Jacobian matrix

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        Serial.printf("iteration %i", iter);
        
        // 1. Evaluate the forward model at the current state (t, R)
        // This populates the 9x1 residual and 9x6 Jacobian
        model.evaluate(t, R, measured_fields, r, J);

        // 2. Construct the Normal Equations (H * dx = g)
        Matrix6x6f H = J.transpose() * J;
        Vector6f g = -J.transpose() * r;

        // 3. Solve the 6x6 linear system for the update step dx = [dt, dw]^T
        // LDLT (Robust Cholesky) is extremely fast and safe for positive semi-definite matrices
        Vector6f dx = H.ldlt().solve(g);

        // 4. Check for convergence
        if (dx.norm() < TOLERANCE) {
            // Optimizer has settled
            break; 
        }

        // 5. Extract the translation and rotation updates
        Eigen::Vector3f dt = dx.head<3>();
        Eigen::Vector3f dw = dx.tail<3>();

        // 6. Apply Updates (Manifold Retraction)
        
        // Translation is a simple linear addition
        t += dt;

        // Rotation requires exponential mapping (Rodrigues' formula).
        // We guard against dw = 0 to prevent a divide-by-zero in the axis normalization.
        float angle = dw.norm();
        if (angle > 1e-7f) {
            // Eigen::AngleAxisf creates an exact rotation matrix from the axis-angle vector
            Eigen::AngleAxisf dR(angle, dw / angle);
            
            // Apply the local rotation update to the global rotation matrix
            R = (dR * R).eval();
            
            // Optional but recommended: Orthonormalize R every few iterations 
            // to prevent floating-point drift from distorting the rotation matrix.
            //R = R.householderQr().householderQ(); 
        }
    }
}