#include <ArduinoEigenDense.h>
#include <magnet_model/forward_model.h>

using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;
using Matrix6x6f = Eigen::Matrix<float, 6, 6>;

void solve_knob_pose(
    Eigen::Vector3f& t,                // In/Out: Current translation guess
    Eigen::Matrix3f& R,                // In/Out: Current rotation matrix guess
    const ForwardModel& model,         // Your evaluated forward model
    const Eigen::Vector3f measured_fields[3] // The 9x1 vector of Hall sensor readings
) {
    const int MAX_ITER = 20;
    const float TOLERANCE = 3e-3f; // Stop if the update step is smaller than this
    
    // Safety limits for 1/r^3 magnetic gradients
    const float MAX_TRANS_STEP = 1.5f; // Max 1.5mm movement per iteration
    const float MAX_ROT_STEP = 0.15f;  // Max ~8.5 degrees per iteration

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
        const float LAMBDA = 0.1f;
        H.diagonal().array() += LAMBDA;
        Vector6f g = -jacobian.transpose() * residual;

        // 3. Solve the 6x6 linear system
        Vector6f dx = H.ldlt().solve(g);

        // 4. Check for convergence
        if (dx.norm() < TOLERANCE) {
            break; 
        }

        // 5. Extract updates
        Eigen::Vector3f dt = dx.head<3>();
        Eigen::Vector3f dw = dx.tail<3>();

        // --- NEW: 5b. Step Clamping (Prevents flying out of interpolation bounds) ---
        float t_norm = dt.norm();
        if (t_norm > MAX_TRANS_STEP) {
            dt *= (MAX_TRANS_STEP / t_norm);
        }
        
        float w_norm = dw.norm();
        if (w_norm > MAX_ROT_STEP) {
            dw *= (MAX_ROT_STEP / w_norm);
        }
        // ------------------------------------------------------------------------

        // 6. Apply Updates 
        t += dt;

        if (w_norm > 1e-7f) {
            // Recompute angle since we might have clamped dw
            float angle = dw.norm(); 
            Eigen::AngleAxisf dR(angle, dw / angle);
            
            // Exactly matches your analytic Jacobian derivation: R_new = exp([w]x) * R_old
            R = (dR * R).eval();
        }
    }

    // Optional: Orthonormalize ONCE per frame outside the loop to prevent long-term drift
    // R = R.householderQr().householderQ(); 
}
