#include <ArduinoEigenDense.h>
#include <magnet_model/forward_model.h>


using Vector9f = Eigen::Matrix<float, 9, 1>;

// returns the magnitude of the residual
float solve_knob_pose(
    Eigen::Vector3f& t,             // In/Out: Current translation guess
    Eigen::Matrix3f& R,             // In/Out: Current rotation matrix guess
    const ForwardModel& model,      // Your evaluated forward model
    const Vec3 measured_fields[3],  // The 9x1 vector of Hall sensor readings
    Vector9f *residual = nullptr,   // Optional residual out
    Matrix9x6f *Jacobian = nullptr  // Optional jacobian out
);
