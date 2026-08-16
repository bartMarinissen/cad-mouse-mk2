#include <magnet_model/forward_model.h>

// returns the magnitude of the residual
float solve_knob_pose(
    Vec3& t,                        // In/Out: Current translation guess
    Mat3& R,                        // In/Out: Current rotation matrix guess
    const ForwardModel& model,      // Your evaluated forward model
    const Vec3 measured_fields[3],  // The 9x1 vector of Hall sensor readings
    Vector9f *residual = nullptr,   // Optional residual out
    Matrix9x6f *Jacobian = nullptr  // Optional jacobian out
);
