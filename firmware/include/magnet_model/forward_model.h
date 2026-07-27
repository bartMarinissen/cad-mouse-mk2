#pragma once

#include "math3D.h"
#include "magnet_local_model.h"
#include "sensor.h"

class ForwardModel {
public:
    // Initializes the model with the physical layout of the PCB and knob
    ForwardModel(const Sensor (&sensors)[3], const MagnetModel (&magnets)[3]);

    // Computes the 9x1 residual and 9x6 Jacobian for a given pose
    // Note the pose for this jacobian is in terms of a translation vector t 
    //  and a rotation vector omega. But we caluclate it in terms of a rotation MATRIX R.
    void evaluate(const Vec3& t, 
                  const Mat3& R, 
                  Eigen::Matrix<float, 9, 1> &B_field, 
                  Eigen::Matrix<float, 9, 6> &J) const;

private:
    const Sensor sensors_[3];
    const MagnetModel magnets_[3];
};
