#pragma once
#include "math3D.h"
#include "magnet_local_model.h"


struct VirtualSensor {
    Vec3 sensor_pos_global;

    VirtualSensor(Vec3 sensor_pos_global);

    void evaluate(const MagnetModel &magnet, const Vec3& t,  
        const Mat3& R, 
        Eigen::Matrix<float, 3, 1> &B_field_global, 
        Eigen::Matrix<float, 3, 6> &J) const;
};
