#pragma once
#include "math3D.h"
#include "magnet_local_model.h"


struct VirtualSensor {
    Vec3 sensor_pos_global;

    VirtualSensor(Vec3 sensor_pos_global);

    void evaluate(const MagnetModel &magnet, const Vec3& t,
        const Mat3& R,
        Vec3 &B_field_global,
        Matrix3x6f &J) const;
};
