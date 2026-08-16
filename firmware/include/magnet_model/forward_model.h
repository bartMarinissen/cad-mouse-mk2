#pragma once

#include "CalibrationParams.h"
#include "math3D.h"
#include "magnet_local_model.h"
#include "virtual_sensor.h"

class ForwardModel {
public:
    // Initializes the model with the physical layout of the PCB and knob
    ForwardModel(const VirtualSensor (&sensors)[3], const MagnetModel (&magnets)[3]);

    // The layout as actually calibrated: sensors at their (fixed, world-frame
    // defining) nominal positions, magnets at their fitted knob-frame positions
    // and tilts. This is how the running firmware builds its model; the array
    // constructor above stays for tests that want to hand-place things.
    explicit ForwardModel(const CalibrationParams& cal);

    /**
     * Computes the 9x1 magnetic field per sensor and 9x6 Jacobian for a given pose of the knob.
     * 
     * The pose is given as: 
     *   t_world, the position of the knob origin in the world frame
     *   R, the rotation matrix needed to translate from the world frame to the knob frame
     */
    void evaluate(const Vec3& t_world, 
                  const Mat3& R, 
                  Eigen::Matrix<float, 9, 1> &B_field, 
                  Eigen::Matrix<float, 9, 6> &J) const;

private:
    const VirtualSensor sensors_[3];
    const MagnetModel magnets_[3];
};
