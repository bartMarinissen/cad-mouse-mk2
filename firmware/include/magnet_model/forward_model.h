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
     *   R, the rotation matrix that carries knob-frame vectors into the world frame
     */
    void evaluate(const Vec3& t_world,
                  const Mat3& R,
                  Vector9f &B_field,
                  Matrix9x6f &J) const;

private:
    const VirtualSensor sensors_[3];
    const MagnetModel magnets_[3];
};
