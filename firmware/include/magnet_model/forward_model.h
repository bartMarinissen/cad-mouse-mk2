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

    // Computes the 9x1 residual and 9x6 Jacobian for a given pose
    // Note the pose for this jacobian is in terms of a translation vector t 
    //  and a rotation vector omega. But we caluclate it in terms of a rotation MATRIX R.
    void evaluate(const Vec3& t,
                  const Mat3& R,
                  Vector9f &B_field,
                  Matrix9x6f &J) const;

private:
    const VirtualSensor sensors_[3];
    const MagnetModel magnets_[3];
};
