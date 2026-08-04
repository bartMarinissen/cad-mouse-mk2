#include "Config.h"

#include "magnet_model/positions.h"

namespace Config {

CalibrationParams defaultCalibration() {
  CalibrationParams cal{};

  const Vec3 nominal_magnet_pos[3] = {
      Positions::Magnet_1_knob,
      Positions::Magnet_2_knob,
      Positions::Magnet_3_knob,
  };

  for (int i = 0; i < 3; i++) {
    // Today's behaviour: a scalar gain per sensor, i.e. no cross-axis skew.
    // The full 3x3 only ever comes from a real bundle calibration.
    cal.sensor_gain[i] = magnet_gains[i] * Mat3::Identity();
    cal.sensor_offset_mT[i] =
        Vec3(sensor_offset_mT[i][0], sensor_offset_mT[i][1], sensor_offset_mT[i][2]);

    cal.magnet_pos_knob[i] = nominal_magnet_pos[i];
    cal.magnet_rotation[i] = Mat3::Identity();
    // 1.0 is the correct default, not a placeholder: magnet_gains above still
    // carries the entire field scale, so the strengths are already normalized
    // out. See the note in CalibrationParams.h.
    cal.magnet_strength[i] = 1.0f;
  }

  return cal;
}

}  // namespace Config
