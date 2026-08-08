#include "Config.h"

#include "magnet_model/magnet_model_table.h"
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
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        // Today's behaviour: a scalar gain per sensor, i.e. no cross-axis
        // skew. The full 3x3 only ever comes from a real bundle calibration.
        cal.sensor_gain[i][r][c] = (r == c) ? magnet_gains[i] : 0.0f;
        cal.magnet_rotation[i][r][c] = (r == c) ? 1.0f : 0.0f;
      }
      cal.sensor_offset_mT[i][r] = sensor_offset_mT[i][r];
      cal.magnet_pos_knob[i][r] = nominal_magnet_pos[i](r);
    }

    // BICUBIC_FIELD_REFERENCE_MT, not a placeholder: it makes
    // magnet_strength_mT[i] / BICUBIC_FIELD_REFERENCE_MT exactly 1.0, i.e.
    // "no separate strength correction", because magnet_gains above is a
    // hand-tuned scalar that already carries the entire field scale -- these
    // defaults are in the gain-carries-scale gauge, not the fit's det(G)=1
    // one. See CalibrationParams.h.
    cal.magnet_strength_mT[i] = BICUBIC_FIELD_REFERENCE_MT;
  }

  return cal;
}

}  // namespace Config
