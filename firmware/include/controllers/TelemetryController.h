#pragma once
#include "controllers/MotionController.h"

class TelemetryController {
 public:
  void begin();
  void publish(const float motion[6], float residual_percent, int buttonBits,
                                  bool hidReportSent, Statistics const &stats,
                                  const float raw_field[9], const Vec3 &last_pos, const Vec3 &last_rot);
  bool enabled() const;

 private:
  int tick_ = 0;
  int last_update_ms {};
};
