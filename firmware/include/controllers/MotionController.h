#pragma once
#include "math3D.h"
#include "magnet_model/positions.h"

using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;

struct Statistics {
  static constexpr float smoothing = 0.99f;

  // profiling
  int time_tot = {};
  int n_time = {};
  
  // Mean tracking
  Vector9f avg_residual = {};
  Matrix9x6f avg_jacobian = {};
  
  // Variance tracking (second moment for EMA)
  Vector9f avg_residual_sq = {};
  
  // Latest values
  Vector9f last_residual = {};
  Matrix9x6f last_jacobian = {};
  
  void update(int time = 0);
  void reset();
  Vector9f get_residual_stddev() const;
};

class MotionController {
 public:
  void reset();
  // Returns the residual in percents
  float compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;
  void set_base_pose(const Vec3 pos, const Vec3 rot);
  // Returns the residual
  float read_pose(const float raw[9], Vec3 &position, Vec3 &rot);
  Statistics statistics{};
  Vec3 last_pos = Positions::approx_rest_pos;
  Vec3 last_rot {};

 private:
  static float clampf(float v, float lo, float hi);
  static float hardZero(float v, float thr);
  static float lowpass(float prev, float x, float dt, float tau);
  static float axisBaseDead(int i);
  float filt_[6] = {};
  bool motionActive_ = false;
  // Baseline measurements, for computing offset against
  Vec3 base_pos = Positions::approx_rest_pos;
  Vec3 base_rot {};
  // Last measurements, for hot-starting the newton-gauss
};

