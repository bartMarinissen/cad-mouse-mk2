#pragma once
#include "CalibrationParams.h"
#include "math3D.h"
#include "magnet_model/forward_model.h"
#include "magnet_model/positions.h"

struct Statistics {
  static constexpr float smoothing = 0.99f;

  // Profiling, in microseconds. Wraps after roughly an hour of solving, which
  // is fine for profiling.
  uint32_t time_tot = {};
  uint32_t n_time = {};

  // Mean tracking
  Vector9f avg_residual = {};

  // Variance tracking (second moment for EMA)
  Vector9f avg_residual_sq = {};

  // Latest values
  Vector9f last_residual = {};

  void update(uint32_t time = 0);
  void reset();
  Vector9f get_residual_stddev() const;
};

// Extracts Z-Y-X Euler angles (Yaw, Pitch, Roll), in degrees, from a rotation
// matrix, with a fixed convention (see the .cpp) for reporting pose over HID
// and for calibration logging. Free function rather than a MotionController
// method: it's a pure function of R, not solver state, and both
// MotionController::compute() and SensorController's calibration path need it
// on a result they got back from read_pose().
Vec3 extract_angles_robust(const Mat3& R);

class MotionController {
 public:
  explicit MotionController(const CalibrationParams& cal);

  void reset();
  // Returns the residual in percents
  float compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;
  void set_base_pose(const Vec3 pos, const Vec3 rot);
  // Returns the residual. `position` and `R` are in/out: they carry the previous
  // frame's estimate in as the solver's starting point and the new one out.
  // This is purely the solve -- it does not derive a reporting representation
  // (Euler degrees, HID axes, ...) from the result. Callers that want that call
  // extract_angles_robust(R) themselves afterward; folding it in here would
  // make every caller pay for a conversion only some of them want, and mixes
  // "solve a pose" with "format a pose for reporting."
  float read_pose(const float raw[9], Vec3 &position, Mat3 &R);
  Statistics statistics{};
  Vec3 last_pos = Positions::approx_rest_pos;
  Vec3 last_rot {};
  // The rotation half of the hot start. Kept as a matrix rather than rebuilt
  // from last_rot each frame: going back through Euler angles is both lossy and
  // expensive, and this is the solver's actual state variable.
  Mat3 last_R = identity3();

 private:
  static float clampf(float v, float lo, float hi);
  static float hardZero(float v, float thr);
  static float lowpass(float prev, float x, float dt, float tau);
  static float axisBaseDead(int i);

  // The model the pose solve runs against. Previously three translation-unit
  // globals in MotionController.cpp, which was the reason a calibration
  // write-back had no path to the running VirtualSensor/MagnetModel instances. Now
  // owned, and const because it is built from this controller's calibration
  // and never changes after that.
  const ForwardModel forward_model_;

  float filt_[6] = {};
  bool motionActive_ = false;
  // Baseline measurements, for computing offset against
  Vec3 base_pos = Positions::approx_rest_pos;
  Vec3 base_rot {};
  // Last measurements, for hot-starting the newton-gauss
};

