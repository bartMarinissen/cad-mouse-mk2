#include "controllers/MotionController.h"
#include "magnet_model/forward_model.h"
#include "math3D.h"
#include "magnet_model/positions.h"

#include <Arduino.h>
#include <math.h>

// #include <sstream>
// #include <string>

#include "Config.h"
#include <magnet_model/solve_pose.h>



MotionController::MotionController(const CalibrationParams& cal)
    : forward_model_(cal) {}

namespace {
enum RawIndex {
  RAW_MAG1_X = 0,
  RAW_MAG1_Y,
  RAW_MAG1_Z,
  RAW_MAG2_X,
  RAW_MAG2_Y,
  RAW_MAG2_Z,
  RAW_MAG3_X,
  RAW_MAG3_Y,
  RAW_MAG3_Z
};

enum AxisIndex {
  AXIS_TX = 0,
  AXIS_TY,
  AXIS_TZ,
  AXIS_RX,
  AXIS_RY,
  AXIS_RZ
};
}  // namespace

void MotionController::reset() {
  for (int i = 0; i < 6; i++) {
    filt_[i] = 0.0;
  }
  motionActive_ = false;
  last_pos = base_pos = Positions::approx_rest_pos;
  last_rot = base_rot = BLA::Zeros<3, 1, float>();
  last_R = identity3();
  statistics.reset();
}

void MotionController::set_base_pose(const Vec3 pos, const Vec3 rot){
  last_pos = base_pos = pos;
  last_rot = base_rot = rot;
  // The hot start is re-seeded from identity rather than rebuilt from `rot`:
  // this runs once after calibration, with the knob at rest and therefore
  // within a couple of degrees of identity anyway, so the round trip back
  // through Euler angles would buy nothing.
  last_R = identity3();
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::hardZero(float v, float thr) {
  return (fabs(v) < thr) ? 0.0 : v;
}

float MotionController::lowpass(float prev, float x, float dt, float tau) {
  if (tau <= 0.0) return x;
  const float a = dt / (tau + dt);
  return prev + a * (x - prev);
}

float MotionController::axisBaseDead(int i) {
  return (i < 3) ? Config::DEAD_T : Config::DEAD_R;
}

// Extracts Z-Y-X Euler angles (Yaw, Pitch, Roll) from a rotation matrix.
// Forces Pitch into the human-intuitive [-90, +90] degree range to prevent 180-deg flips.
// Declared in MotionController.h; see that declaration for why this is a free
// function rather than a method.
Vec3 extract_angles_robust(const Mat3& R) {
    float pitch, roll, yaw;

    // R(row, col)
    // Check for Gimbal Lock (when Pitch approaches exactly +/- 90 degrees)
    if (R(2, 0) < -0.999f) { 
        pitch = M_PI / 2.0f; // +90 degrees
        roll = 0.0f;
        yaw = std::atan2(R(0, 1), R(0, 2));
    } 
    else if (R(2, 0) > 0.999f) {
        pitch = -M_PI / 2.0f; // -90 degrees
        roll = 0.0f;
        yaw = std::atan2(-R(0, 1), -R(0, 2));
    } 
    else {
        // Standard extraction
        // asin() strictly bounds the pitch between -pi/2 and +pi/2 (-90 to +90 deg)
        pitch = std::asin(-R(2, 0));
        
        // atan2 determines the correct quadrant for roll and yaw based on the bounded pitch
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw  = std::atan2(R(1, 0), R(0, 0));
    }

    // Return in radians [Yaw, Pitch, Roll] or [Pitch, Roll, Yaw] depending on your preference
    // Here returning [Pitch, Roll, Yaw]
    // M_PI is a double; BLA's scalar operators deduce DType from both
    // operands (see TODO/eigen-to-bla-migration.md), so this has to reduce
    // to a float before multiplying a Vec3 by it.
    return Vec3(pitch, roll, yaw) * (180.0f / float(M_PI));
}

// Return residual and other quality reports
float MotionController::read_pose(const float raw[9], Vec3 &position, Mat3 &R){
  const Vector9f measured = Vector9f(
    raw[RAW_MAG1_X], raw[RAW_MAG1_Y], raw[RAW_MAG1_Z], 
    raw[RAW_MAG2_X], raw[RAW_MAG2_Y], raw[RAW_MAG2_Z],
    raw[RAW_MAG3_X], raw[RAW_MAG3_Y], raw[RAW_MAG3_Z]
  );

  Vector9f *residual_vec_ptr = Config::statistics ? &statistics.last_residual : nullptr;

  // Both `position` and `R` arrive carrying the caller's starting guess -- for
  // the live path that is the previous frame's pose -- and the solver refines
  // them in place. R used to be reset to identity here, which threw away half
  // the hot start and made the solver re-converge the rotation every frame.
  // solve_knob_pose also re-orthonormalizes R before returning, since it's now
  // long-lived state rather than rebuilt from scratch each call.
  const uint32_t before = micros();
  float residual_magnitude = solve_knob_pose(position, R, forward_model_, measured, residual_vec_ptr);
  const uint32_t after = micros();
  if (Config::statistics)
    statistics.update(after - before);
  return residual_magnitude;
}


float MotionController::compute(const float raw[9], const float baseline[9], float dt,
                               float out[6]) {
  // hot start from previous value

  // TODO: as a backup if this is a bad result, try the base position from calibration
  float residual = read_pose(raw, last_pos, last_R);
  last_rot = extract_angles_robust(last_R);

  // BLA has no constructor taking a raw pointer/array (see
  // TODO/eigen-to-bla-migration.md), and this is only ever used to compute
  // a norm, so compute the sum-of-squares directly rather than
  // materializing a Vector9f just to read it back out.
  float raw_sum_sq = 0.0f;
  for (int i = 0; i < 9; ++i) raw_sum_sq += raw[i] * raw[i];
  float residual_percent = 100 * residual / sqrtf(raw_sum_sq);

  Vec3 t   = last_pos - base_pos;
  Vec3 rot = last_rot - base_rot;

  // Apply sign fixes and gains
  float y[6];
  y[AXIS_TX] = Config::SIGN_AXIS[AXIS_TX] * t(0) * Config::GAIN_T[AXIS_TX];
  y[AXIS_TY] = Config::SIGN_AXIS[AXIS_TY] * t(1) * Config::GAIN_T[AXIS_TY];
  y[AXIS_TZ] = Config::SIGN_AXIS[AXIS_TZ] * t(2) * Config::GAIN_T[AXIS_TZ];
  // TODO: figure out why X and Y rotation here are flipped
  y[AXIS_RX] = Config::SIGN_AXIS[AXIS_RX] * rot(1) * Config::GAIN_R[AXIS_RX - 3];
  y[AXIS_RY] = Config::SIGN_AXIS[AXIS_RY] * rot(0) * Config::GAIN_R[AXIS_RY - 3];
  y[AXIS_RZ] = Config::SIGN_AXIS[AXIS_RZ] * rot(2) * Config::GAIN_R[AXIS_RZ - 3];


  
 
  // Filter, clamp to range and dead zones.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = axisBaseDead(i);

    if (fabs(y[i]) < dead) {
      filt_[i] = 0.0;
    } else {
      filt_[i] = lowpass(filt_[i], y[i], dt, Config::SMOOTH_TAU_S);
    }

    const float limited =
        clampf(filt_[i], -Config::AXIS_LIMIT, Config::AXIS_LIMIT);
    out[i] = hardZero(limited, dead);
    if (out[i] != 0.0) {
      motionActive_ = true;
    }
  }
  return residual_percent;
}

bool MotionController::hasMotionActivity() const { return motionActive_; }

void Statistics::update(uint32_t time_last){
  // Update mean (first moment)
  avg_residual *= smoothing;
  avg_residual += (1 - smoothing) * last_residual;

  // NOTE: there used to be a matching EMA over the 9x6 Jacobian here. It cost
  // ~162 flops every frame and nothing ever read the result -- avg_residual and
  // avg_residual_sq feed TelemetryController, avg_jacobian fed nothing.

  // Update second moment for variance calculation
  avg_residual_sq *= smoothing;
  avg_residual_sq += (1 - smoothing) * cwise_product(last_residual, last_residual);

  time_tot += time_last;
  n_time++;
}

void Statistics::reset(){
  avg_residual = BLA::Zeros<9, 1, float>();
  avg_residual_sq = BLA::Zeros<9, 1, float>();
  last_residual = BLA::Zeros<9, 1, float>();
  time_tot = 0;
  n_time = 0;
}

Vector9f Statistics::get_residual_stddev() const {
  // Variance = E[X²] - E[X]²
  return cwise_sqrt(avg_residual_sq - cwise_product(avg_residual, avg_residual));
}
