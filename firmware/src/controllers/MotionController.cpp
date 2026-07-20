#include "controllers/MotionController.h"
#include "math3D.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

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


void MotionController::compute(const float raw[9], const float baseline[9], float dt,
                               float out[6]) {
  const Vec3 baseline1 = Vec3(baseline[RAW_MAG1_X], baseline[RAW_MAG1_Y], baseline[RAW_MAG1_Z]);
  const Vec3 baseline2 = Vec3(baseline[RAW_MAG2_X], baseline[RAW_MAG2_Y], baseline[RAW_MAG2_Z]);
  const Vec3 baseline3 = Vec3(baseline[RAW_MAG3_X], baseline[RAW_MAG3_Y], baseline[RAW_MAG3_Z]);
  const Vec3 raw1 = Vec3(raw[RAW_MAG1_X], raw[RAW_MAG1_Y], raw[RAW_MAG1_Z]);
  const Vec3 raw2 = Vec3(raw[RAW_MAG2_X], raw[RAW_MAG2_Y], raw[RAW_MAG2_Z]);
  const Vec3 raw3 = Vec3(raw[RAW_MAG3_X], raw[RAW_MAG3_Y], raw[RAW_MAG3_Z]);

  const Vec3 sens1 = pow_magnitude(raw1, -0.3333333333) - pow_magnitude(baseline1, -0.333333333);
  const Vec3 sens2 = pow_magnitude(raw2, -0.3333333333) - pow_magnitude(baseline2, -0.333333333);
  const Vec3 sens3 = pow_magnitude(raw3, -0.3333333333) - pow_magnitude(baseline3, -0.333333333);

  // Translation:
  const Vec3 sensAvg = (sens1 + sens2 + sens3) * (1.0f / 3.0f);
  const float tx = sensAvg[0];
  const float ty = sensAvg[1];
  const float tz = sensAvg[2];

  // Physical PCB layout:
  // MAG2 = top left, MAG3 = top right, MAG1 = bottom.
  const float mag2PosX = -0.5;
  const float mag2PosY = sqrt(3.0) / 6.0;

  const float mag3PosX = 0.5;
  const float mag3PosY = sqrt(3.0) / 6.0;

  const float mag1PosX = 0.0;
  const float mag1PosY = -sqrt(3.0) / 3.0;

  // Rotation estimates:
  //   Ry = mag3z - mag2z
  //     right sensor minus left sensor
  //     -> side to side tilt across the top edge
  //
  //   Rx = sqrt(3) * (mag2z + mag3z - 2 * mag1z) / 3
  //     top pair minus bottom sensor
  //     -> front/back tilt of the triangle
  const float rx = (sqrt(3.0) * (sens2[2] + sens3[2] - 2.0 * sens1[2])) / 3.0;
  const float ry = (sens3[2] - sens2[2]);

  //   Rz = sum_i (posXi * magYi - posYi * magXi)
  // Each sensor contributes according to its x/y position in the triangle.
  const float swirlNum =
      (mag2PosX * sens2[1] - mag2PosY * sens2[0]) +
      (mag3PosX * sens3[1] - mag3PosY * sens3[0]) +
      (mag1PosX * sens1[1] - mag1PosY * sens1[0]);
  const float rz = swirlNum;

  // Apply sign fixes and gains
  float y[6];
  y[AXIS_TX] = Config::SIGN_AXIS[AXIS_TX] * tx * Config::GAIN_T[AXIS_TX];
  y[AXIS_TY] = Config::SIGN_AXIS[AXIS_TY] * ty * Config::GAIN_T[AXIS_TY];
  y[AXIS_TZ] = Config::SIGN_AXIS[AXIS_TZ] * tz * Config::GAIN_T[AXIS_TZ];
  y[AXIS_RX] = Config::SIGN_AXIS[AXIS_RX] * rx * Config::GAIN_R[AXIS_RX - 3];
  y[AXIS_RY] = Config::SIGN_AXIS[AXIS_RY] * ry * Config::GAIN_R[AXIS_RY - 3];
  y[AXIS_RZ] = Config::SIGN_AXIS[AXIS_RZ] * rz * Config::GAIN_R[AXIS_RZ - 3];
  
 

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
}

bool MotionController::hasMotionActivity() const { return motionActive_; }
