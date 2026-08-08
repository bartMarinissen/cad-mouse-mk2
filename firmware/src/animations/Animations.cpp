#include "animations/Animations.h"

#include <algorithm>
#include <cmath>

#include "Controllers.h"

namespace {

void fillAll(Adafruit_NeoPixel& ring, unsigned long color) {
  for (int i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, color);
  }
}

unsigned long toNeoColor(Adafruit_NeoPixel& ring, unsigned long color) {
  int r = (color >> 16) & 0xFF;
  int g = (color >> 8) & 0xFF;
  int b = color & 0xFF;
  return ring.Color(r, g, b);
}

}  // namespace

SolidAnimation::SolidAnimation(Adafruit_NeoPixel& ring, unsigned long color)
    : AnimationBase(ring), color_(toNeoColor(ring, color)) {}

void SolidAnimation::update() {
  fillAll(ring_, color_);
  ring_.show();
}

SpinnerAnimation::SpinnerAnimation(Adafruit_NeoPixel& ring, unsigned long color)
    : AnimationBase(ring), color_(toNeoColor(ring, color)) {}

void SpinnerAnimation::update() {
  const unsigned long now = millis();
  if (lastStepMs_ != 0 && (now - lastStepMs_) < 60) {
    return;
  }
  lastStepMs_ = now;

  fillAll(ring_, 0);
  ring_.setPixelColor(index_, color_);
  ring_.show();

  index_++;
  if (index_ >= ring_.numPixels()) {
    index_ = 0;
  }
}

OffAnimation::OffAnimation(Adafruit_NeoPixel& ring) : AnimationBase(ring) {}

void OffAnimation::update() {
  fillAll(ring_, 0);
  ring_.show();
}

namespace {

// Pose-color mapping constants. Calibrated to ~5mm translation, ~20 degrees
// of tilt total from horizontal, ~10 degrees of twist -- expect heavy
// retuning once this is running on hardware, possibly a different
// parameterization entirely (that's why pointToColor() below is isolated
// from the rotate/translate geometry in PoseColorAnimation::update()).
constexpr float kR0 = 90.0f;
constexpr float kZ0 = 128.0f;
constexpr float kSatFloor = 70.0f;
constexpr float kRMaxExpected = 190.0f;
constexpr float kTransGainXY = 20.0f;
constexpr float kTransGainZ = 20.0f;

uint32_t pointToColor(const Vec3& transformed) {
  const float radiusXY = std::sqrt(transformed.x() * transformed.x() +
                                    transformed.y() * transformed.y());
  const float height = kZ0 + transformed.z();
  const float hueAngle = std::atan2(transformed.y(), transformed.x());

  const float satFrac = std::clamp(radiusXY / kRMaxExpected, 0.0f, 1.0f);
  const uint8_t sat = static_cast<uint8_t>(kSatFloor + (255.0f - kSatFloor) * satFrac);
  const uint8_t val = static_cast<uint8_t>(std::clamp(height, 0.0f, 255.0f));
  const uint16_t hue = static_cast<uint16_t>(
      (hueAngle + float(M_PI)) / (2.0f * float(M_PI)) * 65535.0f);

  return Adafruit_NeoPixel::ColorHSV(hue, sat, val);
}

}  // namespace

PoseColorAnimation::PoseColorAnimation(Adafruit_NeoPixel& ring) : AnimationBase(ring) {
  for (int i = 0; i < Config::LED_COUNT; i++) {
    const float angle = i * (2.0f * float(M_PI) / Config::LED_COUNT);
    referenceOffsets_[i] = Vec3(kR0 * std::cos(angle), kR0 * std::sin(angle), 0.0f);
  }
}

void PoseColorAnimation::update() {
  MotionController& motion = motionController();

  // [pitch, roll, yaw], degrees, relative to the calibrated rest pose.
  const Vec3 rotDeltaDeg = motion.last_rot - motion.base_rot;
  const float toRad = float(M_PI) / 180.0f;
  const float pitch = rotDeltaDeg[0] * toRad;
  const float roll = rotDeltaDeg[1] * toRad;
  const float yaw = rotDeltaDeg[2] * toRad;
  const float cp = std::cos(pitch), sp = std::sin(pitch);
  const float cr = std::cos(roll), sr = std::sin(roll);
  const float cy = std::cos(yaw), sy = std::sin(yaw);

  // R = Rz(yaw) * Ry(pitch) * Rx(roll), matching extract_angles_robust()'s
  // convention in MotionController.cpp exactly (verified by hand).
  Mat3 R;
  R << cy * cp,             cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
       sy * cp,             sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
       -sp,                 cp * sr,                 cp * cr;

  const Vec3 t = motion.last_pos - motion.base_pos;  // mm, relative to rest
  const Vec3 T(t.x() * kTransGainXY, t.y() * kTransGainXY, t.z() * kTransGainZ);

  const int n = ring_.numPixels();
  for (int i = 0; i < n; i++) {
    const Vec3 transformed = R * referenceOffsets_[i] + T;
    ring_.setPixelColor(i, pointToColor(transformed));
  }
  ring_.show();
}
