#include "animations/Animations.h"

#include <algorithm>
#include <cmath>

#include "Controllers.h"
#include "magnet_model/positions.h"

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

// Radius of the ring of virtual samplers, fixed in world space around the
// knob's expected neutral position. A real dimension: 40mm diameter.
constexpr float kSamplerRingRadiusMm = 20.0f;

// --- Shape of the colour solid -------------------------------------------
// The knob only travels a few millimetres, so the solid has to vary fast to
// make that visible. Both channels below map a 10mm window (+/-5mm about
// where the samplers sit at rest) onto the full byte range; motion past the
// window clips, which is intended.
constexpr float kZWindowMm = 5.0f;
constexpr float kRadiusWindowMm = 5.0f;
// How many times hue wraps around the solid's axis. Raise to make twist more
// visible (twist only moves azimuth), at the cost of the ring reading as a
// repeating pattern instead of one clean hue wheel.
constexpr float kHueWindings = 1.0f;

// Maps value linearly from [lo, hi] onto 0..255, clamping outside the range.
uint8_t mapToByte(float value, float lo, float hi) {
  const float frac = (value - lo) / (hi - lo);
  return static_cast<uint8_t>(std::lround(std::clamp(frac, 0.0f, 1.0f) * 255.0f));
}

// Colour of the solid at a point in the knob's own frame, in millimetres.
// The solid is rigidly attached to the knob, so this is the only place that
// decides what a position looks like. It need not be HSV, cylindrical, or
// even continuous -- nothing outside this function may assume it is.
uint32_t solidColor(const Vec3& pKnob) {
  const float radius = std::sqrt(pKnob.x() * pKnob.x() + pKnob.y() * pKnob.y());
  const float azimuth = std::atan2(pKnob.y(), pKnob.x());

  // Hue wraps rather than clamps, so it does not go through mapToByte().
  const float turns = (azimuth / (2.0f * float(M_PI))) * kHueWindings;
  const uint16_t hue = static_cast<uint16_t>(
      std::lround((turns - std::floor(turns)) * 65535.0f));
  const uint8_t sat = mapToByte(radius, kSamplerRingRadiusMm - kRadiusWindowMm,
                                        kSamplerRingRadiusMm + kRadiusWindowMm);
  const uint8_t val = mapToByte(pKnob.z(), -kZWindowMm, kZWindowMm);

  return Adafruit_NeoPixel::ColorHSV(hue, sat, val);
}

}  // namespace

PoseColorAnimation::PoseColorAnimation(Adafruit_NeoPixel& ring) : AnimationBase(ring) {
  for (int i = 0; i < Config::LED_COUNT; i++) {
    const float angle = i * (2.0f * float(M_PI) / Config::LED_COUNT);
    samplerWorld_[i] = Positions::approx_rest_pos +
                       Vec3(kSamplerRingRadiusMm * std::cos(angle),
                            kSamplerRingRadiusMm * std::sin(angle), 0.0f);
  }
}

void PoseColorAnimation::update() {
  MotionController& motion = motionController();

  // last_rot is [pitch, roll, yaw] in degrees, absolute in the world frame.
  const float toRad = float(M_PI) / 180.0f;
  const float pitch = motion.last_rot[0] * toRad;
  const float roll = motion.last_rot[1] * toRad;
  const float yaw = motion.last_rot[2] * toRad;
  const float cp = std::cos(pitch), sp = std::sin(pitch);
  const float cr = std::cos(roll), sr = std::sin(roll);
  const float cy = std::cos(yaw), sy = std::sin(yaw);

  // R = Rz(yaw) * Ry(pitch) * Rx(roll), the inverse of the extraction that
  // produced last_rot (extract_angles_robust(), MotionController.cpp).
  // Maps knob frame -> world frame.
  Mat3 R;
  R << cy * cp,             cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
       sy * cp,             sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
       -sp,                 cp * sr,                 cp * cr;

  // World point -> knob frame, the same transform Sensor::evaluate() applies
  // to the (likewise world-fixed) physical sensors.
  const Mat3 worldToKnob = R.transpose();

  const int n = ring_.numPixels();
  for (int i = 0; i < n; i++) {
    const Vec3 pKnob = worldToKnob * (samplerWorld_[i] - motion.last_pos);
    ring_.setPixelColor(i, solidColor(pKnob));
  }
  ring_.show();
}
