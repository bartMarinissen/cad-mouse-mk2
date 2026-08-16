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
// make that visible. The windows below are sized so that travel spans the
// full range; motion past a window clips, which is intended.
constexpr float kZWindowMm = 4.0f;
constexpr float kRadiusWindowMm = 2.0f;
// Times the red/blue wave winds around the solid's axis. Twist only moves
// azimuth, so this is the knob for twist sensitivity. At 2 the ring's 8 LEDs
// sample 4 per period -- comfortably clear of aliasing, and the pattern still
// reads as a shape rather than a repeat.
constexpr float kTriangleWindings = 2.0f;

// Maps value linearly from [lo, hi] onto 0..1, clamping outside the range.
float mapToUnit(float value, float lo, float hi) {
  return std::clamp((value - lo) / (hi - lo), 0.0f, 1.0f);
}

// Signed triangle wave over turns: +1 at whole turns, -1 at half turns.
float triangleWave(float turns) {
  const float phase = turns - std::floor(turns);
  return 4.0f * std::fabs(phase - 0.5f) - 1.0f;
}

uint8_t toByte(float unit) {
  return static_cast<uint8_t>(std::lround(std::clamp(unit, 0.0f, 1.0f) * 255.0f));
}

// Colour of the solid at a point in the knob's own frame, in millimetres.
// The solid is rigidly attached to the knob, so this is the only place that
// decides what a position looks like. It need not be HSV, cylindrical, or
// even continuous -- nothing outside this function may assume it is.
//
// Built straight in RGB. Saturation is maximal everywhere by construction:
// the tangential triangle wave puts red and blue on opposite signs, so one of
// the two is always exactly zero and no point in the solid washes out.
//
//   z       -> brightness, scaling all three channels together
//   radius  -> green
//   azimuth -> triangle wave, red on its positive half, blue on its negative
uint32_t solidColor(const Vec3& pKnob) {
  const float radius = std::sqrt(pKnob(0) * pKnob(0) + pKnob(1) * pKnob(1));
  const float azimuth = std::atan2(pKnob(1), pKnob(0));

  const float wave =
      triangleWave(azimuth / (2.0f * float(M_PI)) * kTriangleWindings);
  const float green = mapToUnit(radius, kSamplerRingRadiusMm - kRadiusWindowMm,
                                        kSamplerRingRadiusMm + kRadiusWindowMm);
  const float bright = mapToUnit(pKnob(2), kZWindowMm, -kZWindowMm);

  return Adafruit_NeoPixel::Color(toByte(std::max(wave, 0.0f) * bright),
                                  toByte(green * bright),
                                  toByte(std::max(-wave, 0.0f) * bright));
}

}  // namespace

PoseColorAnimation::PoseColorAnimation(Adafruit_NeoPixel& ring) : AnimationBase(ring) {
  for (int i = 0; i < Config::LED_COUNT; i++) {
    const float angle = (i + 1/16) * (-2.0f * float(M_PI) / Config::LED_COUNT);
    samplerWorld_[i] = Positions::approx_rest_pos +
                       Vec3(kSamplerRingRadiusMm * std::sin(angle),
                            kSamplerRingRadiusMm * std::cos(angle), 0.0f);
  }
}

void PoseColorAnimation::update() {
  MotionController& motion = motionController();

  // last_rot is [pitch, roll, yaw] in degrees, absolute in the world frame.
  const float toRad = float(M_PI) / 180.0f;
  const float pitch = motion.last_rot(0) * toRad;
  const float roll = motion.last_rot(1) * toRad;
  const float yaw = motion.last_rot(2) * toRad;
  const float cp = std::cos(pitch), sp = std::sin(pitch);
  const float cr = std::cos(roll), sr = std::sin(roll);
  const float cy = std::cos(yaw), sy = std::sin(yaw);

  // R = Rz(yaw) * Ry(pitch) * Rx(roll), the inverse of the extraction that
  // produced last_rot (extract_angles_robust(), MotionController.cpp).
  // Maps knob frame -> world frame. BLA's variadic constructor fills
  // row-major, same order the comma operator used to.
  Mat3 R(cy * cp,     cy * sp * sr - sy * cr,    cy * sp * cr + sy * sr,
         sy * cp,     sy * sp * sr + cy * cr,    sy * sp * cr - cy * sr,
         -sp,         cp * sr,                   cp * cr);

  // World point -> knob frame, the same transform VirtualSensor::evaluate() applies
  // to the (likewise world-fixed) physical sensors.
  const Mat3 worldToKnob = ~R;

  const int n = ring_.numPixels();
  for (int i = 0; i < n; i++) {
    const Vec3 pKnob = worldToKnob * (samplerWorld_[i] - motion.last_pos);
    ring_.setPixelColor(i, solidColor(pKnob));
  }
  ring_.show();
}
