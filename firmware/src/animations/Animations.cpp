#include "animations/Animations.h"

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
