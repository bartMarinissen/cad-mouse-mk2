#pragma once

#include "animations/AnimationBase.h"

class SolidAnimation : public AnimationBase {
 public:
  SolidAnimation(Adafruit_NeoPixel& ring, unsigned long color);
  void update() override;

 private:
  unsigned long color_;
};

class SpinnerAnimation : public AnimationBase {
 public:
  SpinnerAnimation(Adafruit_NeoPixel& ring, unsigned long color);
  void update() override;

 private:
  unsigned long color_;
  int index_ = 0;
  unsigned long lastStepMs_ = 0;
};

class OffAnimation : public AnimationBase {
 public:
  explicit OffAnimation(Adafruit_NeoPixel& ring);
  void update() override;
  bool wantsPower() const override { return false; }
};
