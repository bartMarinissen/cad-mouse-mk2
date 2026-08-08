#pragma once

#include "Config.h"
#include "animations/AnimationBase.h"
#include "math3D.h"

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

// Represents the knob's live pose (rotation + translation, relative to its
// calibrated rest pose) as colors sampled from a cylindrical-HSV color solid.
// See TODO-plan discussion / commit message for the math; constants and the
// point-to-color mapping are expected to need heavy retuning.
class PoseColorAnimation : public AnimationBase {
 public:
  explicit PoseColorAnimation(Adafruit_NeoPixel& ring);
  void update() override;

 private:
  Vec3 referenceOffsets_[Config::LED_COUNT];
};
