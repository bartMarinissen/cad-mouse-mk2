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

// Shows the knob's live 6DOF pose as colour on the ring.
//
// A colour solid is rigidly attached to the knob, defined in the knob's own
// frame in millimetres. Eight virtual samplers sit fixed in world space, in
// a flat ring around the knob's expected neutral position. As the knob moves
// the solid slides and turns past those stationary samplers, and each one
// reads out whatever colour sits at its location inside the solid.
//
// There are no gain factors anywhere -- every quantity is real millimetres.
// The only thing to tune is the shape of the solid itself (see solidColor()
// in the .cpp), i.e. how fast colour varies per millimetre of travel.
class PoseColorAnimation : public AnimationBase {
 public:
  explicit PoseColorAnimation(Adafruit_NeoPixel& ring);
  void update() override;

 private:
  // Sampler positions in the world frame, fixed. Built once in the ctor.
  Vec3 samplerWorld_[Config::LED_COUNT];
};
