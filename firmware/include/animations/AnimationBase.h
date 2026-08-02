#pragma once

#include <Adafruit_NeoPixel.h>

class AnimationBase {
 public:
  explicit AnimationBase(Adafruit_NeoPixel& ring) : ring_(ring) {}
  virtual ~AnimationBase() {}
  virtual void update() = 0;
  virtual bool wantsPower() const { return true; }

 protected:
  Adafruit_NeoPixel& ring_;
};
