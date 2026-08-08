#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include <utility>
#include <variant>

#include "animations/Animations.h"

using Animation = std::variant<SolidAnimation, SpinnerAnimation, OffAnimation>;

class LEDController {
 public:
  LEDController();
  void begin();
  Adafruit_NeoPixel& ring();

  // A template rather than an Animation-typed parameter: AnimationBase holds
  // a reference member, which makes SolidAnimation/SpinnerAnimation/OffAnimation
  // move-constructible but not move-assignable, so std::variant's operator=
  // is implicitly deleted for Animation. emplace<T>() only needs
  // constructibility (destroy the old alternative, construct the new one in
  // place), which sidesteps that.
  template <typename T>
  void set(T animation) {
    active_.emplace<T>(std::move(animation));
    activate();
  }

  void update();

 private:
  void activate();
  void setPower(bool enabled);

  bool isPowered_ = false;
  Adafruit_NeoPixel ring_;
  Animation active_;
};
