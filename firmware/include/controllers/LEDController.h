#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "animations/Animations.h"

enum class AnimationKind { None, Solid, Spinner, Off };

union AnimationUnion {
  AnimationUnion() {}
  ~AnimationUnion() {}
  SolidAnimation solid;
  SpinnerAnimation spinner;
  OffAnimation off;
};

struct AnimationSlot {
  AnimationKind kind = AnimationKind::None;
  AnimationUnion storage;

  AnimationBase* pointer() {
    switch (kind) {
      case AnimationKind::Solid:
        return &storage.solid;
      case AnimationKind::Spinner:
        return &storage.spinner;
      case AnimationKind::Off:
        return &storage.off;
      case AnimationKind::None:
        return nullptr;
    }
    return nullptr;
  }

  void destroy() {
    switch (kind) {
      case AnimationKind::Solid:
        storage.solid.~SolidAnimation();
        break;
      case AnimationKind::Spinner:
        storage.spinner.~SpinnerAnimation();
        break;
      case AnimationKind::Off:
        storage.off.~OffAnimation();
        break;
      case AnimationKind::None:
        break;
    }
    kind = AnimationKind::None;
  }
};

class LEDController {
 public:
  LEDController();
  void begin();
  Adafruit_NeoPixel& ring();
  void set(SolidAnimation animation);
  void set(SpinnerAnimation animation);
  void set(OffAnimation animation);
  void update();

 private:
  void activate();
  void setPower(bool enabled);

  bool isPowered_ = false;
  AnimationSlot active_;
  Adafruit_NeoPixel ring_;
};
