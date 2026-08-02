#include "controllers/LEDController.h"

#include <new>
#include <utility>

#include "Config.h"

LEDController::LEDController()
    : ring_(Config::LED_COUNT, Config::PIN_LED_DATA,
            NEO_GRB + NEO_KHZ800) {}

void LEDController::begin() {
  pinMode(Config::PIN_LED_LS, OUTPUT);
  digitalWrite(Config::PIN_LED_LS, LOW);

  ring_.begin();
  ring_.setBrightness(Config::LED_BRIGHTNESS);
  ring_.show();
}

Adafruit_NeoPixel& LEDController::ring() { return ring_; }

void LEDController::setPower(bool enabled) {
  if (enabled == isPowered_) {
    return;
  }

  isPowered_ = enabled;
  digitalWrite(Config::PIN_LED_LS, enabled ? HIGH : LOW);
  delay(10);
}

void LEDController::activate() {
  AnimationBase* anim = active_.pointer();
  if (anim->wantsPower()) {
    setPower(true);
    anim->update();
  } else {
    anim->update();
    setPower(false);
  }
}

void LEDController::set(SolidAnimation animation) {
  active_.destroy();
  new (&active_.storage.solid) SolidAnimation(std::move(animation));
  active_.kind = AnimationKind::Solid;
  activate();
}

void LEDController::set(SpinnerAnimation animation) {
  active_.destroy();
  new (&active_.storage.spinner) SpinnerAnimation(std::move(animation));
  active_.kind = AnimationKind::Spinner;
  activate();
}

void LEDController::set(OffAnimation animation) {
  active_.destroy();
  new (&active_.storage.off) OffAnimation(std::move(animation));
  active_.kind = AnimationKind::Off;
  activate();
}

void LEDController::update() {
  if (AnimationBase* anim = active_.pointer()) {
    anim->update();
  }
}
