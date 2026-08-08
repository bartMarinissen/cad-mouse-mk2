#include "controllers/LEDController.h"

#include <utility>

#include "Config.h"

LEDController::LEDController()
    : ring_(Config::LED_COUNT, Config::PIN_LED_DATA, NEO_GRB + NEO_KHZ800),
      active_(OffAnimation(ring_)) {}

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
  const bool wantsPower = std::visit([](auto& anim) { return anim.wantsPower(); }, active_);
  if (wantsPower) {
    setPower(true);
    std::visit([](auto& anim) { anim.update(); }, active_);
  } else {
    std::visit([](auto& anim) { anim.update(); }, active_);
    setPower(false);
  }
}

void LEDController::update() {
  std::visit([](auto& anim) { anim.update(); }, active_);
}
