#include "states/ErrorState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"
#include "animations/Animations.h"

void ErrorState::enter() {
  ledController().set(SpinnerAnimation(ledController().ring(), Config::LED_ERROR_COLOR));
}

void ErrorState::update() {
  ledController().update();
}

void ErrorState::exit() {}
