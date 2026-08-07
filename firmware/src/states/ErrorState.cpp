#include "states/ErrorState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void ErrorState::enter() {
  ledController().startSpinner(Config::LED_ERROR_COLOR);
}

void ErrorState::update() {
  ledController().updateSpinner();
}

void ErrorState::exit() {}
