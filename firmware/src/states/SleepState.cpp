#include "states/SleepState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void SleepState::enter() {
  ledController().off();
}

void SleepState::update() {
  InputController& input = inputController();
  input.update();

  if (input.takeActivity()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }
}

void SleepState::exit() {}
