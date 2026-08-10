#include "states/SleepState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"
#include "animations/Animations.h"

void SleepState::enter() {
  ledController().set(OffAnimation(ledController().ring()));
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
