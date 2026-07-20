#include "StateMachine.h"

StateMachine stateMachine;

IdleState StateMachine::idleState;
SleepState StateMachine::sleepState;
CalibratingState StateMachine::calibratingState;
ErrorState StateMachine::errorState;

StateMachine::StateMachine() : currentState(nullptr) {}

void StateMachine::changeState(State *newState) {
  if (newState == nullptr || newState == currentState) {
    return;
  }

  if (currentState != nullptr) {
    currentState->exit();
  }

  currentState = newState;
  currentState->enter();
}

void StateMachine::update() {
  if (currentState == nullptr) {
    return;
  }
  currentState->update();
}
