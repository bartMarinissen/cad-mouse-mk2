#pragma once

#include "State.h"
#include "states/CalibratingState.h"
#include "states/IdleState.h"
#include "states/SleepState.h"
#include "states/ErrorState.h"
#include "states/BundleState.h"

class StateMachine {
 public:
  StateMachine();
  void changeState(State *newState);
  void update();

  static IdleState idleState;
  static SleepState sleepState;
  static CalibratingState calibratingState;
  static ErrorState errorState;
  static BundleState bundleState;

 private:
  State *currentState = nullptr;
};

extern StateMachine stateMachine;
