#include "states/CalibratingState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void CalibratingState::enter() {
  calibrated().sensor.beginCalibration();
  calibrated().motion.reset();
  ledController.startSpinner(Config::LED_CALIBRATING_COLOR);
}

void CalibratingState::update() {
  inputController.update();
  ledController.updateSpinner();
  calibrated().sensor.updateCalibration();

  // TODO check serial
  if (inputController.takeActivity()) {
    stateMachine.changeState(&StateMachine::bundleState);
  }
  

  if (calibrated().sensor.calibrationDone()) {
    stateMachine.changeState(&StateMachine::idleState);
  }
}

void CalibratingState::exit() {}
