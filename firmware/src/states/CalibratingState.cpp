#include "states/CalibratingState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void CalibratingState::enter() {
  sensorController().beginCalibration();
  motionController().reset();
  ledController().startSpinner(Config::LED_CALIBRATING_COLOR);
}

void CalibratingState::update() {
  SensorController& sensor = sensorController();
  InputController& input = inputController();

  input.update();
  ledController().updateSpinner();
  sensor.updateCalibration();

  // TODO check serial
  if (input.takeActivity()) {
    stateMachine.changeState(&StateMachine::bundleState);
  }


  if (sensor.calibrationDone()) {
    stateMachine.changeState(&StateMachine::idleState);
  }
}

void CalibratingState::exit() {}
