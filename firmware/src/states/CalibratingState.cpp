#include "states/CalibratingState.h"

#include <Arduino.h>
#include <string.h>

#include "CalibrationStorage.h"
#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void CalibratingState::enter() {
  sensorController().beginCalibration();
  motionController().reset();
  ledController().startSpinner(Config::LED_CALIBRATING_COLOR);

  // Tells the host that its command window is open. This state is the only
  // place CAL_START and CAL_UPLOAD are accepted and it lasts just
  // Config::ZERO_SAMPLES ticks (~1.7s), so the PC waits for this line and
  // replies immediately rather than trying to guess the timing. The expected
  // route in is the deliberate both-buttons-3s tare gesture from IdleState,
  // where the port is already open and there is no enumeration to race.
  Serial.println("STATUS TARE_BEGIN");
}

void CalibratingState::update() {
  SensorController& sensor = sensorController();
  InputController& input = inputController();

  input.update();
  ledController().updateSpinner();
  sensor.updateCalibration();

  // Bundle calibration is entered from the host, not from a gesture. Button
  // activity used to jump straight to BundleState here, so a stray tap during
  // boot silently dropped the mouse into a serial protocol with no PC on the
  // other end -- it just looked hung. Requiring CAL_START makes the handoff
  // deliberate on both sides: the user opens the window, the host decides what
  // happens in it. See TODO/calibration-mode-entry.md.
  const char* line = serialController().takeLine();
  if (line != nullptr) {
    if (strcmp(line, "CAL_START") == 0) {
      stateMachine.changeState(&StateMachine::bundleState);
      return;
    }
    // Reboots on success, so it does not come back.
    CalibrationStorage::handleUploadCommand(line);
  }

  if (sensor.calibrationDone()) {
    stateMachine.changeState(&StateMachine::idleState);
  }
}

void CalibratingState::exit() {}
