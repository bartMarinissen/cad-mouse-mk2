#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

#ifndef PIO_UNIT_TESTING
void setup() {
  // Initialize USB HID first
  hidController().begin();

  // Unconditional: Serial carries the calibration protocol as well as
  // telemetry, so the link has to come up either way -- and the boot
  // diagnostics from resolveCalibration() below need somewhere to land.
  // TelemetryController still decides for itself whether to *print*.
  serialController().begin();

  inputController().begin();
  ledController().begin();

  // Note: this is where the callibration is actually read for the sensor and the motion controller.
  if (!sensorController().begin()) {
    stateMachine.changeState(&StateMachine::errorState);
    return;
  }
  motionController().reset();
  telemetryController().begin();

  stateMachine.changeState(&StateMachine::calibratingState);
}

void loop() {
  hidController().task();
  // Before the state machine, so whichever state is active sees a line on the
  // same tick it finished arriving.
  serialController().update();
  stateMachine.update();
}
#endif
