#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

#ifndef PIO_UNIT_TESTING
void setup() {
  // Initialize USB HID first
  hidController().begin();

  if (Config::ENABLE_TELEMETRY) {
    Serial.begin(115200);
    delay(1000);
  }

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
  stateMachine.update();
}
#endif
