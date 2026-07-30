#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

#ifndef PIO_UNIT_TESTING
InputController inputController;
LEDController ledController;
SensorController sensorController;
MotionController motionController;
HIDController hidController;
TelemetryController telemetryController;
BundleCalibrationController bundleCalibrationController{};

void setup() {
  // Initialize USB HID first
  hidController.begin();

  if (Config::ENABLE_TELEMETRY) {
    Serial.begin(115200);
    delay(1000);
  }

  inputController.begin();
  ledController.begin();
  if (!sensorController.begin()) {
    stateMachine.changeState(&StateMachine::errorState);
    return;
  }
  motionController.reset();
  telemetryController.begin();

  stateMachine.changeState(&StateMachine::calibratingState);
}

void loop() {
  hidController.task();
  stateMachine.update();
}
#endif
