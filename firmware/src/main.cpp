#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

#ifndef PIO_UNIT_TESTING
InputController inputController;
LEDController ledController;
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

  // Builds the calibrated controllers, from resolveCalibration(). Done here,
  // explicitly, rather than being left to whichever call site happens to touch
  // calibrated() first -- in Stage 2 this is the point at which the filesystem
  // gets read, and it needs to be after Serial is up so a failed load can say
  // so, and before anything starts solving poses.
  CalibratedControllers& c = calibrated();

  if (!c.sensor.begin()) {
    stateMachine.changeState(&StateMachine::errorState);
    return;
  }
  c.motion.reset();
  telemetryController.begin();

  stateMachine.changeState(&StateMachine::calibratingState);
}

void loop() {
  hidController.task();
  stateMachine.update();
}
#endif
