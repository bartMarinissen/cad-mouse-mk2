#include "Controllers.h"

#include "CalibrationStorage.h"
#include "Config.h"

CalibrationParams resolveCalibration() {
  CalibrationParams stored;
  if (CalibrationStorage::load(stored)) {
    return stored;
  }
  return Config::defaultCalibration;
}

namespace {
InputController inputControllerInstance;
LEDController ledControllerInstance;
HIDController hidControllerInstance;
TelemetryController telemetryControllerInstance;
BundleCalibrationController bundleCalibrationControllerInstance;
SerialController serialControllerInstance;
}  // namespace

InputController& inputController() { return inputControllerInstance; }
LEDController& ledController() { return ledControllerInstance; }
HIDController& hidController() { return hidControllerInstance; }
TelemetryController& telemetryController() { return telemetryControllerInstance; }
BundleCalibrationController& bundleCalibrationController() { return bundleCalibrationControllerInstance; }
SerialController& serialController() { return serialControllerInstance; }

SensorController& sensorController() {
  // Function-local static rather than a namespace-scope global: it defers
  // construction until first use, which is what lets setup() decide *when*
  // the calibration is resolved, and sidesteps static-init order entirely.
  static SensorController instance(resolveCalibration());
  return instance;
}

MotionController& motionController() {
  static MotionController instance(resolveCalibration());
  return instance;
}
