#include "Controllers.h"

#include "Config.h"

CalibrationParams resolveCalibration() {
  // Stage 2 hook: read /calibration.bin off LittleFS here, validate it, and
  // fall back to the line below when there is nothing stored or what is stored
  // does not survive its CRC and plausibility checks.
  return Config::defaultCalibration();
}

namespace {
InputController inputControllerInstance;
LEDController ledControllerInstance;
HIDController hidControllerInstance;
TelemetryController telemetryControllerInstance;
BundleCalibrationController bundleCalibrationControllerInstance;
}  // namespace

InputController& inputController() { return inputControllerInstance; }
LEDController& ledController() { return ledControllerInstance; }
HIDController& hidController() { return hidControllerInstance; }
TelemetryController& telemetryController() { return telemetryControllerInstance; }
BundleCalibrationController& bundleCalibrationController() { return bundleCalibrationControllerInstance; }

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
