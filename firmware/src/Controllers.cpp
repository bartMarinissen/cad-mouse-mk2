#include "Controllers.h"

#include "Config.h"

CalibratedControllers::CalibratedControllers(const CalibrationParams& cal)
    : calibration(cal), sensor(calibration), motion(calibration) {}

CalibrationParams resolveCalibration() {
  // Stage 2 hook: read /calibration.bin off LittleFS here, validate it, and
  // fall back to the line below when there is nothing stored or what is stored
  // does not survive its CRC and plausibility checks.
  return Config::defaultCalibration();
}

CalibratedControllers& calibrated() {
  // Function-local static rather than a namespace-scope global: it defers
  // construction until first use, which is what lets setup() decide *when* the
  // calibrated pipeline is built, and sidesteps static-init order entirely.
  static CalibratedControllers instance{resolveCalibration()};
  return instance;
}
