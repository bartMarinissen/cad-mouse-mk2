#pragma once

#include "CalibrationParams.h"
#include "controllers/HIDController.h"
#include "controllers/InputController.h"
#include "controllers/LEDController.h"
#include "controllers/MotionController.h"
#include "controllers/SensorController.h"
#include "controllers/TelemetryController.h"
#include "controllers/BundleCalibrationController.h"

// Controllers with no dependency on calibration. Plain globals, constructed at
// static-init time as before.
extern InputController inputController;
extern LEDController ledController;
extern HIDController hidController;
extern TelemetryController telemetryController;
extern BundleCalibrationController bundleCalibrationController;

// The two controllers that do depend on calibration, plus the calibration they
// were built from.
//
// They are grouped and constructed together because they cannot be static-init
// globals: their calibration-derived members are const, so they have to be
// *constructed from* a CalibrationParams, which means the calibration has to
// already exist -- and in Stage 2 it comes off the filesystem, which is not
// available until setup(). Grouping them also means there is exactly one moment
// at which the whole calibrated pipeline comes into existence, rather than a
// window where one controller has the stored calibration and the other still
// has defaults.
//
// Declaration order is load-bearing: `calibration` is initialized first, so the
// two controllers below can take it by const& in their constructors.
struct CalibratedControllers {
  const CalibrationParams calibration;
  SensorController sensor;
  MotionController motion;

  explicit CalibratedControllers(const CalibrationParams& cal);
};

// Where the calibration comes from at boot.
//
// Stage 1: always Config::defaultCalibration(). Stage 2 replaces the body with
// a LittleFS read that falls back to exactly that when there is no stored
// calibration, or when the stored one fails its CRC or plausibility checks.
CalibrationParams resolveCalibration();

// Built on first call, from resolveCalibration(). setup() forces that to happen
// at a known point, after the calibration source is available; everything else
// just uses whatever was built then.
CalibratedControllers& calibrated();
