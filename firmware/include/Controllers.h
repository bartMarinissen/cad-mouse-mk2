#pragma once

#include "CalibrationParams.h"
#include "controllers/HIDController.h"
#include "controllers/InputController.h"
#include "controllers/LEDController.h"
#include "controllers/MotionController.h"
#include "controllers/SensorController.h"
#include "controllers/TelemetryController.h"
#include "controllers/BundleCalibrationController.h"

// Every controller is reached through an accessor function, never a bare
// global -- one consistent access pattern regardless of how each one
// actually needs to be constructed.
//
// input/led/hid/telemetry/bundleCalibration are trivially default-
// constructible and have no dependency on anything resolved in setup(), so
// their accessors just return a reference to a static-init global -- same
// construction as a plain extern, just reached through a function.
InputController& inputController();
LEDController& ledController();
HIDController& hidController();
TelemetryController& telemetryController();
BundleCalibrationController& bundleCalibrationController();

// sensor/motion cannot be static-init globals: their calibration-derived
// members are const, so they have to be *constructed from* a
// CalibrationParams, which means the calibration has to already exist -- and
// in Stage 2 it comes off the filesystem, which is not available until
// setup(). Their accessors are real Meyers singletons (function-local
// static), which defers construction until first use.
//
// They resolve their own calibration independently rather than sharing one
// instance: neither holds a reference to a CalibrationParams or to the
// other, each only copies out the specific fields it needs at construction
// time, and resolveCalibration() is deterministic, so two independent calls
// at startup produce equal results. See resolveCalibration() below for what
// that costs.
SensorController& sensorController();
MotionController& motionController();

// Where the calibration comes from at boot.
//
// Stage 1: always Config::defaultCalibration(). Stage 2 replaces the body with
// a LittleFS read that falls back to exactly that when there is no stored
// calibration, or when the stored one fails its CRC or plausibility checks.
//
// Called once each by sensorController() and motionController() -- in Stage 2
// that's one extra flash read at boot over sharing a single resolved value,
// which is negligible next to USB enumeration latency.
CalibrationParams resolveCalibration();
