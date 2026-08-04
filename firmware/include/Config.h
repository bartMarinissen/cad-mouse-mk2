#pragma once

#include <Arduino.h>

#include "CalibrationParams.h"

namespace Config {

const bool ENABLE_TELEMETRY = true;
const bool statistics = true;

// Hardware pins (XIAO RP2040)
const int PIN_RIGHT_BTN = D0;
const int PIN_LEFT_BTN = D2;
const int PIN_LED_DATA = D3;
const int PIN_LED_LS = D1;
const int PIN_MAG1_LS = D10;
const int PIN_MAG2_LS = D9;
const int PIN_MAG3_LS = D8;

// Samples for calibration offset
const int ZERO_SAMPLES = 200;

// AXES:
// X axis - side to side    through the buttons
// Y axis - front to back   through the USB-C port
// Z axis - vertical        through the centre nob axis of symmetry

// Gains and sign fixes
// translation gains     X,    Y,    Z      
// in 'points' per mm             
const float GAIN_T[3] = {80.0, 80.0, 100.0};
// rotation gains        Rx,   Ry,   Rz
// in 'points' per degree
const float GAIN_R[3] = {15, 15, 15};
// signs are for:         X,   Y,  Z, Rx, Ry, Rz
const int SIGN_AXIS[6] = {+1, -1, +1, -1, -1, -1};

// Dead zones in 'points
const float DEAD_T = 18.0;
const float DEAD_R = 14.0;

// Smoothing
const float SMOOTH_TAU_S = 0.08;

// Final axis output range
const float AXIS_LIMIT = 350.0;

// RGB LEDs
const int LED_COUNT = 8;
const int LED_BRIGHTNESS = 40;
const unsigned long LED_IDLE_COLOR = 0x00FF00;
const unsigned long LED_CALIBRATING_COLOR = 0x0000FF;
const unsigned long LED_ERROR_COLOR = 0xFF0000;

// FSM timing
const long IDLE_SLEEP_TIMEOUT_MS = 2 * 60 * 1000;

const float magnet_gains[3] = {-0.96, -1.2, -0.98};

// Per-sensor XYZ offset, in mT, subtracted after the gain matrix is applied
// (i.e. in read_mT()'s output space, not raw sensor counts).
const float sensor_offset_mT[3][3] = {
  {0.1f,  -1.4f, 0.0f},
  {-1.6f, -0.2f, 0.0f},
  {0.0f,   0.0f, 0.0f},
};

// The constants above, assembled into the struct the rest of the firmware
// actually consumes. This is the fallback used when there is no calibration
// stored on the device, so it has to stand on its own as a working (if
// uncalibrated) unit -- identity magnet rotations and unit magnet strengths,
// with magnet positions taken from the nominal CAD geometry in positions.h.
//
// A function rather than a constant on purpose: CalibrationParams holds Eigen
// types, and a namespace-scope instance would be built during static init, in
// an order nothing here controls.
CalibrationParams defaultCalibration();

}  // namespace Config
