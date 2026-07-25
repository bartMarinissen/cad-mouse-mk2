#pragma once

#include <Arduino.h>

namespace Config {

const bool ENABLE_TELEMETRY = true;

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
const float GAIN_T[3] = {1200.0, 1200.0, 3500.0};
// rotation gains        Rx,   Ry,   Rz
const float GAIN_R[3] = {800.0, 1000.0, 900.0};
// signs are for:         X,   Y,  Z, Rx, Ry, Rz
const int SIGN_AXIS[6] = {+1, -1, -1, +1, +1, -1};

// Dead zones
const float DEAD_T = 10.0;
const float DEAD_R = 15.0;

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

}  // namespace Config
