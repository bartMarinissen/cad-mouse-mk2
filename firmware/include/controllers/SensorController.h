#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <TLx493D_inc.hpp>
#include "CalibrationParams.h"
#include "math3D.h"

class SensorController {
 public:
  explicit SensorController(const CalibrationParams& cal);

  bool begin();
  // Gain-corrected reading, in mT. Used by anything feeding the pose solver.
  void read_mT(float out[9]);
  // True, uncorrected driver output. Used by the bundle-calibration data
  // capture path, which needs real sensor counts to fit a gain matrix from.
  void readUncorrected(float out[9]);

  void beginCalibration();
  void updateCalibration();
  bool calibrationDone() const;

  const float* baseline() const;

 private:
  static void powerOff(int pin);
  static void powerOn(int pin);
  static bool setup_sensor(ifx::tlx493d::TLx493D_A2B6& sensor, int pin, TLx493D_IICAddressType_t address);

  ifx::tlx493d::TLx493D_A2B6 mag1Sensor_;
  ifx::tlx493d::TLx493D_A2B6 mag2Sensor_;
  ifx::tlx493d::TLx493D_A2B6 mag3Sensor_;

  // millis() timestamp of the most recent readUncorrected() call. Used to
  // detect a gap since the last read -- see readUncorrected() in the .cpp
  // for why that matters with Master-Controlled Mode's trigger-on-read setup.
  unsigned long lastReadMs_ = 0;

  bool calibrationActive_ = false;
  bool calibrationDone_ = false;
  int calibrationSamples_ = 0;
  unsigned long lastCalibrationSampleMs_ = 0;
  float calibrationSum_[9] = {};
  float baseline_[9] = {};
  Vec3 calibration_pos = {};
  Vec3 calibration_rot = {};

  // Per-sensor gain/skew correction matrix and mT offset, applied to
  // readUncorrected()'s output to produce read_mT()'s. Const: they come from
  // the CalibrationParams this controller was constructed with, and a sensor
  // whose correction changed underneath a running pose solve would be a bug,
  // not a feature.
  const Mat3 sensor_gain_[3];
  const Vec3 sensor_offset_mT_[3];
};
