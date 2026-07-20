#include "controllers/SensorController.h"

#include "Config.h"

using namespace ifx::tlx493d;

SensorController::SensorController()
    : mag1Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      mag2Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      mag3Sensor_(Wire, TLx493D_IIC_ADDR_A0_e) {}

void SensorController::powerOff(int pin) { digitalWrite(pin, LOW); }

void SensorController::powerOn(int pin) {
  digitalWrite(pin, HIGH);
  delay(5);
}

void SensorController::begin() {

  pinMode(Config::PIN_MAG1_LS, OUTPUT);
  pinMode(Config::PIN_MAG2_LS, OUTPUT);
  pinMode(Config::PIN_MAG3_LS, OUTPUT);

  // All three rails are pulled high in hardware, so force them all off first
  // before bringing sensors up one-by-one for address assignment.

  powerOff(Config::PIN_MAG1_LS);
  powerOff(Config::PIN_MAG2_LS);
  powerOff(Config::PIN_MAG3_LS);
  delay(5);

  bool magGood;

  Wire.begin();
  Wire.setClock(400000);

  powerOn(Config::PIN_MAG1_LS);
  magGood = mag1Sensor_.begin(true, false, true, true);
  if (!magGood) {
    Serial.println("Failed to initialize MAG1 sensor!");
  }
  magGood = mag1Sensor_.setIICAddress(TLx493D_IIC_ADDR_A2_e);
  if (!magGood) {
    Serial.println("Failed to set MAG1 sensor I2C address!");
  }
  magGood = mag1Sensor_.setSensitivity(TLx493D_SHORT_RANGE_e);
  if (!magGood) {
    Serial.println("Failed to set MAG1 sensor sensitivity!");
  }
  delay(100);

  powerOn(Config::PIN_MAG2_LS);
  magGood = mag2Sensor_.begin(true, false, true, true);
  if (!magGood) {
    Serial.println("Failed to initialize MAG2 sensor!");
  }
  magGood = mag2Sensor_.setIICAddress(TLx493D_IIC_ADDR_A1_e);
  if (!magGood) {
    Serial.println("Failed to set MAG2 sensor I2C address!");
  }
  magGood = mag2Sensor_.setSensitivity(TLx493D_SHORT_RANGE_e);
  if (!magGood) {
    Serial.println("Failed to set MAG2 sensor sensitivity!");
  }
  delay(100);

  powerOn(Config::PIN_MAG3_LS);
  magGood = mag3Sensor_.begin(true, false, true, true);
  if (!magGood) {
    Serial.println("Failed to initialize MAG3 sensor!");
  }
  magGood = mag3Sensor_.setSensitivity(TLx493D_SHORT_RANGE_e);
  if (!magGood) {
    Serial.println("Failed to set MAG3 sensor sensitivity!");
  }
  delay(100);

  delay(1000);
  Serial.println("sensor1 has valid data: " + String(mag1Sensor_.hasValidData()));
  Serial.println("sensor2 has valid data: " + String(mag2Sensor_.hasValidData()));
  Serial.println("sensor3 has valid data: " + String(mag3Sensor_.hasValidData()));

  Serial.println("sensor1 is functional: " + String(mag1Sensor_.isFunctional()));
  Serial.println("sensor2 is functional: " + String(mag2Sensor_.isFunctional()));
  Serial.println("sensor3 is functional: " + String(mag3Sensor_.isFunctional()));

  Serial.println("sensor1 I2C address: " + String(mag1Sensor_.getI2CAddress()));
  Serial.println("sensor2 I2C address: " + String(mag2Sensor_.getI2CAddress()));
  Serial.println("sensor3 I2C address: " + String(mag3Sensor_.getI2CAddress()));
}

void SensorController::readRaw(float out[9]) {
  double mag1x = 0, mag1y = 0, mag1z = 0, temp1 = 0;
  double mag2x = 0, mag2y = 0, mag2z = 0, temp2 = 0;
  double mag3x = 0, mag3y = 0, mag3z = 0, temp3 = 0;

  Serial.println("Reading raw sensor data...");
  Serial.flush();
  bool res_1 = mag1Sensor_.getMagneticFieldAndTemperature(&mag1x, &mag1y, &mag1z, &temp1);
  bool res_2 = mag2Sensor_.getMagneticFieldAndTemperature(&mag2x, &mag2y, &mag2z, &temp2);
  bool res_3 = mag3Sensor_.getMagneticFieldAndTemperature(&mag3x, &mag3y, &mag3z, &temp3);
  Serial.println("read raw sensor data, results:");
  Serial.print("MAG1: ");
  Serial.print(mag1x, 6);
  Serial.print(", MAG2: ");
  Serial.print(mag2x, 6);
  Serial.print(", MAG3: ");
  Serial.println(mag3x, 6);
  Serial.flush();

  // MAG1 = bottom, MAG2 = top left, MAG3 = top right.
  out[0] = mag1x;
  out[1] = mag1y;
  out[2] = mag1z;
  out[3] = mag2x;
  out[4] = mag2y;
  out[5] = mag2z;
  out[6] = mag3x;
  out[7] = mag3y;
  out[8] = mag3z;
}

void SensorController::beginCalibration() {
  calibrationActive_ = true;
  calibrationDone_ = false;
  calibrationSamples_ = 0;
  lastCalibrationSampleMs_ = 0;
  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] = 0.0;
  }
}

void SensorController::updateCalibration() {
  if (!calibrationActive_) {
    return;
  }

  const unsigned long now = millis();
  if (lastCalibrationSampleMs_ != 0 &&
      (now - lastCalibrationSampleMs_) < 10) {
    return;
  }
  lastCalibrationSampleMs_ = now;

  float raw[9] = {};
  readRaw(raw);

  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] += raw[i];
  }

  calibrationSamples_++;
  if (calibrationSamples_ < Config::ZERO_SAMPLES) {
    return;
  }

  for (int i = 0; i < 9; i++) {
    baseline_[i] = calibrationSum_[i] / Config::ZERO_SAMPLES;
  }

  Serial.println("Calibration complete. Baseline values:");
  for (int i = 0; i < 9; i++) {
    Serial.print(baseline_[i], 6);
    if (i < 8) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
  Serial.flush();
  calibrationActive_ = false;
  calibrationDone_ = true;
}

bool SensorController::calibrationDone() const { return calibrationDone_; }

const float* SensorController::baseline() const { return baseline_; }
