#include "controllers/SensorController.h"
#include "controllers/MotionController.h"
#include "Config.h"
// Need motion controller to get pose
extern MotionController motionController;


using namespace ifx::tlx493d;

SensorController::SensorController()
    : mag1Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      mag2Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      mag3Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      sensor_gain_{
        Config::magnet_gains[0] * Mat3::Identity(),
        Config::magnet_gains[1] * Mat3::Identity(),
        Config::magnet_gains[2] * Mat3::Identity(),
      },
      sensor_offset_mT_{
        Vec3(Config::sensor_offset_mT[0][0], Config::sensor_offset_mT[0][1], Config::sensor_offset_mT[0][2]),
        Vec3(Config::sensor_offset_mT[1][0], Config::sensor_offset_mT[1][1], Config::sensor_offset_mT[1][2]),
        Vec3(Config::sensor_offset_mT[2][0], Config::sensor_offset_mT[2][1], Config::sensor_offset_mT[2][2]),
      } {}

void SensorController::powerOff(int pin) { digitalWrite(pin, LOW); }

void SensorController::powerOn(int pin) {
  digitalWrite(pin, HIGH);
  delay(5);
}

bool SensorController::setup_sensor(ifx::tlx493d::TLx493D_A2B6& sensor, int pin, TLx493D_IICAddressType_t address) {
  SensorController::powerOn(pin);
  bool res = sensor.begin(true, false, true, true);
  sensor.printRegisters();
  if (!res) {
    Serial.println("Failed to initialize sensor!");
    return false;
  }
  res = sensor.setIICAddress(address);
  if (!res) {
    Serial.println("Failed to set sensor I2C address!");
    return false;
  }
  res = sensor.setSensitivity(TLx493D_FULL_RANGE_e);
  if (!res) {
    Serial.println("Failed to set sensor sensitivity!");
    return false;
  }
  res = sensor.setPowerMode(TLx493D_MASTER_CONTROLLED_MODE_e);
  if (!res) {
    Serial.println("Failed to set sensor power mode!");
    return false;
  }
  res = sensor.setTrigger(TLx493D_ADC_ON_READ_AFTER_REG_05_e);
  if (!res) {
    Serial.println("Failed to set sensor trigger mode!");
    return false;
  }
  // No conversion is running yet at this point (Master-Controlled Mode
  // starts powered down until triggered), so the first real read in the
  // main loop pays for that first conversion via clock stretching. If that
  // turns out to be too costly in practice, trigger one conversion here
  // explicitly (e.g. a throwaway getMagneticFieldAndTemperature() call) so
  // it's already in flight before the main loop starts reading.
  delay(10);
  return true;
}

bool SensorController::begin() {
 
  pinMode(Config::PIN_MAG1_LS, OUTPUT);
  pinMode(Config::PIN_MAG2_LS, OUTPUT);
  pinMode(Config::PIN_MAG3_LS, OUTPUT);

  // All three rails are pulled high in hardware, so force them all off first
  // before bringing sensors up one-by-one for address assignment.

  powerOff(Config::PIN_MAG1_LS);
  powerOff(Config::PIN_MAG2_LS);
  powerOff(Config::PIN_MAG3_LS);
  delay(5);

  
  Wire.begin();
  Wire.setClock(400000);

  powerOn(Config::PIN_MAG1_LS);

  bool res;
  res = setup_sensor(mag1Sensor_, Config::PIN_MAG1_LS, TLx493D_IIC_ADDR_A2_e);
  if (not res) {
    Serial.println("Failed to setup sensor 1!");
    return false;
  }
  res = setup_sensor(mag2Sensor_, Config::PIN_MAG2_LS, TLx493D_IIC_ADDR_A1_e);
  if (not res) {
    Serial.println("Failed to setup sensor 2!");
    return false;
  }
  res = setup_sensor(mag3Sensor_, Config::PIN_MAG3_LS, TLx493D_IIC_ADDR_A0_e);
  if (not res) {
    Serial.println("Failed to setup sensor 3!");
    return false;
  }
  Serial.println("All sensors initialized successfully.");
  Serial.println("sensor1 has valid data: " + String(mag1Sensor_.hasValidData()));
  Serial.println("sensor2 has valid data: " + String(mag2Sensor_.hasValidData()));
  Serial.println("sensor3 has valid data: " + String(mag3Sensor_.hasValidData()));

  Serial.println("sensor1 I2C address: " + String(mag1Sensor_.getI2CAddress()));
  Serial.println("sensor2 I2C address: " + String(mag2Sensor_.getI2CAddress()));
  Serial.println("sensor3 I2C address: " + String(mag3Sensor_.getI2CAddress()));
  return true;
}

void SensorController::readUncorrected(float out[9]) {
  double mag1x = 0, mag1y = 0, mag1z = 0, temp1 = 0;
  double mag2x = 0, mag2y = 0, mag2z = 0, temp2 = 0;
  double mag3x = 0, mag3y = 0, mag3z = 0, temp3 = 0;

  bool res_1 = mag1Sensor_.getMagneticFieldAndTemperature(&mag1x, &mag1y, &mag1z, &temp1);
  bool res_2 = mag2Sensor_.getMagneticFieldAndTemperature(&mag2x, &mag2y, &mag2z, &temp2);
  bool res_3 = mag3Sensor_.getMagneticFieldAndTemperature(&mag3x, &mag3y, &mag3z, &temp3);

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

void SensorController::read_mT(float out[9]) {
  float uncorrected[9];
  readUncorrected(uncorrected);

  for (int i = 0; i < 3; i++) {
    Vec3 corrected = sensor_gain_[i] * Vec3(uncorrected[i * 3 + 0], uncorrected[i * 3 + 1], uncorrected[i * 3 + 2])
                      - sensor_offset_mT_[i];
    out[i * 3 + 0] = corrected[0];
    out[i * 3 + 1] = corrected[1];
    out[i * 3 + 2] = corrected[2];
  }
}

void SensorController::beginCalibration() {
  calibrationActive_ = true;
  calibrationDone_ = false;
  calibrationSamples_ = 0;
  lastCalibrationSampleMs_ = 0;
  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] = 0.0;
  }
  calibration_pos = Vec3::Zero();
  calibration_rot = Vec3::Zero();
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
  read_mT(raw);

  Vec3 pos = Positions::approx_rest_pos - Vec3(0.1, 0.1, 0.1);
  Vec3 rot = Vec3::Zero();
  // Track pose aswell
  float res = motionController.read_pose(raw, pos, rot);
  calibration_pos += pos;
  calibration_rot += rot;
  Serial.printf("\ncalibrating intermediate pose : t= %f %f %f r= %f %f %f res=%f", 
    pos[0], pos[1], pos[2], 
    rot[0], rot[1], rot[2],
    res
  );

  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] += raw[i];
  }

  calibrationSamples_++;
  if (calibrationSamples_ < Config::ZERO_SAMPLES) {
    return;
  }
  // END OF Calibration RUN, return results

  calibration_pos /= Config::ZERO_SAMPLES;
  calibration_rot /= Config::ZERO_SAMPLES;
  Serial.printf("\ncalibrated pose : t= %f %f %f r= %f %f %f ", 
    calibration_pos[0], calibration_pos[1], calibration_pos[2], calibration_rot[0], calibration_rot[1], calibration_rot[2]
  );
  motionController.set_base_pose(calibration_pos, calibration_rot);

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
