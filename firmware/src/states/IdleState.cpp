#include "states/IdleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void IdleState::enter() {
  lastUpdateMs_ = 0;
  lastActivityMs_ = millis();
  ledController.setSolid(Config::LED_IDLE_COLOR);
}

bool IdleState::handleCalibrationRequest() {
  if (inputController.takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  CalibratedControllers& c = calibrated();

  float raw[9] = {};
  c.sensor.read_mT(raw);

  // Serial.printf(
  // "s1: %f %f %f \ns2: %f %f %f \ns3: %f %f %f \n", 
  // raw[0], raw[1], raw[2],
  // raw[3], raw[4], raw[5],
  // raw[6], raw[7], raw[8]
  // );
  float motion[6] = {};
  float res_percent = c.motion.compute(raw, c.sensor.baseline(), dt, motion);

  if (c.motion.hasMotionActivity()) {
    lastActivityMs_ = now;
  }

  const uint16_t buttonBits = inputController.buttonBits();
  const bool hidReportSent = hidController.sendReports(motion, buttonBits);
  if (telemetryController.enabled()) {
    // we have the raw field as raw
    // we can get the pose from c.motion.last_pos and c.motion.last_rot

    //auto eig_vals = c.motion.statistics.last_jacobian.jacobiSvd().singularValues();
    float rcond = 1.0;
    telemetryController.publish(
      motion, res_percent, buttonBits, hidReportSent, c.motion.statistics, raw, c.motion.last_pos, c.motion.last_rot, rcond);
  }
} 

void IdleState::handleSleepTransition(unsigned long now) {
  const unsigned long inactiveMs = now - lastActivityMs_;
  if (inactiveMs >= Config::IDLE_SLEEP_TIMEOUT_MS) {
    stateMachine.changeState(&StateMachine::sleepState);
  }
}

void IdleState::update() {
  inputController.update();

  if (handleCalibrationRequest()) {
    return;
  }

  const unsigned long now = millis();
  if (inputController.takeActivity()) {
    lastActivityMs_ = now;
  }

  const float dt = (lastUpdateMs_ == 0) ? 0.01
                                        : ((now - lastUpdateMs_) / 1000.0);
  lastUpdateMs_ = now;
  runMotionPipeline(dt, now);
  handleSleepTransition(now);
}

void IdleState::exit() {}
