#include "states/IdleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"
#include "animations/Animations.h"

void IdleState::enter() {
  lastUpdateMs_ = 0;
  lastActivityMs_ = millis();
  ledController().set(PoseColorAnimation(ledController().ring()));
}

bool IdleState::handleCalibrationRequest() {
  if (inputController().takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  SensorController& sensor = sensorController();
  MotionController& motion = motionController();

  float raw[9] = {};
  sensor.read_mT(raw);

  // Serial.printf(
  // "s1: %f %f %f \ns2: %f %f %f \ns3: %f %f %f \n",
  // raw[0], raw[1], raw[2],
  // raw[3], raw[4], raw[5],
  // raw[6], raw[7], raw[8]
  // );
  float motionOut[6] = {};
  float res_percent = motion.compute(raw, sensor.baseline(), dt, motionOut);

  if (motion.hasMotionActivity()) {
    lastActivityMs_ = now;
  }

  const uint16_t buttonBits = inputController().buttonBits();
  const bool hidReportSent = hidController().sendReports(motionOut, buttonBits);
  TelemetryController& telemetry = telemetryController();
  if (telemetry.enabled()) {
    // we have the raw field as raw
    // we can get the pose from motion.last_pos and motion.last_rot
    telemetry.publish(
      motionOut, res_percent, buttonBits, hidReportSent, motion.statistics, raw, motion.last_pos, motion.last_rot);
  }
}

void IdleState::handleSleepTransition(unsigned long now) {
  const unsigned long inactiveMs = now - lastActivityMs_;
  if (inactiveMs >= Config::IDLE_SLEEP_TIMEOUT_MS) {
    stateMachine.changeState(&StateMachine::sleepState);
  }
}

void IdleState::update() {
  InputController& input = inputController();
  input.update();

  if (handleCalibrationRequest()) {
    return;
  }

  const unsigned long now = millis();
  if (input.takeActivity()) {
    lastActivityMs_ = now;
  }

  const float dt = (lastUpdateMs_ == 0) ? 0.01
                                        : ((now - lastUpdateMs_) / 1000.0);
  lastUpdateMs_ = now;
  runMotionPipeline(dt, now);
  ledController().update();
  handleSleepTransition(now);
}

void IdleState::exit() {}
