#include "states/IdleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "Settings.h"
#include "StateMachine.h"

void IdleState::enter() {
  lastUpdateMs_ = 0;
  lastActivityMs_ = millis();
  lastMotionMs_ = lastActivityMs_;
  ledController.setSolid(
      static_cast<unsigned long>(settingsStore.data().ledIdleColor));
}

bool IdleState::handleCalibrationRequest() {
  if (inputController.takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  float raw[9] = {};
  sensorController.readRaw(raw);

  float motion[6] = {};
  motionController.compute(raw, sensorController.baseline(), dt, motion);

  if (motionController.hasMotionActivity()) {
    lastActivityMs_ = now;
    lastMotionMs_ = now;
  } else {
    // Quiet long enough: slowly re-learn the rest pose to absorb thermal
    // drift and spring settling without a manual recalibration.
    const DeviceSettings& s = settingsStore.data();
    if (s.rezeroEnabled &&
        (now - lastMotionMs_) >= (unsigned long)(s.rezeroDelayS * 1000.0)) {
      sensorController.slewBaseline(raw, dt, s.rezeroTauS);
    }
  }

  const uint16_t buttonBits = inputController.buttonBits();
  const bool hidReportSent = hidController.sendReports(motion, buttonBits);
  if (telemetryController.enabled()) {
    telemetryController.publish(raw, sensorController.temperatures(), motion,
                                buttonBits, hidReportSent);
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
