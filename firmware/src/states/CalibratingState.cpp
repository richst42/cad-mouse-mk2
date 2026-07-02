#include "states/CalibratingState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "Settings.h"
#include "StateMachine.h"

void CalibratingState::enter() {
  sensorController.beginCalibration();
  motionController.reset();
  ledController.startSpinner(
      static_cast<unsigned long>(settingsStore.data().ledCalColor));
}

void CalibratingState::update() {
  inputController.update();
  ledController.updateSpinner();
  sensorController.updateCalibration();

  if (sensorController.calibrationDone()) {
    stateMachine.changeState(&StateMachine::idleState);
  }
}

void CalibratingState::exit() {}
