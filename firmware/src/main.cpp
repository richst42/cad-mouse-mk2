#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "Settings.h"
#include "StateMachine.h"

InputController inputController;
LEDController ledController;
SensorController sensorController;
MotionController motionController;
HIDController hidController;
TelemetryController telemetryController;
CommandController commandController;

void setup() {
  // Initialize USB HID first
  hidController.begin();

  // Serial carries both telemetry and the tuner command channel.
  Serial.begin(115200);
  delay(200);

  settingsStore.begin();

  inputController.begin();
  ledController.begin();
  sensorController.begin();
  motionController.reset();
  telemetryController.begin();
  commandController.begin();

  stateMachine.changeState(&StateMachine::calibratingState);
}

void loop() {
  hidController.task();
  commandController.update();

  if (commandController.takeZeroRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
  }

  stateMachine.update();
}
