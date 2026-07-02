#include "controllers/TelemetryController.h"

#include <Arduino.h>

#include "Config.h"

namespace {
const int kPlotPrintEvery = 5;
}

void TelemetryController::begin() {
  tick_ = 0;
  mode_ = Config::ENABLE_TELEMETRY ? Mode::Plot : Mode::Off;
}

bool TelemetryController::enabled() const { return mode_ != Mode::Off; }

void TelemetryController::setMode(Mode mode) { mode_ = mode; }

TelemetryController::Mode TelemetryController::mode() const { return mode_; }

void TelemetryController::publish(const float raw[9], const float temps[3],
                                  const float motion[6], int buttonBits,
                                  bool hidReportSent) {
  switch (mode_) {
    case Mode::Off:
      return;
    case Mode::Plot:
      publishPlot(motion, buttonBits, hidReportSent);
      return;
    case Mode::Out:
      publishOut(motion, buttonBits);
      return;
    case Mode::Full:
      publishFull(raw, temps, motion, buttonBits);
      return;
  }
}

void TelemetryController::publishPlot(const float motion[6], int buttonBits,
                                      bool hidReportSent) {
  tick_++;
  if ((tick_ % kPlotPrintEvery) != 0) {
    return;
  }

  Serial.print(">X:");
  Serial.println(motion[0]);
  Serial.print(">Y:");
  Serial.println(motion[1]);
  Serial.print(">Z:");
  Serial.println(motion[2]);
  Serial.print(">Rx:");
  Serial.println(motion[3]);
  Serial.print(">Ry:");
  Serial.println(motion[4]);
  Serial.print(">Rz:");
  Serial.println(motion[5]);
  Serial.print(">btn:");
  Serial.println(buttonBits & 0x0003);
  Serial.print(">hid:");
  Serial.println(hidReportSent ? 1 : 0);
}

void TelemetryController::publishOut(const float motion[6], int buttonBits) {
  Serial.print("O ");
  Serial.print(millis());
  for (int i = 0; i < 6; i++) {
    Serial.print(' ');
    Serial.print(motion[i], 1);
  }
  Serial.print(' ');
  Serial.println(buttonBits & 0x0003);
}

void TelemetryController::publishFull(const float raw[9], const float temps[3],
                                      const float motion[6], int buttonBits) {
  Serial.print("D ");
  Serial.print(millis());
  for (int i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(raw[i], 3);
  }
  for (int i = 0; i < 3; i++) {
    Serial.print(' ');
    Serial.print(temps[i], 2);
  }
  for (int i = 0; i < 6; i++) {
    Serial.print(' ');
    Serial.print(motion[i], 1);
  }
  Serial.print(' ');
  Serial.println(buttonBits & 0x0003);
}
