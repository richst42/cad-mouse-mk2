#pragma once

#include <Arduino.h>

// Runtime-tunable settings. Config.h values are the boot defaults; anything
// changed over the serial command channel can be persisted to flash-backed
// EEPROM so the device keeps its tuning across power cycles.
struct DeviceSettings {
  float gainT[3];
  float gainR[3];
  float signAxis[6];  // +1.0 or -1.0 per axis
  float deadT;
  float deadR;
  float smoothTauS;
  float curveExp;  // response curve exponent, 1.0 = linear

  uint8_t rezeroEnabled;  // slow baseline re-zero while at rest
  float rezeroDelayS;     // how long the device must be quiet first
  float rezeroTauS;       // slew time constant once active

  // 6x9 decoupling matrix mapping baseline-subtracted sensor deltas to a
  // 6DoF pose. When matrixValid is 0 the built-in geometry-derived matrix
  // is used instead (equivalent to the original hand-derived formulas).
  uint8_t matrixValid;
  float matrix[6][9];
};

class SettingsStore {
 public:
  void begin();  // loads from EEPROM, falls back to defaults
  void loadDefaults();
  bool load();
  bool save();

  DeviceSettings& data() { return data_; }
  const DeviceSettings& data() const { return data_; }

 private:
  static uint32_t crc32(const uint8_t* buf, size_t len);

  DeviceSettings data_;
};

extern SettingsStore settingsStore;
