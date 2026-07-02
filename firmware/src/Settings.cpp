#include "Settings.h"

#include <EEPROM.h>
#include <string.h>

#include "Config.h"

SettingsStore settingsStore;

namespace {
const uint32_t kMagic = 0x43444D32;  // "CDM2"
const uint16_t kVersion = 2;
const int kEepromSize = 1024;

struct PersistHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
};
}  // namespace

void SettingsStore::loadDefaults() {
  for (int i = 0; i < 3; i++) {
    data_.gainT[i] = Config::GAIN_T[i];
    data_.gainR[i] = Config::GAIN_R[i];
  }
  for (int i = 0; i < 6; i++) {
    data_.signAxis[i] = static_cast<float>(Config::SIGN_AXIS[i]);
  }
  data_.deadT = Config::DEAD_T;
  data_.deadR = Config::DEAD_R;
  data_.smoothTauS = Config::SMOOTH_TAU_S;
  data_.curveExp = Config::CURVE_EXP;

  data_.rezeroEnabled = Config::REZERO_ENABLED ? 1 : 0;
  data_.rezeroDelayS = Config::REZERO_DELAY_S;
  data_.rezeroTauS = Config::REZERO_TAU_S;

  data_.ledBrightness = Config::LED_BRIGHTNESS;
  data_.ledIdleColor = Config::LED_IDLE_COLOR;
  data_.ledCalColor = Config::LED_CALIBRATING_COLOR;

  data_.matrixValid = 0;
  memset(data_.matrix, 0, sizeof(data_.matrix));
}

void SettingsStore::begin() {
  EEPROM.begin(kEepromSize);
  loadDefaults();
  load();
}

bool SettingsStore::load() {
  PersistHeader header;
  EEPROM.get(0, header);
  if (header.magic != kMagic || header.version != kVersion ||
      header.size != sizeof(DeviceSettings)) {
    return false;
  }

  DeviceSettings candidate;
  EEPROM.get(sizeof(PersistHeader), candidate);

  uint32_t storedCrc;
  EEPROM.get(sizeof(PersistHeader) + sizeof(DeviceSettings), storedCrc);

  const uint32_t computed =
      crc32(reinterpret_cast<const uint8_t*>(&candidate), sizeof(candidate));
  if (computed != storedCrc) {
    return false;
  }

  data_ = candidate;
  return true;
}

bool SettingsStore::save() {
  PersistHeader header;
  header.magic = kMagic;
  header.version = kVersion;
  header.size = sizeof(DeviceSettings);

  const uint32_t crc =
      crc32(reinterpret_cast<const uint8_t*>(&data_), sizeof(data_));

  EEPROM.put(0, header);
  EEPROM.put(sizeof(PersistHeader), data_);
  EEPROM.put(sizeof(PersistHeader) + sizeof(DeviceSettings), crc);
  return EEPROM.commit();
}

uint32_t SettingsStore::crc32(const uint8_t* buf, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (-(int32_t)(crc & 1)));
    }
  }
  return ~crc;
}
