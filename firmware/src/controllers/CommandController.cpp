#include "controllers/CommandController.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "Controllers.h"
#include "Settings.h"

namespace {
const int kProtocolVersion = 1;

struct ParamEntry {
  const char* name;
  float* value;
  float min;
  float max;
};

// Built fresh on each access because settingsStore is a global whose data
// lives at a fixed address; the table just points into it.
int paramTable(ParamEntry out[32]) {
  DeviceSettings& s = settingsStore.data();
  int n = 0;
  out[n++] = {"gain_tx", &s.gainT[0], 0.01, 1000.0};
  out[n++] = {"gain_ty", &s.gainT[1], 0.01, 1000.0};
  out[n++] = {"gain_tz", &s.gainT[2], 0.01, 1000.0};
  out[n++] = {"gain_rx", &s.gainR[0], 0.01, 1000.0};
  out[n++] = {"gain_ry", &s.gainR[1], 0.01, 1000.0};
  out[n++] = {"gain_rz", &s.gainR[2], 0.01, 1000.0};
  out[n++] = {"sign_tx", &s.signAxis[0], -1.0, 1.0};
  out[n++] = {"sign_ty", &s.signAxis[1], -1.0, 1.0};
  out[n++] = {"sign_tz", &s.signAxis[2], -1.0, 1.0};
  out[n++] = {"sign_rx", &s.signAxis[3], -1.0, 1.0};
  out[n++] = {"sign_ry", &s.signAxis[4], -1.0, 1.0};
  out[n++] = {"sign_rz", &s.signAxis[5], -1.0, 1.0};
  out[n++] = {"dead_t", &s.deadT, 0.0, 300.0};
  out[n++] = {"dead_r", &s.deadR, 0.0, 300.0};
  out[n++] = {"tau", &s.smoothTauS, 0.0, 1.0};
  out[n++] = {"curve", &s.curveExp, 0.25, 4.0};
  out[n++] = {"rezero_delay", &s.rezeroDelayS, 0.1, 60.0};
  out[n++] = {"rezero_tau", &s.rezeroTauS, 0.5, 300.0};
  return n;
}

bool nameEquals(const char* a, const char* b) { return strcmp(a, b) == 0; }
}  // namespace

void CommandController::begin() {
  bufLen_ = 0;
  zeroRequested_ = false;
}

bool CommandController::takeZeroRequest() {
  const bool requested = zeroRequested_;
  zeroRequested_ = false;
  return requested;
}

void CommandController::update() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (bufLen_ > 0) {
        buf_[bufLen_] = '\0';
        handleLine(buf_);
        bufLen_ = 0;
      }
      continue;
    }
    if (bufLen_ < kBufSize - 1) {
      buf_[bufLen_++] = c;
    } else {
      bufLen_ = 0;  // overlong line: discard
      Serial.println("ERR line too long");
    }
  }
}

void CommandController::printVal(const char* name, float value) {
  Serial.print("VAL ");
  Serial.print(name);
  Serial.print(' ');
  Serial.println(value, 4);
}

void CommandController::handleLine(char* line) {
  // Uppercase the verb only; parameter names are lowercase by convention.
  char* verb = line;
  char* argsStr = strchr(line, ' ');
  if (argsStr != nullptr) {
    *argsStr = '\0';
    argsStr++;
    while (*argsStr == ' ') argsStr++;
  }
  for (char* p = verb; *p; p++) {
    *p = toupper(*p);
  }

  if (nameEquals(verb, "PING")) {
    Serial.print("PONG cad-mouse-mk2 ");
    Serial.println(kProtocolVersion);
  } else if (nameEquals(verb, "GET")) {
    if (argsStr == nullptr) {
      Serial.println("ERR missing name");
      return;
    }
    handleGet(argsStr);
  } else if (nameEquals(verb, "SET")) {
    char* value = (argsStr != nullptr) ? strchr(argsStr, ' ') : nullptr;
    if (argsStr == nullptr || value == nullptr) {
      Serial.println("ERR usage: SET <name> <value>");
      return;
    }
    *value = '\0';
    value++;
    handleSet(argsStr, value);
  } else if (nameEquals(verb, "SAVE")) {
    Serial.println(settingsStore.save() ? "OK" : "ERR save failed");
  } else if (nameEquals(verb, "LOAD")) {
    if (!settingsStore.load()) {
      settingsStore.loadDefaults();
    }
    Serial.println("OK");
  } else if (nameEquals(verb, "DEFAULTS")) {
    settingsStore.loadDefaults();
    Serial.println("OK");
  } else if (nameEquals(verb, "ZERO")) {
    zeroRequested_ = true;
    Serial.println("OK");
  } else if (nameEquals(verb, "STREAM")) {
    if (argsStr == nullptr) {
      Serial.println("ERR usage: STREAM OFF|PLOT|OUT|FULL");
      return;
    }
    handleStream(argsStr);
  } else if (nameEquals(verb, "MAT")) {
    if (argsStr == nullptr) {
      Serial.println("ERR usage: MAT <row> <c0..c8>");
      return;
    }
    handleMatRow(argsStr);
  } else if (nameEquals(verb, "MAT?")) {
    dumpMatrix();
  } else if (nameEquals(verb, "MATON")) {
    settingsStore.data().matrixValid = 1;
    Serial.println("OK");
  } else if (nameEquals(verb, "MATOFF")) {
    settingsStore.data().matrixValid = 0;
    Serial.println("OK");
  } else {
    Serial.println("ERR unknown command");
  }
}

void CommandController::handleGet(const char* name) {
  ParamEntry params[32];
  const int count = paramTable(params);

  if (nameEquals(name, "*")) {
    for (int i = 0; i < count; i++) {
      printVal(params[i].name, *params[i].value);
    }
    printVal("rezero_on", settingsStore.data().rezeroEnabled);
    printVal("matvalid", settingsStore.data().matrixValid);
    Serial.println("OK");
    return;
  }

  if (nameEquals(name, "rezero_on")) {
    printVal(name, settingsStore.data().rezeroEnabled);
    return;
  }
  if (nameEquals(name, "matvalid")) {
    printVal(name, settingsStore.data().matrixValid);
    return;
  }
  for (int i = 0; i < count; i++) {
    if (nameEquals(name, params[i].name)) {
      printVal(params[i].name, *params[i].value);
      return;
    }
  }
  Serial.println("ERR unknown param");
}

void CommandController::handleSet(const char* name, const char* value) {
  char* end = nullptr;
  const float v = strtof(value, &end);
  if (end == value) {
    Serial.println("ERR bad value");
    return;
  }

  if (nameEquals(name, "rezero_on")) {
    settingsStore.data().rezeroEnabled = (v != 0.0) ? 1 : 0;
    Serial.println("OK");
    return;
  }

  ParamEntry params[32];
  const int count = paramTable(params);
  for (int i = 0; i < count; i++) {
    if (!nameEquals(name, params[i].name)) {
      continue;
    }
    if (v < params[i].min || v > params[i].max) {
      Serial.println("ERR out of range");
      return;
    }
    // Signs must be exactly +/-1.
    if (strncmp(name, "sign_", 5) == 0) {
      *params[i].value = (v < 0.0) ? -1.0 : 1.0;
    } else {
      *params[i].value = v;
    }
    Serial.println("OK");
    return;
  }
  Serial.println("ERR unknown param");
}

void CommandController::handleStream(const char* mode) {
  char upper[8];
  strncpy(upper, mode, sizeof(upper) - 1);
  upper[sizeof(upper) - 1] = '\0';
  for (char* p = upper; *p; p++) {
    *p = toupper(*p);
  }

  if (nameEquals(upper, "OFF")) {
    telemetryController.setMode(TelemetryController::Mode::Off);
  } else if (nameEquals(upper, "PLOT")) {
    telemetryController.setMode(TelemetryController::Mode::Plot);
  } else if (nameEquals(upper, "OUT")) {
    telemetryController.setMode(TelemetryController::Mode::Out);
  } else if (nameEquals(upper, "FULL")) {
    telemetryController.setMode(TelemetryController::Mode::Full);
  } else {
    Serial.println("ERR usage: STREAM OFF|PLOT|OUT|FULL");
    return;
  }
  Serial.println("OK");
}

void CommandController::handleMatRow(char* argsStr) {
  char* cursor = argsStr;
  char* end = nullptr;

  const long row = strtol(cursor, &end, 10);
  if (end == cursor || row < 0 || row > 5) {
    Serial.println("ERR bad row");
    return;
  }
  cursor = end;

  float values[9];
  for (int i = 0; i < 9; i++) {
    values[i] = strtof(cursor, &end);
    if (end == cursor) {
      Serial.println("ERR expected 9 values");
      return;
    }
    cursor = end;
  }

  for (int i = 0; i < 9; i++) {
    settingsStore.data().matrix[row][i] = values[i];
  }
  Serial.println("OK");
}

void CommandController::dumpMatrix() {
  const DeviceSettings& s = settingsStore.data();
  const float(*m)[9] = s.matrixValid ? s.matrix : MotionController::kDefaultMatrix;
  for (int r = 0; r < 6; r++) {
    Serial.print("MATROW ");
    Serial.print(r);
    for (int c = 0; c < 9; c++) {
      Serial.print(' ');
      Serial.print(m[r][c], 6);
    }
    Serial.println();
  }
  Serial.print("MATVALID ");
  Serial.println(s.matrixValid);
}
