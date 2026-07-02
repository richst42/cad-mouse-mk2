#pragma once

#include <Arduino.h>

// Line-based serial command channel used by the desktop tuner app.
//
//   PING                     -> PONG cad-mouse-mk2 <protocol version>
//   GET <name> | GET *       -> VAL <name> <value>
//   SET <name> <value>       -> OK | ERR <reason>
//   SAVE / LOAD / DEFAULTS   -> persist / reload / reset settings
//   ZERO                     -> re-run the rest-pose zero calibration
//   STREAM OFF|PLOT|OUT|FULL -> select telemetry stream mode
//   MAT <row> <c0..c8>       -> stage one decoupling matrix row
//   MATON / MATOFF / MAT?    -> enable / disable / dump the matrix
class CommandController {
 public:
  void begin();
  void update();

  bool takeZeroRequest();

 private:
  void handleLine(char* line);
  void handleGet(const char* name);
  void handleSet(const char* name, const char* value);
  void handleStream(const char* mode);
  void handleMatRow(char* argsStr);
  void dumpMatrix();
  void printVal(const char* name, float value);

  static const int kBufSize = 256;
  char buf_[kBufSize];
  int bufLen_ = 0;
  bool zeroRequested_ = false;
};
