#pragma once

#include <string>

// Text protocol shared with the firmware's CommandController /
// TelemetryController. Pure parsing/formatting — no I/O — so it can be unit
// tested off-target.
namespace protocol {

struct StreamSample {
  double tMs = 0.0;
  float raw[9] = {};
  float temps[3] = {};
  float out[6] = {};
  int buttons = 0;
  bool hasRaw = false;  // true for "D" (full) lines, false for "O" lines
};

// Parses "D <ms> <raw0..8> <t0..2> <out0..5> <btn>" or
// "O <ms> <out0..5> <btn>". Returns false for any other line.
bool parseStreamLine(const char* line, StreamSample* out);

struct Response {
  enum class Kind { Ok, Err, Val, Pong, MatRow, MatValid, Other };
  Kind kind = Kind::Other;
  std::string name;        // Val: parameter name
  double value = 0.0;      // Val / MatValid
  int row = 0;             // MatRow
  double rowValues[9] = {};  // MatRow
  std::string text;        // Err reason / Pong id / raw line for Other
};

// Parses a non-stream response line. Returns false if the line is a stream
// line (starts with "D ", "O " or ">").
bool parseResponseLine(const char* line, Response* out);

std::string cmdPing();
std::string cmdGetAll();
std::string cmdGet(const std::string& name);
std::string cmdSet(const std::string& name, double value);
std::string cmdSave();
std::string cmdLoad();
std::string cmdDefaults();
std::string cmdZero();
std::string cmdStream(const std::string& mode);  // "OFF" "PLOT" "OUT" "FULL"
std::string cmdMatRow(int row, const double values[9]);
std::string cmdMatOn();
std::string cmdMatOff();
std::string cmdMatDump();

}  // namespace protocol
