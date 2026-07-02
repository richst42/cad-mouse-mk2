#include "Protocol.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace protocol {
namespace {

// Reads `count` doubles from `cursor`, advancing it. Returns false on a
// malformed field.
bool readDoubles(const char*& cursor, double* out, int count) {
  for (int i = 0; i < count; i++) {
    char* end = nullptr;
    out[i] = std::strtod(cursor, &end);
    if (end == cursor) return false;
    cursor = end;
  }
  return true;
}

}  // namespace

bool parseStreamLine(const char* line, StreamSample* out) {
  const bool full = (line[0] == 'D' && line[1] == ' ');
  const bool outOnly = (line[0] == 'O' && line[1] == ' ');
  if (!full && !outOnly) return false;

  const char* cursor = line + 2;
  double t;
  if (!readDoubles(cursor, &t, 1)) return false;
  out->tMs = t;
  out->hasRaw = full;

  double values[19];
  const int expected = full ? 19 : 7;  // 9 raw + 3 temps + 6 out + btn | 6 out + btn
  if (!readDoubles(cursor, values, expected)) return false;

  if (full) {
    for (int i = 0; i < 9; i++) out->raw[i] = static_cast<float>(values[i]);
    for (int i = 0; i < 3; i++)
      out->temps[i] = static_cast<float>(values[9 + i]);
    for (int i = 0; i < 6; i++)
      out->out[i] = static_cast<float>(values[12 + i]);
    out->buttons = static_cast<int>(values[18]);
  } else {
    for (int i = 0; i < 9; i++) out->raw[i] = 0.0f;
    for (int i = 0; i < 3; i++) out->temps[i] = 0.0f;
    for (int i = 0; i < 6; i++) out->out[i] = static_cast<float>(values[i]);
    out->buttons = static_cast<int>(values[6]);
  }
  return true;
}

bool parseResponseLine(const char* line, Response* out) {
  if ((line[0] == 'D' || line[0] == 'O') && line[1] == ' ') return false;
  if (line[0] == '>') return false;

  if (std::strcmp(line, "OK") == 0) {
    out->kind = Response::Kind::Ok;
    return true;
  }
  if (std::strncmp(line, "ERR", 3) == 0) {
    out->kind = Response::Kind::Err;
    out->text = (line[3] == ' ') ? line + 4 : "";
    return true;
  }
  if (std::strncmp(line, "PONG", 4) == 0) {
    out->kind = Response::Kind::Pong;
    out->text = (line[4] == ' ') ? line + 5 : "";
    return true;
  }
  if (std::strncmp(line, "VAL ", 4) == 0) {
    const char* cursor = line + 4;
    const char* space = std::strchr(cursor, ' ');
    if (space == nullptr) return false;
    out->kind = Response::Kind::Val;
    out->name.assign(cursor, space - cursor);
    out->value = std::strtod(space + 1, nullptr);
    return true;
  }
  if (std::strncmp(line, "MATROW ", 7) == 0) {
    const char* cursor = line + 7;
    double row;
    if (!readDoubles(cursor, &row, 1)) return false;
    if (!readDoubles(cursor, out->rowValues, 9)) return false;
    out->kind = Response::Kind::MatRow;
    out->row = static_cast<int>(row);
    return true;
  }
  if (std::strncmp(line, "MATVALID ", 9) == 0) {
    out->kind = Response::Kind::MatValid;
    out->value = std::strtod(line + 9, nullptr);
    return true;
  }

  out->kind = Response::Kind::Other;
  out->text = line;
  return true;
}

std::string cmdPing() { return "PING\n"; }
std::string cmdGetAll() { return "GET *\n"; }
std::string cmdGet(const std::string& name) { return "GET " + name + "\n"; }

std::string cmdSet(const std::string& name, double value) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "SET %s %.6f\n", name.c_str(), value);
  return buf;
}

std::string cmdSave() { return "SAVE\n"; }
std::string cmdLoad() { return "LOAD\n"; }
std::string cmdDefaults() { return "DEFAULTS\n"; }
std::string cmdZero() { return "ZERO\n"; }

std::string cmdStream(const std::string& mode) {
  return "STREAM " + mode + "\n";
}

std::string cmdMatRow(int row, const double values[9]) {
  char buf[256];
  int len = std::snprintf(buf, sizeof(buf), "MAT %d", row);
  for (int i = 0; i < 9; i++) {
    len += std::snprintf(buf + len, sizeof(buf) - len, " %.8f", values[i]);
  }
  std::snprintf(buf + len, sizeof(buf) - len, "\n");
  return buf;
}

std::string cmdMatOn() { return "MATON\n"; }
std::string cmdMatOff() { return "MATOFF\n"; }
std::string cmdMatDump() { return "MAT?\n"; }

}  // namespace protocol
