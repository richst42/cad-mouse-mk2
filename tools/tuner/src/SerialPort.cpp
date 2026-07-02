#include "SerialPort.h"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace {
HANDLE toHandle(void* h) { return static_cast<HANDLE>(h); }
}  // namespace

SerialPort::~SerialPort() { close(); }

std::vector<std::string> SerialPort::enumeratePorts() {
  std::vector<std::string> ports;
  char target[256];
  for (int i = 1; i <= 64; i++) {
    const std::string name = "COM" + std::to_string(i);
    if (QueryDosDeviceA(name.c_str(), target, sizeof(target)) != 0) {
      ports.push_back(name);
    }
  }
  return ports;
}

bool SerialPort::open(const std::string& portName) {
  close();

  const std::string path = "\\\\.\\" + portName;
  HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                         nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) {
    return false;
  }

  DCB dcb = {};
  dcb.DCBlength = sizeof(dcb);
  GetCommState(h, &dcb);
  dcb.BaudRate = CBR_115200;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;  // CDC devices often gate on DTR
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  SetCommState(h, &dcb);

  // Non-blocking reads: return immediately with whatever is buffered.
  COMMTIMEOUTS timeouts = {};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutConstant = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 200;
  timeouts.WriteTotalTimeoutMultiplier = 1;
  SetCommTimeouts(h, &timeouts);

  PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

  handle_ = h;
  portName_ = portName;
  pending_.clear();
  return true;
}

void SerialPort::close() {
  if (handle_ != nullptr) {
    CloseHandle(toHandle(handle_));
    handle_ = nullptr;
  }
  portName_.clear();
  pending_.clear();
}

bool SerialPort::isOpen() const { return handle_ != nullptr; }

bool SerialPort::write(const std::string& data) {
  if (handle_ == nullptr) return false;
  DWORD written = 0;
  if (!WriteFile(toHandle(handle_), data.data(),
                 static_cast<DWORD>(data.size()), &written, nullptr)) {
    return false;
  }
  return written == data.size();
}

bool SerialPort::poll(std::vector<std::string>& lines) {
  if (handle_ == nullptr) return false;

  char buf[4096];
  for (;;) {
    DWORD bytesRead = 0;
    if (!ReadFile(toHandle(handle_), buf, sizeof(buf), &bytesRead, nullptr)) {
      close();
      return false;
    }
    if (bytesRead == 0) break;
    pending_.append(buf, bytesRead);
    if (bytesRead < sizeof(buf)) break;
  }

  // Split completed lines out of the pending buffer.
  size_t start = 0;
  for (;;) {
    const size_t nl = pending_.find('\n', start);
    if (nl == std::string::npos) break;
    std::string line = pending_.substr(start, nl - start);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!line.empty()) lines.push_back(std::move(line));
    start = nl + 1;
  }
  pending_.erase(0, start);

  // Guard against a runaway line with no terminator.
  if (pending_.size() > 65536) pending_.clear();
  return true;
}

#else  // !_WIN32 — stub so the core can be built/tested off Windows.

SerialPort::~SerialPort() { close(); }
std::vector<std::string> SerialPort::enumeratePorts() { return {}; }
bool SerialPort::open(const std::string&) { return false; }
void SerialPort::close() {}
bool SerialPort::isOpen() const { return false; }
bool SerialPort::write(const std::string&) { return false; }
bool SerialPort::poll(std::vector<std::string>&) { return false; }

#endif
