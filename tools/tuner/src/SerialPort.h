#pragma once

#include <string>
#include <vector>

// Thin Win32 serial wrapper. The device is a USB CDC port, so the baud rate
// is cosmetic. Reads are non-blocking and split into lines by poll().
class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  static std::vector<std::string> enumeratePorts();  // e.g. {"COM3", "COM7"}

  bool open(const std::string& portName);
  void close();
  bool isOpen() const;
  const std::string& portName() const { return portName_; }

  bool write(const std::string& data);

  // Reads whatever is available and appends complete lines (without CR/LF)
  // to `lines`. Returns false if the port died (unplugged).
  bool poll(std::vector<std::string>& lines);

 private:
  void* handle_ = nullptr;  // HANDLE, kept void* to keep windows.h out
  std::string portName_;
  std::string pending_;
};
