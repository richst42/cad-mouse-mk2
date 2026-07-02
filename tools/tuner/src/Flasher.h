#pragma once

#include <string>

// UF2 firmware flasher for the RP2040.
//
// The RP2040 Arduino core reboots into the UF2 bootloader when its CDC port
// is opened at 1200 baud and closed again (the same trick the Arduino IDE
// uses). The bootloader then enumerates as a mass-storage drive labelled
// RPI-RP2, and flashing is just copying the .uf2 file onto it; the board
// reboots into the new firmware by itself.
//
// Drive detection and the copy are driven by tick() so the UI stays
// responsive; no threads.
class Flasher {
 public:
  enum class State { Idle, WaitingForDrive, Done, Error };

  // Opens a native "*.uf2" file picker. Empty string if cancelled.
  static std::string pickUf2File();

  // Starts a flash. comPort may be empty if the device is already in
  // bootloader mode (RPI-RP2 drive present); otherwise the 1200-baud touch
  // is sent first. The port must not be open elsewhere.
  void start(const std::string& comPort, const std::string& uf2Path);

  // Call once per frame. Polls for the bootloader drive and performs the
  // copy when it appears.
  void tick();

  State state() const { return state_; }
  const std::string& message() const { return message_; }
  void reset() { state_ = State::Idle; message_.clear(); }

 private:
  static bool touch1200(const std::string& comPort);
  static std::string findBootloaderDrive();  // "" if not present

  State state_ = State::Idle;
  std::string message_;
  std::string uf2Path_;
  double deadlineMs_ = 0.0;
  double nextPollMs_ = 0.0;
};
