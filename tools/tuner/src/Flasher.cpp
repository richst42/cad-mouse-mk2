#include "Flasher.h"

#include <chrono>

namespace {
double nowMs() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>

std::string Flasher::pickUf2File() {
  char path[MAX_PATH] = "";
  OPENFILENAMEA ofn = {};
  ofn.lStructSize = sizeof(ofn);
  ofn.lpstrFilter = "UF2 firmware (*.uf2)\0*.uf2\0All files (*.*)\0*.*\0";
  ofn.lpstrFile = path;
  ofn.nMaxFile = sizeof(path);
  ofn.lpstrTitle = "Select firmware .uf2";
  ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
  if (GetOpenFileNameA(&ofn) == 0) return "";
  return path;
}

bool Flasher::touch1200(const std::string& comPort) {
  const std::string path = "\\\\.\\" + comPort;
  HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                         nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) return false;

  DCB dcb = {};
  dcb.DCBlength = sizeof(dcb);
  GetCommState(h, &dcb);
  dcb.BaudRate = CBR_1200;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  SetCommState(h, &dcb);

  EscapeCommFunction(h, SETDTR);
  Sleep(100);
  EscapeCommFunction(h, CLRDTR);
  CloseHandle(h);
  return true;
}

std::string Flasher::findBootloaderDrive() {
  const DWORD drives = GetLogicalDrives();
  for (int i = 0; i < 26; i++) {
    if ((drives & (1u << i)) == 0) continue;
    const std::string root = std::string(1, static_cast<char>('A' + i)) + ":\\";
    if (GetDriveTypeA(root.c_str()) != DRIVE_REMOVABLE) continue;

    char label[MAX_PATH + 1] = "";
    if (GetVolumeInformationA(root.c_str(), label, sizeof(label), nullptr,
                              nullptr, nullptr, nullptr, 0) &&
        std::string(label) == "RPI-RP2") {
      return root;
    }
  }
  return "";
}

void Flasher::start(const std::string& comPort, const std::string& uf2Path) {
  uf2Path_ = uf2Path;
  message_.clear();

  if (findBootloaderDrive().empty()) {
    if (comPort.empty()) {
      state_ = State::Error;
      message_ = "no port selected and no RPI-RP2 drive found";
      return;
    }
    if (!touch1200(comPort)) {
      state_ = State::Error;
      message_ = "could not open " + comPort + " for the bootloader reset";
      return;
    }
    message_ = "rebooting into bootloader...";
  } else {
    message_ = "bootloader drive already present";
  }

  state_ = State::WaitingForDrive;
  deadlineMs_ = nowMs() + 30000.0;
  nextPollMs_ = 0.0;
}

void Flasher::tick() {
  if (state_ != State::WaitingForDrive) return;

  const double now = nowMs();
  if (now < nextPollMs_) return;
  nextPollMs_ = now + 500.0;

  const std::string drive = findBootloaderDrive();
  if (drive.empty()) {
    if (now > deadlineMs_) {
      state_ = State::Error;
      message_ =
          "RPI-RP2 drive never appeared. Unplug, hold BOOT while plugging "
          "back in, then retry.";
    } else {
      message_ = "waiting for RPI-RP2 drive...";
    }
    return;
  }

  message_ = "copying firmware...";
  const std::string dest = drive + "firmware.uf2";
  if (CopyFileA(uf2Path_.c_str(), dest.c_str(), FALSE) == 0) {
    state_ = State::Error;
    message_ = "copy to " + dest + " failed";
    return;
  }

  state_ = State::Done;
  message_ =
      "flashed - the device reboots by itself. Refresh ports and reconnect.";
}

#else  // !_WIN32 — stubs so off-Windows builds of the core still link.

std::string Flasher::pickUf2File() { return ""; }
bool Flasher::touch1200(const std::string&) { return false; }
std::string Flasher::findBootloaderDrive() { return ""; }

void Flasher::start(const std::string&, const std::string&) {
  state_ = State::Error;
  message_ = "flashing is only supported on Windows";
}

void Flasher::tick() {}

#endif
