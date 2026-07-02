#pragma once

#include <cstdio>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "Calibration.h"
#include "Protocol.h"
#include "SerialPort.h"

// The tuner UI. One instance, frame() called once per rendered frame with
// an active ImGui context.
class App {
 public:
  App();
  ~App();
  void frame();

 private:
  // --- connection / device model -------------------------------------
  void refreshPorts();
  void connect(const std::string& port);
  void disconnect();
  void pumpSerial();
  void send(const std::string& cmd);
  void handleResponse(const protocol::Response& r);
  void onSample(const protocol::StreamSample& s);

  // --- UI panels -------------------------------------------------------
  void drawConnectionPanel();
  void drawTuningPanel();
  void drawPlots();
  void drawPreview();
  void drawCalibrationWizard();
  void drawCrosstalkPanel();
  void drawConsole();

  void applyCalibrationToDevice();
  void exportConfigHeader();
  void startCsvLog();
  void stopCsvLog();

  // --- state -----------------------------------------------------------
  SerialPort port_;
  std::vector<std::string> availablePorts_;
  int selectedPort_ = 0;
  bool deviceAlive_ = false;
  std::string deviceId_;

  // Mirror of the device's runtime settings (name -> value).
  std::map<std::string, float> params_;
  double matrix_[6][9] = {};
  bool matrixValid_ = false;

  // Sample history for plots (bounded).
  std::deque<protocol::StreamSample> history_;
  double lastSampleMs_ = 0.0;

  std::vector<std::string> console_;

  // Calibration wizard.
  enum class WizStep {
    Idle,
    Rest,
    PressHold,
    TwistHold,
    SlideHold,
    SlideSweep,
    RimHold,
    RimSweep,
    Review
  };
  WizStep wizStep_ = WizStep::Idle;
  bool wizCapturing_ = false;
  double wizSettleUntilMs_ = 0.0;  // host clock, ms
  double wizCaptureUntilMs_ = 0.0;  // 0 = manual stop (sweeps)
  std::vector<calibration::Vec9>* wizTarget_ = nullptr;
  calibration::CaptureSet wizCaptures_;
  calibration::SolveResult wizResult_;
  bool wizApplied_ = false;

  void wizBeginCapture(std::vector<calibration::Vec9>* target,
                       double settleS, double durationS);
  void wizFinishCapture();
  double hostNowMs() const;

  // Crosstalk measurement.
  int ctAxis_ = 0;
  bool ctRecording_ = false;
  double ctUntilMs_ = 0.0;
  std::vector<std::array<float, 6>> ctSamples_;
  float ctResult_[6][6] = {};  // [driven][output] ratio, diag = 1
  bool ctHasRow_[6] = {};

  // CSV logging.
  FILE* csv_ = nullptr;
  std::string csvPath_;
};
