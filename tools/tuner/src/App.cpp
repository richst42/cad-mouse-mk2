#include "App.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>

#include "imgui.h"
#include "implot.h"

namespace {

const double kAxisLimit = 350.0;

struct ParamMeta {
  const char* name;
  const char* label;
  float min;
  float max;
  bool log;
};

const ParamMeta kGainParams[] = {
    {"gain_tx", "Gain Tx", 0.05f, 100.0f, true},
    {"gain_ty", "Gain Ty", 0.05f, 100.0f, true},
    {"gain_tz", "Gain Tz", 0.05f, 100.0f, true},
    {"gain_rx", "Gain Rx", 0.05f, 100.0f, true},
    {"gain_ry", "Gain Ry", 0.05f, 100.0f, true},
    {"gain_rz", "Gain Rz", 0.05f, 100.0f, true},
};

const ParamMeta kFeelParams[] = {
    {"dead_t", "Dead zone T", 0.0f, 100.0f, false},
    {"dead_r", "Dead zone R", 0.0f, 100.0f, false},
    {"tau", "Smoothing tau [s]", 0.0f, 0.5f, false},
    {"curve", "Response curve exp", 0.25f, 4.0f, false},
    {"rezero_delay", "Re-zero delay [s]", 0.1f, 30.0f, false},
    {"rezero_tau", "Re-zero tau [s]", 0.5f, 120.0f, false},
};

const char* kAxisNames[6] = {"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"};
const char* kSignParams[6] = {"sign_tx", "sign_ty", "sign_tz",
                              "sign_rx", "sign_ry", "sign_rz"};
const char* kRawNames[9] = {"m1x", "m1y", "m1z", "m2x", "m2y",
                            "m2z", "m3x", "m3y", "m3z"};

struct SeriesRef {
  const std::deque<protocol::StreamSample>* history;
  int index;
  bool raw;  // raw channel vs output channel
};

ImPlotPoint seriesGetter(int idx, void* user) {
  const SeriesRef* ref = static_cast<const SeriesRef*>(user);
  const protocol::StreamSample& s = (*ref->history)[idx];
  const double t = s.tMs / 1000.0;
  if (ref->raw) {
    if (!s.hasRaw) return ImPlotPoint(t, NAN);
    return ImPlotPoint(t, s.raw[ref->index]);
  }
  return ImPlotPoint(t, s.out[ref->index]);
}

}  // namespace

App::App() { refreshPorts(); }

App::~App() {
  stopCsvLog();
  disconnect();
}

double App::hostNowMs() const {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch())
      .count();
}

// --------------------------------------------------------------------------
// Connection / device model
// --------------------------------------------------------------------------

void App::refreshPorts() {
  availablePorts_ = SerialPort::enumeratePorts();
  if (selectedPort_ >= static_cast<int>(availablePorts_.size())) {
    selectedPort_ = 0;
  }
}

void App::connect(const std::string& portName) {
  if (!port_.open(portName)) {
    console_.push_back("failed to open " + portName);
    return;
  }
  console_.push_back("opened " + portName);
  deviceAlive_ = false;
  history_.clear();
  send(protocol::cmdPing());
  send(protocol::cmdStream("FULL"));
  send(protocol::cmdGetAll());
  send(protocol::cmdMatDump());
}

void App::disconnect() {
  if (port_.isOpen()) {
    port_.write(protocol::cmdStream("OFF"));
    port_.close();
    console_.push_back("disconnected");
  }
  deviceAlive_ = false;
}

void App::send(const std::string& cmd) {
  if (!port_.isOpen()) return;
  port_.write(cmd);
  std::string logLine = "> " + cmd;
  if (!logLine.empty() && logLine.back() == '\n') logLine.pop_back();
  console_.push_back(logLine);
}

void App::pumpSerial() {
  if (!port_.isOpen()) return;

  std::vector<std::string> lines;
  if (!port_.poll(lines)) {
    console_.push_back("port lost");
    deviceAlive_ = false;
    return;
  }

  for (const std::string& line : lines) {
    protocol::StreamSample sample;
    if (protocol::parseStreamLine(line.c_str(), &sample)) {
      onSample(sample);
      continue;
    }
    protocol::Response response;
    if (protocol::parseResponseLine(line.c_str(), &response)) {
      handleResponse(response);
    }
  }

  if (console_.size() > 400) {
    console_.erase(console_.begin(), console_.end() - 300);
  }
}

void App::handleResponse(const protocol::Response& r) {
  using Kind = protocol::Response::Kind;
  switch (r.kind) {
    case Kind::Pong:
      deviceAlive_ = true;
      deviceId_ = r.text;
      console_.push_back("device: " + r.text);
      break;
    case Kind::Val:
      params_[r.name] = static_cast<float>(r.value);
      if (r.name == "matvalid") matrixValid_ = (r.value != 0.0);
      break;
    case Kind::MatRow:
      if (r.row >= 0 && r.row < 6) {
        for (int i = 0; i < 9; i++) matrix_[r.row][i] = r.rowValues[i];
      }
      break;
    case Kind::MatValid:
      matrixValid_ = (r.value != 0.0);
      break;
    case Kind::Ok:
      break;
    case Kind::Err:
      console_.push_back("ERR " + r.text);
      break;
    case Kind::Other:
      console_.push_back(r.text);
      break;
  }
}

void App::onSample(const protocol::StreamSample& s) {
  history_.push_back(s);
  while (history_.size() > 20000) history_.pop_front();
  lastSampleMs_ = s.tMs;

  const double now = hostNowMs();

  if (wizCapturing_ && s.hasRaw && wizTarget_ != nullptr) {
    if (now >= wizSettleUntilMs_) {
      calibration::Vec9 v;
      for (int i = 0; i < 9; i++) v[i] = s.raw[i];
      wizTarget_->push_back(v);
    }
    if (wizCaptureUntilMs_ > 0.0 && now >= wizCaptureUntilMs_) {
      wizFinishCapture();
    }
  }

  if (ctRecording_) {
    std::array<float, 6> o;
    for (int i = 0; i < 6; i++) o[i] = s.out[i];
    ctSamples_.push_back(o);
    if (now >= ctUntilMs_) {
      ctRecording_ = false;
      double onPeak = 0.0;
      for (const auto& v : ctSamples_) {
        onPeak = std::max(onPeak, static_cast<double>(std::fabs(v[ctAxis_])));
      }
      if (onPeak < 0.1 * kAxisLimit) {
        console_.push_back(
            "crosstalk: axis barely moved; row discarded (drive it to full "
            "deflection while recording)");
      } else {
        for (int i = 0; i < 6; i++) {
          double sum = 0.0;
          for (const auto& v : ctSamples_) sum += double(v[i]) * double(v[i]);
          const double rms = std::sqrt(sum / ctSamples_.size());
          ctResult_[ctAxis_][i] =
              (i == ctAxis_) ? 1.0f : static_cast<float>(rms / onPeak);
        }
        ctHasRow_[ctAxis_] = true;
      }
      ctSamples_.clear();
    }
  }

  if (csv_ != nullptr && s.hasRaw) {
    std::fprintf(csv_, "%.1f", s.tMs);
    for (int i = 0; i < 9; i++) std::fprintf(csv_, ",%.4f", s.raw[i]);
    for (int i = 0; i < 3; i++) std::fprintf(csv_, ",%.2f", s.temps[i]);
    for (int i = 0; i < 6; i++) std::fprintf(csv_, ",%.1f", s.out[i]);
    std::fprintf(csv_, ",%d\n", s.buttons);
  }
}

// --------------------------------------------------------------------------
// Frame
// --------------------------------------------------------------------------

void App::frame() {
  pumpSerial();

  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(360, 330), ImGuiCond_FirstUseEver);
  drawConnectionPanel();

  ImGui::SetNextWindowPos(ImVec2(10, 350), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(360, 470), ImGuiCond_FirstUseEver);
  drawTuningPanel();

  ImGui::SetNextWindowPos(ImVec2(380, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(620, 560), ImGuiCond_FirstUseEver);
  drawPlots();

  ImGui::SetNextWindowPos(ImVec2(1010, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(330, 420), ImGuiCond_FirstUseEver);
  drawPreview();

  ImGui::SetNextWindowPos(ImVec2(380, 580), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(620, 300), ImGuiCond_FirstUseEver);
  drawCalibrationWizard();

  ImGui::SetNextWindowPos(ImVec2(1010, 440), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(330, 260), ImGuiCond_FirstUseEver);
  drawCrosstalkPanel();

  ImGui::SetNextWindowPos(ImVec2(1010, 710), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(330, 170), ImGuiCond_FirstUseEver);
  drawConsole();
}

// --------------------------------------------------------------------------
// Panels
// --------------------------------------------------------------------------

void App::drawConnectionPanel() {
  ImGui::Begin("Connection");

  if (ImGui::Button("Refresh")) refreshPorts();
  ImGui::SameLine();

  const char* preview = availablePorts_.empty()
                            ? "<no ports>"
                            : availablePorts_[selectedPort_].c_str();
  ImGui::SetNextItemWidth(120);
  if (ImGui::BeginCombo("##port", preview)) {
    for (int i = 0; i < static_cast<int>(availablePorts_.size()); i++) {
      if (ImGui::Selectable(availablePorts_[i].c_str(), i == selectedPort_)) {
        selectedPort_ = i;
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine();

  if (!port_.isOpen()) {
    if (ImGui::Button("Connect") && !availablePorts_.empty()) {
      connect(availablePorts_[selectedPort_]);
    }
  } else {
    if (ImGui::Button("Disconnect")) disconnect();
  }

  ImGui::TextColored(
      port_.isOpen() ? ImVec4(0.3f, 0.9f, 0.3f, 1) : ImVec4(0.9f, 0.4f, 0.3f, 1),
      port_.isOpen() ? "connected: %s%s" : "not connected%s%s",
      port_.isOpen() ? port_.portName().c_str() : "",
      deviceAlive_ ? (" (" + deviceId_ + ")").c_str() : "");

  ImGui::Separator();

  if (ImGui::Button("Zero (re-center)")) send(protocol::cmdZero());
  ImGui::SameLine();
  if (ImGui::Button("Save to flash")) send(protocol::cmdSave());
  ImGui::SameLine();
  if (ImGui::Button("Reload")) {
    send(protocol::cmdLoad());
    send(protocol::cmdGetAll());
    send(protocol::cmdMatDump());
  }

  if (ImGui::Button("Factory defaults")) {
    send(protocol::cmdDefaults());
    send(protocol::cmdGetAll());
    send(protocol::cmdMatDump());
  }
  ImGui::SameLine();
  if (ImGui::Button("Export Config.h")) exportConfigHeader();

  ImGui::Separator();
  ImGui::Text("Decoupling matrix: %s", matrixValid_ ? "CALIBRATED" : "default");
  if (matrixValid_) {
    ImGui::SameLine();
    if (ImGui::SmallButton("disable")) {
      send(protocol::cmdMatOff());
      matrixValid_ = false;
    }
  }

  ImGui::Separator();
  if (csv_ == nullptr) {
    if (ImGui::Button("Start CSV log")) startCsvLog();
  } else {
    if (ImGui::Button("Stop CSV log")) stopCsvLog();
    ImGui::SameLine();
    ImGui::TextUnformatted(csvPath_.c_str());
  }

  ImGui::End();
}

void App::drawTuningPanel() {
  ImGui::Begin("Tuning");
  ImGui::TextDisabled("changes apply live; Save to flash to persist");

  auto slider = [&](const ParamMeta& p) {
    auto it = params_.find(p.name);
    if (it == params_.end()) return;
    float v = it->second;
    const ImGuiSliderFlags flags =
        p.log ? ImGuiSliderFlags_Logarithmic : ImGuiSliderFlags_None;
    if (ImGui::SliderFloat(p.label, &v, p.min, p.max, "%.3f", flags)) {
      it->second = v;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      send(protocol::cmdSet(p.name, v));
    }
  };

  if (ImGui::CollapsingHeader("Gains", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (const ParamMeta& p : kGainParams) slider(p);
  }

  if (ImGui::CollapsingHeader("Feel", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (const ParamMeta& p : kFeelParams) slider(p);

    auto it = params_.find("rezero_on");
    if (it != params_.end()) {
      bool on = it->second != 0.0f;
      if (ImGui::Checkbox("Auto re-zero at rest", &on)) {
        it->second = on ? 1.0f : 0.0f;
        send(protocol::cmdSet("rezero_on", on ? 1.0 : 0.0));
      }
    }
  }

  if (ImGui::CollapsingHeader("Axis directions",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    for (int i = 0; i < 6; i++) {
      auto it = params_.find(kSignParams[i]);
      if (it == params_.end()) continue;
      bool inverted = it->second < 0.0f;
      char label[32];
      std::snprintf(label, sizeof(label), "Invert %s", kAxisNames[i]);
      if (ImGui::Checkbox(label, &inverted)) {
        it->second = inverted ? -1.0f : 1.0f;
        send(protocol::cmdSet(kSignParams[i], inverted ? -1.0 : 1.0));
      }
      if ((i % 3) != 2) ImGui::SameLine();
    }
  }

  ImGui::End();
}

void App::drawPlots() {
  ImGui::Begin("Signals");

  const double tMax = history_.empty() ? 10.0 : lastSampleMs_ / 1000.0;
  const int count = static_cast<int>(history_.size());

  if (ImPlot::BeginPlot("Raw sensors [mT]", ImVec2(-1, 250))) {
    ImPlot::SetupAxes("t [s]", nullptr);
    ImPlot::SetupAxisLimits(ImAxis_X1, tMax - 10.0, tMax, ImGuiCond_Always);
    static SeriesRef refs[9];
    for (int i = 0; i < 9; i++) {
      refs[i] = {&history_, i, true};
      ImPlot::PlotLineG(kRawNames[i], seriesGetter, &refs[i], count);
    }
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("Outputs [counts]", ImVec2(-1, 250))) {
    ImPlot::SetupAxes("t [s]", nullptr);
    ImPlot::SetupAxisLimits(ImAxis_X1, tMax - 10.0, tMax, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -kAxisLimit, kAxisLimit,
                            ImGuiCond_Once);
    static SeriesRef refs[6];
    for (int i = 0; i < 6; i++) {
      refs[i] = {&history_, i, false};
      ImPlot::PlotLineG(kAxisNames[i], seriesGetter, &refs[i], count);
    }
    ImPlot::EndPlot();
  }

  ImGui::End();
}

void App::drawPreview() {
  ImGui::Begin("6DoF Preview");

  float out[6] = {};
  if (!history_.empty()) {
    for (int i = 0; i < 6; i++) out[i] = history_.back().out[i];
  }

  for (int i = 0; i < 6; i++) {
    const float frac = 0.5f + 0.5f * out[i] / static_cast<float>(kAxisLimit);
    char overlay[32];
    std::snprintf(overlay, sizeof(overlay), "%s %+.0f", kAxisNames[i], out[i]);
    ImGui::ProgressBar(frac, ImVec2(-1, 14), overlay);
  }

  // Cube gizmo driven by the live pose.
  ImGui::Separator();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float w = avail.x;
  const float h = std::max(120.0f, avail.y);
  const ImVec2 center(origin.x + w * 0.5f, origin.y + h * 0.5f);

  const float ax = out[3] / kAxisLimit * 0.6f;
  const float ay = out[4] / kAxisLimit * 0.6f;
  const float az = out[5] / kAxisLimit * 0.6f;
  const float tx = out[0] / kAxisLimit * 0.9f;
  const float ty = out[1] / kAxisLimit * 0.9f;
  const float tz = out[2] / kAxisLimit * 1.2f;

  const float cx = std::cos(ax), sx = std::sin(ax);
  const float cy = std::cos(ay), sy = std::sin(ay);
  const float cz = std::cos(az), sz = std::sin(az);

  ImVec2 proj[8];
  const int edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                            {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (int i = 0; i < 8; i++) {
    float x = (i & 1) ? 1.0f : -1.0f;
    float y = (i & 2) ? 1.0f : -1.0f;
    float z = (i & 4) ? 1.0f : -1.0f;

    // Rz * Ry * Rx, then translate.
    float y1 = y * cx - z * sx, z1 = y * sx + z * cx;
    float x2 = x * cy + z1 * sy, z2 = -x * sy + z1 * cy;
    float x3 = x2 * cz - y1 * sz, y3 = x2 * sz + y1 * cz;

    x3 += tx;
    y3 -= ty;    // screen y grows downward
    z2 += 5.0f - tz;  // camera distance; push/pull moves the cube

    const float f = 2.2f / std::max(1.0f, z2);
    const float scalePx = std::min(w, h) * 0.45f;
    proj[i] = ImVec2(center.x + x3 * f * scalePx, center.y + y3 * f * scalePx);
  }
  for (const auto& e : edges) {
    dl->AddLine(proj[e[0]], proj[e[1]], IM_COL32(120, 200, 255, 220), 2.0f);
  }
  // Mark the "front-top" corner so twist is visible.
  dl->AddCircleFilled(proj[5], 4.0f, IM_COL32(255, 180, 60, 255));

  ImGui::Dummy(ImVec2(w, h));
  ImGui::End();
}

void App::wizBeginCapture(std::vector<calibration::Vec9>* target,
                          double settleS, double durationS) {
  target->clear();
  wizTarget_ = target;
  const double now = hostNowMs();
  wizSettleUntilMs_ = now + settleS * 1000.0;
  wizCaptureUntilMs_ = (durationS > 0.0) ? wizSettleUntilMs_ + durationS * 1000.0
                                         : 0.0;
  wizCapturing_ = true;
}

void App::wizFinishCapture() {
  wizCapturing_ = false;
  wizTarget_ = nullptr;
}

void App::drawCalibrationWizard() {
  ImGui::Begin("Calibration Wizard");

  struct StepInfo {
    WizStep step;
    std::vector<calibration::Vec9>* target;
    const char* title;
    const char* instructions;
    double settleS;
    double durationS;  // 0 = manual finish (sweep)
  };
  StepInfo steps[] = {
      {WizStep::Rest, &wizCaptures_.rest, "1/7 Rest",
       "Hands OFF the device. Capturing the rest pose and noise floor.", 0.5,
       2.0},
      {WizStep::PressHold, &wizCaptures_.pressHold, "2/7 Press (Tz)",
       "Press the knob STRAIGHT DOWN as far as it goes and hold it there. "
       "Click Capture while holding.",
       0.7, 1.5},
      {WizStep::TwistHold, &wizCaptures_.twistHold, "3/7 Twist (Rz)",
       "TWIST the knob clockwise (seen from above) to the limit and hold. "
       "Try not to push or tilt.",
       0.7, 1.5},
      {WizStep::SlideHold, &wizCaptures_.slideHold, "4/7 Slide (Tx)",
       "PUSH the knob horizontally toward the RIGHT to the limit and hold. "
       "Keep it level - no tilt.",
       0.7, 1.5},
      {WizStep::SlideSweep, &wizCaptures_.slideSweep, "5/7 Slide circle (Ty)",
       "Keep the knob pushed to its limit and slowly trace 2 full "
       "horizontal circles, starting from the RIGHT and moving AWAY from "
       "you first. Click Finish when done.",
       0.5, 0.0},
      {WizStep::RimHold, &wizCaptures_.rimHold, "6/7 Rim press (Rx)",
       "Press DOWN on the FAR edge of the knob (away from you) so it tilts, "
       "and hold.",
       0.7, 1.5},
      {WizStep::RimSweep, &wizCaptures_.rimSweep, "7/7 Rim circle (Ry)",
       "Keep rim pressure and slowly walk the pressed point around the rim, "
       "2 full circles, starting from the FAR edge and moving toward the "
       "RIGHT first. Click Finish when done.",
       0.5, 0.0},
  };

  if (wizStep_ == WizStep::Idle) {
    ImGui::TextWrapped(
        "Guided calibration measures how YOUR unit's sensors respond to each "
        "motion and computes a decoupling matrix that cancels axis bleed. "
        "You do not need perfectly pure motions - each step only needs to "
        "roughly excite the right direction; the math does the separation.");
    ImGui::TextWrapped(
        "Takes about 2 minutes. The device keeps streaming; do a Zero first "
        "if the rest point looks off.");
    if (port_.isOpen() && ImGui::Button("Start calibration")) {
      wizCaptures_ = calibration::CaptureSet{};
      wizApplied_ = false;
      wizStep_ = WizStep::Rest;
    }
    if (!port_.isOpen()) ImGui::TextDisabled("connect to the device first");
    ImGui::End();
    return;
  }

  if (wizStep_ == WizStep::Review) {
    if (wizResult_.ok) {
      ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.3f, 1), "Solve OK");
      ImGui::Text("rest noise: %.4f  |  conditioning: %.1f",
                  wizResult_.restNoise, wizResult_.conditioning);
      ImGui::Text("plane separation - slide: %.1f, rim: %.1f  (higher=better)",
                  wizResult_.slidePlaneRatio, wizResult_.rimPlaneRatio);
      if (ImGui::BeginTable("axes", 3, ImGuiTableFlags_Borders)) {
        ImGui::TableSetupColumn("axis");
        ImGui::TableSetupColumn("full-scale delta");
        ImGui::TableSetupColumn("signal/noise");
        ImGui::TableHeadersRow();
        for (int i = 0; i < 6; i++) {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::TextUnformatted(kAxisNames[i]);
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("%.4f", wizResult_.axes[i].fullScale);
          ImGui::TableSetColumnIndex(2);
          ImGui::Text("%.0fx", wizResult_.axes[i].fullScale /
                                   std::max(1e-9, wizResult_.restNoise));
        }
        ImGui::EndTable();
      }
      if (!wizApplied_) {
        if (ImGui::Button("Apply to device")) {
          applyCalibrationToDevice();
          wizApplied_ = true;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("uploads matrix, sets gains to 1.0");
      } else {
        ImGui::TextWrapped(
            "Applied. Test the feel, flip any inverted axes in Tuning, then "
            "Save to flash. Dead zones can usually be lowered now.");
      }
    } else {
      ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.3f, 1), "Solve failed: %s",
                         wizResult_.error.c_str());
    }
    if (ImGui::Button("Restart")) wizStep_ = WizStep::Idle;
    ImGui::End();
    return;
  }

  // An active capture step.
  const StepInfo* info = nullptr;
  int stepIndex = 0;
  for (int i = 0; i < 7; i++) {
    if (steps[i].step == wizStep_) {
      info = &steps[i];
      stepIndex = i;
      break;
    }
  }

  ImGui::TextUnformatted(info->title);
  ImGui::TextWrapped("%s", info->instructions);
  ImGui::Separator();

  const size_t captured = info->target->size();
  if (wizCapturing_) {
    ImGui::Text("capturing... %d samples", static_cast<int>(captured));
    if (info->durationS == 0.0) {
      if (ImGui::Button("Finish") ||
          (wizCaptureUntilMs_ == 0.0 && captured > 4000)) {
        wizFinishCapture();
      }
    }
  } else {
    if (captured == 0) {
      if (ImGui::Button(info->durationS == 0.0 ? "Start sweep" : "Capture")) {
        wizBeginCapture(info->target, info->settleS, info->durationS);
      }
    } else {
      ImGui::Text("captured %d samples", static_cast<int>(captured));
      if (ImGui::Button("Next")) {
        if (stepIndex == 6) {
          wizResult_ = calibration::solve(wizCaptures_, kAxisLimit);
          wizStep_ = WizStep::Review;
        } else {
          wizStep_ = steps[stepIndex + 1].step;
        }
      }
      ImGui::SameLine();
      if (ImGui::Button("Redo")) {
        info->target->clear();
      }
    }
  }

  ImGui::Separator();
  if (ImGui::SmallButton("Cancel")) {
    wizFinishCapture();
    wizStep_ = WizStep::Idle;
  }

  ImGui::End();
}

void App::applyCalibrationToDevice() {
  for (int r = 0; r < 6; r++) {
    send(protocol::cmdMatRow(r, wizResult_.matrix[r]));
  }
  send(protocol::cmdMatOn());
  // The matrix already maps full deflection to full output; gains become
  // pure speed multipliers.
  for (int i = 0; i < 6; i++) {
    send(protocol::cmdSet(kGainParams[i].name, 1.0));
  }
  send(protocol::cmdGetAll());
  send(protocol::cmdMatDump());
  console_.push_back(
      "calibration applied (not yet saved - use Save to flash)");
}

void App::drawCrosstalkPanel() {
  ImGui::Begin("Crosstalk");
  ImGui::TextWrapped(
      "Pick an axis, click Record, and move ONLY that axis to full "
      "deflection for 5 seconds. Off-diagonal cells show bleed as %% of the "
      "driven axis.");

  ImGui::SetNextItemWidth(80);
  ImGui::Combo("axis", &ctAxis_, kAxisNames, 6);
  ImGui::SameLine();
  if (!ctRecording_) {
    if (ImGui::Button("Record 5s")) {
      ctSamples_.clear();
      ctRecording_ = true;
      ctUntilMs_ = hostNowMs() + 5000.0;
    }
  } else {
    ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.2f, 1), "recording...");
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Clear")) {
    std::memset(ctResult_, 0, sizeof(ctResult_));
    std::memset(ctHasRow_, 0, sizeof(ctHasRow_));
  }

  if (ImGui::BeginTable("ct", 7, ImGuiTableFlags_Borders)) {
    ImGui::TableSetupColumn("driven\\out");
    for (int i = 0; i < 6; i++) ImGui::TableSetupColumn(kAxisNames[i]);
    ImGui::TableHeadersRow();
    for (int r = 0; r < 6; r++) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(kAxisNames[r]);
      for (int c = 0; c < 6; c++) {
        ImGui::TableSetColumnIndex(c + 1);
        if (!ctHasRow_[r]) {
          ImGui::TextDisabled("-");
          continue;
        }
        const float v = ctResult_[r][c];
        if (r == c) {
          ImGui::TextDisabled("100");
          continue;
        }
        ImVec4 color = v < 0.05f   ? ImVec4(0.3f, 0.8f, 0.3f, 1)
                       : v < 0.15f ? ImVec4(0.9f, 0.8f, 0.2f, 1)
                                   : ImVec4(0.9f, 0.3f, 0.3f, 1);
        ImGui::TextColored(color, "%.1f", v * 100.0f);
      }
    }
    ImGui::EndTable();
  }

  ImGui::End();
}

void App::drawConsole() {
  ImGui::Begin("Console");

  const float footer = ImGui::GetFrameHeightWithSpacing();
  ImGui::BeginChild("scroll", ImVec2(0, -footer));
  for (const std::string& line : console_) {
    ImGui::TextUnformatted(line.c_str());
  }
  if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20) {
    ImGui::SetScrollHereY(1.0f);
  }
  ImGui::EndChild();

  static char input[128] = "";
  ImGui::SetNextItemWidth(-60);
  const bool entered = ImGui::InputText("##cmd", input, sizeof(input),
                                        ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::SameLine();
  if ((ImGui::Button("Send") || entered) && input[0] != '\0') {
    send(std::string(input) + "\n");
    input[0] = '\0';
  }

  ImGui::End();
}

// --------------------------------------------------------------------------
// Utilities
// --------------------------------------------------------------------------

void App::exportConfigHeader() {
  const char* path = "cadmouse_config_export.h";
  FILE* f = std::fopen(path, "w");
  if (f == nullptr) {
    console_.push_back("could not write cadmouse_config_export.h");
    return;
  }
  auto p = [&](const char* name, float fallback) {
    auto it = params_.find(name);
    return it != params_.end() ? it->second : fallback;
  };
  std::fprintf(f,
               "// Exported by the CAD Mouse MK2 tuner. Paste the values into "
               "firmware/include/Config.h\n"
               "const float GAIN_T[3] = {%.3f, %.3f, %.3f};\n"
               "const float GAIN_R[3] = {%.3f, %.3f, %.3f};\n"
               "const int SIGN_AXIS[6] = {%+.0f, %+.0f, %+.0f, %+.0f, %+.0f, "
               "%+.0f};\n"
               "const float DEAD_T = %.1f;\n"
               "const float DEAD_R = %.1f;\n"
               "const float SMOOTH_TAU_S = %.3f;\n"
               "const float CURVE_EXP = %.3f;\n"
               "const bool REZERO_ENABLED = %s;\n"
               "const float REZERO_DELAY_S = %.2f;\n"
               "const float REZERO_TAU_S = %.2f;\n",
               p("gain_tx", 28), p("gain_ty", 28), p("gain_tz", 24),
               p("gain_rx", 18), p("gain_ry", 18), p("gain_rz", 20),
               p("sign_tx", -1), p("sign_ty", 1), p("sign_tz", -1),
               p("sign_rx", 1), p("sign_ry", 1), p("sign_rz", 1),
               p("dead_t", 16), p("dead_r", 20), p("tau", 0.08f),
               p("curve", 1.0f), p("rezero_on", 1) != 0.0f ? "true" : "false",
               p("rezero_delay", 2.0f), p("rezero_tau", 10.0f));
  std::fclose(f);
  console_.push_back(std::string("wrote ") + path);
}

void App::startCsvLog() {
  csvPath_ = "cadmouse_log.csv";
  csv_ = std::fopen(csvPath_.c_str(), "w");
  if (csv_ == nullptr) {
    console_.push_back("could not open cadmouse_log.csv");
    return;
  }
  std::fprintf(csv_, "t_ms");
  for (int i = 0; i < 9; i++) std::fprintf(csv_, ",%s", kRawNames[i]);
  std::fprintf(csv_, ",temp1,temp2,temp3");
  for (int i = 0; i < 6; i++) std::fprintf(csv_, ",%s", kAxisNames[i]);
  std::fprintf(csv_, ",buttons\n");
  console_.push_back("logging to " + csvPath_);
}

void App::stopCsvLog() {
  if (csv_ != nullptr) {
    std::fclose(csv_);
    csv_ = nullptr;
    console_.push_back("log closed");
  }
}
