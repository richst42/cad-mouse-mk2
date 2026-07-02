#pragma once

class TelemetryController {
 public:
  enum class Mode {
    Off,   // no streaming
    Plot,  // Teleplot-style output values (original behavior)
    Out,   // "O <ms> <out0..out5> <btn>" per frame
    Full   // "D <ms> <raw0..raw8> <t1..t3> <out0..out5> <btn>" per frame
  };

  void begin();
  void publish(const float raw[9], const float temps[3],
               const float motion[6], int buttonBits, bool hidReportSent);
  bool enabled() const;

  void setMode(Mode mode);
  Mode mode() const;

 private:
  void publishPlot(const float motion[6], int buttonBits, bool hidReportSent);
  void publishOut(const float motion[6], int buttonBits);
  void publishFull(const float raw[9], const float temps[3],
                   const float motion[6], int buttonBits);

  Mode mode_ = Mode::Off;
  int tick_ = 0;
};
