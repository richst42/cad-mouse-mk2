#pragma once

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;

  // Geometry-derived fallback mapping, used when no calibrated matrix is
  // stored. Exposed so the command channel can report it.
  static const float kDefaultMatrix[6][9];

 private:
  static float clampf(float v, float lo, float hi);
  static float lowpass(float prev, float x, float dt, float tau);
  float filt_[6] = {};
  bool motionActive_ = false;
};
