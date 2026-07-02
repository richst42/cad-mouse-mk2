#include "controllers/MotionController.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Settings.h"

namespace {
enum AxisIndex {
  AXIS_TX = 0,
  AXIS_TY,
  AXIS_TZ,
  AXIS_RX,
  AXIS_RY,
  AXIS_RZ
};

const float kSqrt3Over3 = 0.57735027;
const float kSqrt3Over6 = 0.28867513;
const float kTwoSqrt3Over3 = 1.15470054;
const float kOneThird = 0.33333334;
}  // namespace

// Encodes the original hand-derived formulas as a matrix over the raw
// vector [mag1x mag1y mag1z mag2x mag2y mag2z mag3x mag3y mag3z]:
//   Tx, Ty, Tz = per-component average of the three sensors
//   Rx = sqrt(3) * (mag2z + mag3z - 2 * mag1z) / 3
//   Ry = mag3z - mag2z
//   Rz = sum_i (posXi * magYi - posYi * magXi) for the triangle layout
//        MAG1 bottom (0, -sqrt(3)/3), MAG2 top left (-0.5, sqrt(3)/6),
//        MAG3 top right (0.5, sqrt(3)/6)
const float MotionController::kDefaultMatrix[6][9] = {
    {kOneThird, 0, 0, kOneThird, 0, 0, kOneThird, 0, 0},
    {0, kOneThird, 0, 0, kOneThird, 0, 0, kOneThird, 0},
    {0, 0, kOneThird, 0, 0, kOneThird, 0, 0, kOneThird},
    {0, 0, -kTwoSqrt3Over3, 0, 0, kSqrt3Over3, 0, 0, kSqrt3Over3},
    {0, 0, 0, 0, 0, -1.0, 0, 0, 1.0},
    {kSqrt3Over3, 0, 0, -kSqrt3Over6, -0.5, 0, -kSqrt3Over6, 0.5, 0}};

void MotionController::reset() {
  for (int i = 0; i < 6; i++) {
    filt_[i] = 0.0;
  }
  motionActive_ = false;
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::lowpass(float prev, float x, float dt, float tau) {
  if (tau <= 0.0) return x;
  const float a = dt / (tau + dt);
  return prev + a * (x - prev);
}

void MotionController::compute(const float raw[9], const float* baseline,
                               float dt, float out[6]) {
  const DeviceSettings& s = settingsStore.data();

  // Baseline subtraction converts magnetic deltas around the calibrated
  // rest pose.
  float delta[9];
  for (int i = 0; i < 9; i++) {
    delta[i] = raw[i] - baseline[i];
  }

  // Map sensor deltas to a 6DoF pose. The calibrated matrix (from the
  // guided sweep procedure) cancels inter-axis coupling; without one the
  // geometry-derived default reproduces the original behavior.
  const float(*m)[9] = s.matrixValid ? s.matrix : kDefaultMatrix;

  float y[6];
  for (int i = 0; i < 6; i++) {
    float pose = 0.0;
    for (int j = 0; j < 9; j++) {
      pose += m[i][j] * delta[j];
    }
    const float gain = (i < 3) ? s.gainT[i] : s.gainR[i - 3];
    y[i] = s.signAxis[i] * pose * gain;
  }

  // Filter first so entering/leaving the dead band stays continuous, then
  // apply a soft dead band: output ramps from zero at the band edge instead
  // of snapping to the threshold value.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = (i < 3) ? s.deadT : s.deadR;

    filt_[i] = lowpass(filt_[i], y[i], dt, s.smoothTauS);

    const float mag = fabs(filt_[i]) - dead;
    if (mag <= 0.0) {
      out[i] = 0.0;
      continue;
    }

    const float span = Config::AXIS_LIMIT - dead;
    float n = (span > 0.0) ? clampf(mag / span, 0.0, 1.0) : 1.0;
    if (s.curveExp != 1.0) {
      n = pow(n, s.curveExp);
    }

    out[i] = (filt_[i] > 0.0 ? 1.0 : -1.0) * n * Config::AXIS_LIMIT;
    motionActive_ = true;
  }
}

bool MotionController::hasMotionActivity() const { return motionActive_; }
