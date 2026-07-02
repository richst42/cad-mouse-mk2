// Off-target tests for the tuner's math core and protocol parsing.
// Build: g++ -std=c++17 -O2 -I../src test_core.cpp ../src/LinAlg.cpp
//        ../src/Calibration.cpp ../src/Protocol.cpp -o test_core && ./test_core
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "Calibration.h"
#include "LinAlg.h"
#include "Protocol.h"

namespace {

int failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
      failures++;                                                      \
    }                                                                  \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                          \
  do {                                                                 \
    const double a_ = (a), b_ = (b), tol_ = (tol);                     \
    if (std::fabs(a_ - b_) > tol_) {                                   \
      std::printf("FAIL %s:%d: %s=%g vs %s=%g (tol %g)\n", __FILE__,   \
                  __LINE__, #a, a_, #b, b_, tol_);                     \
      failures++;                                                      \
    }                                                                  \
  } while (0)

using linalg::Vec9;

std::mt19937 rng(42);

double noise(double sigma) {
  std::normal_distribution<double> d(0.0, sigma);
  return d(rng);
}

// ---------------------------------------------------------------------------
// Synthetic device: a ground-truth 9x6 mixing matrix A with deliberate
// inter-axis coupling and per-axis scale differences. Sensor deltas for a
// pose p are A * p + noise.
// ---------------------------------------------------------------------------
struct SyntheticDevice {
  double A[9][6];

  SyntheticDevice() {
    std::uniform_real_distribution<double> u(-0.3, 0.3);
    // Distinct dominant patterns per axis plus random cross-coupling.
    const double base[9][6] = {
        // Tx    Ty    Tz    Rx    Ry    Rz
        {1.0, 0.05, 0.02, 0.10, 0.00, 0.30},
        {0.05, 1.0, 0.03, 0.00, 0.12, 0.02},
        {0.02, 0.03, 1.0, -0.60, 0.05, 0.01},
        {0.90, 0.10, 0.02, 0.15, 0.08, -0.25},
        {0.03, 1.1, 0.02, 0.05, 0.10, -0.40},
        {0.05, 0.02, 0.9, 0.35, -0.50, 0.03},
        {1.1, 0.02, 0.05, 0.12, -0.10, -0.20},
        {0.02, 0.9, 0.01, 0.03, 0.15, 0.45},
        {0.03, 0.05, 1.1, 0.30, 0.55, 0.02}};
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 6; j++) A[i][j] = base[i][j] + 0.05 * u(rng);
  }

  Vec9 sample(const double pose[6], double sigma) const {
    Vec9 s{};
    for (int i = 0; i < 9; i++) {
      for (int j = 0; j < 6; j++) s[i] += A[i][j] * pose[j];
      s[i] += noise(sigma);
    }
    return s;
  }
};

// Human-imperfect gestures: every hold and sweep leaks a bit of unintended
// pose onto other axes.
void makeCaptures(const SyntheticDevice& dev, calibration::CaptureSet* c) {
  const double kNoise = 0.004;
  const double kLeak = 0.08;  // 8% contamination of other axes

  std::uniform_real_distribution<double> leak(-kLeak, kLeak);

  auto contaminated = [&](int axis, double amount) {
    double pose[6];
    for (int j = 0; j < 6; j++) pose[j] = amount * leak(rng);
    pose[axis] = amount;
    return dev.sample(pose, kNoise);
  };

  for (int i = 0; i < 60; i++) {
    double zero[6] = {0, 0, 0, 0, 0, 0};
    c->rest.push_back(dev.sample(zero, kNoise));
  }

  // Full deflections differ per axis (springs are not symmetric).
  for (int i = 0; i < 80; i++) c->slideHold.push_back(contaminated(0, 1.0));
  for (int i = 0; i < 80; i++) c->pressHold.push_back(contaminated(2, -0.8));
  for (int i = 0; i < 80; i++) c->rimHold.push_back(contaminated(3, 0.6));
  for (int i = 0; i < 80; i++) c->twistHold.push_back(contaminated(5, 0.7));

  // Sweeps: two slow circles starting at the hold pose, moving so that the
  // second axis initially goes positive; slightly elliptical and wobbly.
  for (int i = 0; i < 400; i++) {
    const double th = 2.0 * (2.0 * M_PI) * i / 400.0;
    double pose[6] = {0, 0, 0, 0, 0, 0};
    pose[0] = 1.0 * std::cos(th);
    pose[1] = 0.85 * std::sin(th);
    pose[2] = 0.05 * std::sin(3.0 * th);  // incidental push while circling
    pose[5] = 0.04 * std::cos(2.0 * th);
    c->slideSweep.push_back(dev.sample(pose, kNoise));
  }
  for (int i = 0; i < 400; i++) {
    const double th = 2.0 * (2.0 * M_PI) * i / 400.0;
    double pose[6] = {0, 0, 0, 0, 0, 0};
    pose[3] = 0.6 * std::cos(th);
    pose[4] = 0.5 * std::sin(th);
    pose[2] = -0.06 * std::fabs(std::sin(th));  // rim pressing pushes down
    c->rimSweep.push_back(dev.sample(pose, kNoise));
  }
}

void testLinAlg() {
  // invert6 on a known matrix: inv(A)*A == I.
  double a[6][6];
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) a[i][j] = u(rng) + (i == j ? 3.0 : 0.0);

  double inv[6][6];
  CHECK(linalg::invert6(a, inv));
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      double s = 0.0;
      for (int k = 0; k < 6; k++) s += inv[i][k] * a[k][j];
      CHECK_NEAR(s, i == j ? 1.0 : 0.0, 1e-9);
    }
  }

  // Jacobi eigen on a matrix with a known spectrum: diag(9,4,1) rotated.
  double m[9][9] = {};
  m[0][0] = 5.0;
  m[0][1] = m[1][0] = 2.0;
  m[1][1] = 5.0;
  m[2][2] = 1.0;
  double eigval[9], eigvec[9][9];
  linalg::jacobiEigen(m, 3, eigval, eigvec);
  CHECK_NEAR(eigval[0], 7.0, 1e-9);
  CHECK_NEAR(eigval[1], 3.0, 1e-9);
  CHECK_NEAR(eigval[2], 1.0, 1e-9);

  // pinv9x6: M*J == I6.
  double J[9][6];
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 6; j++) J[i][j] = u(rng) + ((i % 6) == j ? 2.0 : 0.0);
  double M[6][9];
  CHECK(linalg::pinv9x6(J, M));
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      double s = 0.0;
      for (int k = 0; k < 9; k++) s += M[i][k] * J[k][j];
      CHECK_NEAR(s, i == j ? 1.0 : 0.0, 1e-9);
    }
  }
}

void testCalibration() {
  SyntheticDevice dev;
  calibration::CaptureSet captures;
  makeCaptures(dev, &captures);

  const double kAxisLimit = 350.0;
  const calibration::SolveResult result =
      calibration::solve(captures, kAxisLimit);
  if (!result.ok) std::printf("solve error: %s\n", result.error.c_str());
  CHECK(result.ok);

  // Apply the solved matrix to pure ground-truth motions and measure
  // crosstalk: off-axis output relative to on-axis output.
  const double trueFull[6] = {1.0, 0.85, 0.8, 0.6, 0.5, 0.7};
  double worst = 0.0;
  for (int axis = 0; axis < 6; axis++) {
    double pose[6] = {0, 0, 0, 0, 0, 0};
    pose[axis] = trueFull[axis];
    const Vec9 delta = dev.sample(pose, 0.0);

    double out[6];
    for (int i = 0; i < 6; i++) {
      out[i] = 0.0;
      for (int j = 0; j < 9; j++) out[i] += result.matrix[i][j] * delta[j];
    }

    const double on = std::fabs(out[axis]);
    CHECK(on > 0.5 * kAxisLimit);  // full deflection lands near full scale
    for (int i = 0; i < 6; i++) {
      if (i == axis) continue;
      const double ratio = std::fabs(out[i]) / on;
      if (ratio > worst) worst = ratio;
    }
  }
  std::printf("worst crosstalk after calibration: %.1f%%\n", worst * 100.0);
  CHECK(worst < 0.12);  // leaks in the captures were 8%; must not amplify

  // Compare with the uncalibrated result: identity-style per-axis readout of
  // the same device has to be much worse for this synthetic coupling.
  CHECK(result.slidePlaneRatio > 5.0);
  CHECK(result.rimPlaneRatio > 5.0);
  CHECK(result.conditioning < 50.0);
}

void testProtocol() {
  protocol::StreamSample s;
  CHECK(protocol::parseStreamLine(
      "D 12345 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 25.5 26.0 24.9 10 -20 30 "
      "-40 50 -60 3",
      &s));
  CHECK(s.hasRaw);
  CHECK_NEAR(s.tMs, 12345.0, 1e-9);
  CHECK_NEAR(s.raw[8], 0.9, 1e-6);
  CHECK_NEAR(s.temps[1], 26.0, 1e-6);
  CHECK_NEAR(s.out[5], -60.0, 1e-6);
  CHECK(s.buttons == 3);

  CHECK(protocol::parseStreamLine("O 99 1 2 3 4 5 6 1", &s));
  CHECK(!s.hasRaw);
  CHECK_NEAR(s.out[2], 3.0, 1e-9);
  CHECK(s.buttons == 1);

  CHECK(!protocol::parseStreamLine("VAL gain_tx 28.0", &s));

  protocol::Response r;
  CHECK(protocol::parseResponseLine("VAL gain_tx 28.0", &r));
  CHECK(r.kind == protocol::Response::Kind::Val);
  CHECK(r.name == "gain_tx");
  CHECK_NEAR(r.value, 28.0, 1e-9);

  CHECK(protocol::parseResponseLine("OK", &r));
  CHECK(r.kind == protocol::Response::Kind::Ok);

  CHECK(protocol::parseResponseLine("ERR out of range", &r));
  CHECK(r.kind == protocol::Response::Kind::Err);
  CHECK(r.text == "out of range");

  CHECK(protocol::parseResponseLine("MATROW 2 1 0 0 0 0 0 0 0 0.5", &r));
  CHECK(r.kind == protocol::Response::Kind::MatRow);
  CHECK(r.row == 2);
  CHECK_NEAR(r.rowValues[8], 0.5, 1e-9);

  CHECK(!protocol::parseResponseLine("D 1 2 3", &r));
  CHECK(!protocol::parseResponseLine(">X:1.0", &r));

  // Round-trip: a command built here parses on the firmware side (spot
  // check the formatting).
  const double row[9] = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8, 0.9};
  const std::string cmd = protocol::cmdMatRow(3, row);
  CHECK(cmd.rfind("MAT 3 ", 0) == 0);
  CHECK(cmd.back() == '\n');
}

}  // namespace

int main() {
  testLinAlg();
  testCalibration();
  testProtocol();

  if (failures == 0) {
    std::printf("all tests passed\n");
    return 0;
  }
  std::printf("%d failure(s)\n", failures);
  return 1;
}
