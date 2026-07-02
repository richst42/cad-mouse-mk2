#include "Calibration.h"

#include <algorithm>
#include <cmath>

namespace calibration {
namespace {

using linalg::Vec9;

const double kMinAmplitude = 1e-9;

double rms(const std::vector<Vec9>& samples, const Vec9& about) {
  if (samples.empty()) return 0.0;
  double sum = 0.0;
  for (const Vec9& s : samples) {
    const Vec9 d = linalg::sub(s, about);
    sum += linalg::dot(d, d);
  }
  return std::sqrt(sum / static_cast<double>(samples.size()));
}

std::vector<Vec9> deltas(const std::vector<Vec9>& samples, const Vec9& rest) {
  std::vector<Vec9> out;
  out.reserve(samples.size());
  for (const Vec9& s : samples) out.push_back(linalg::sub(s, rest));
  return out;
}

double percentileAbs(std::vector<double> values, double pct) {
  if (values.empty()) return 0.0;
  for (double& v : values) v = std::fabs(v);
  std::sort(values.begin(), values.end());
  const size_t idx = static_cast<size_t>(pct * (values.size() - 1));
  return values[idx];
}

bool solveHold(const std::vector<Vec9>& samples, const Vec9& rest,
               const char* name, AxisSolve* out, std::string* error) {
  if (samples.empty()) {
    *error = std::string(name) + ": no samples captured";
    return false;
  }
  const Vec9 delta = linalg::sub(linalg::mean(samples), rest);
  const double amplitude = linalg::norm(delta);
  if (amplitude < kMinAmplitude) {
    *error = std::string(name) + ": deflection too small";
    return false;
  }
  out->direction = linalg::normalize(delta);
  out->fullScale = amplitude;
  return true;
}

// Extracts the sweep-plane axis orthogonal to `anchor`.
//
// PCA gives the plane spanned by the two dominant directions. The anchor
// (from the paired hold) is projected into the plane; the second axis is
// the in-plane direction orthogonal to it. Its sign is chosen so that the
// sweep's early travel — which starts at the anchor pose and moves in the
// announced direction — heads toward +axis2.
bool solveSweep(const std::vector<Vec9>& samples, const Vec9& rest,
                const Vec9& anchor, const char* name, AxisSolve* out,
                double* planeRatio, std::string* error) {
  if (samples.size() < 16) {
    *error = std::string(name) + ": not enough sweep samples";
    return false;
  }

  const std::vector<Vec9> d = deltas(samples, rest);

  Vec9 dirs[3];
  double eig[3];
  linalg::pca(d, 3, dirs, eig);

  if (eig[1] < kMinAmplitude) {
    *error = std::string(name) + ": sweep did not trace a plane";
    return false;
  }
  *planeRatio = (eig[2] > 1e-15) ? (eig[1] / eig[2]) : 1e9;

  // Anchor projected into the plane.
  Vec9 w{};
  const double a1 = linalg::dot(anchor, dirs[0]);
  const double a2 = linalg::dot(anchor, dirs[1]);
  for (int i = 0; i < 9; i++) w[i] = a1 * dirs[0][i] + a2 * dirs[1][i];
  if (linalg::norm(w) < 0.2) {
    *error = std::string(name) +
             ": hold direction is not in the sweep plane; redo the pair";
    return false;
  }
  w = linalg::normalize(w);

  // In-plane direction orthogonal to w.
  Vec9 e2 = linalg::sub(dirs[0], linalg::scale(w, linalg::dot(dirs[0], w)));
  if (linalg::norm(e2) < 1e-6) {
    e2 = linalg::sub(dirs[1], linalg::scale(w, linalg::dot(dirs[1], w)));
  }
  if (linalg::norm(e2) < 1e-6) {
    *error = std::string(name) + ": degenerate sweep plane";
    return false;
  }
  e2 = linalg::normalize(e2);

  // Fix the sign of e2 from the direction of early travel: accumulate the
  // wrapped phase increment over the first third of the sweep.
  double signAccum = 0.0;
  const size_t third = std::max<size_t>(2, d.size() / 3);
  double prevTheta = std::atan2(linalg::dot(d[0], e2), linalg::dot(d[0], w));
  for (size_t i = 1; i < third; i++) {
    const double theta =
        std::atan2(linalg::dot(d[i], e2), linalg::dot(d[i], w));
    double dTheta = theta - prevTheta;
    while (dTheta > M_PI) dTheta -= 2.0 * M_PI;
    while (dTheta < -M_PI) dTheta += 2.0 * M_PI;
    signAccum += dTheta;
    prevTheta = theta;
  }
  if (signAccum < 0.0) e2 = linalg::scale(e2, -1.0);

  std::vector<double> projections;
  projections.reserve(d.size());
  for (const Vec9& s : d) projections.push_back(linalg::dot(s, e2));
  const double amplitude = percentileAbs(projections, 0.95);
  if (amplitude < kMinAmplitude) {
    *error = std::string(name) + ": no deflection along the second axis";
    return false;
  }

  out->direction = e2;
  out->fullScale = amplitude;
  return true;
}

}  // namespace

SolveResult solve(const CaptureSet& captures, double axisLimit) {
  SolveResult result;

  if (captures.rest.size() < 8) {
    result.error = "rest: not enough samples";
    return result;
  }
  const Vec9 rest = linalg::mean(captures.rest);
  result.restNoise = rms(captures.rest, rest);

  // Axis order: Tx Ty Tz Rx Ry Rz.
  if (!solveHold(captures.slideHold, rest, "slide hold (Tx)", &result.axes[0],
                 &result.error) ||
      !solveHold(captures.pressHold, rest, "press hold (Tz)", &result.axes[2],
                 &result.error) ||
      !solveHold(captures.rimHold, rest, "rim hold (Rx)", &result.axes[3],
                 &result.error) ||
      !solveHold(captures.twistHold, rest, "twist hold (Rz)", &result.axes[5],
                 &result.error)) {
    return result;
  }

  if (!solveSweep(captures.slideSweep, rest, result.axes[0].direction,
                  "slide sweep (Ty)", &result.axes[1],
                  &result.slidePlaneRatio, &result.error) ||
      !solveSweep(captures.rimSweep, rest, result.axes[3].direction,
                  "rim sweep (Ry)", &result.axes[4], &result.rimPlaneRatio,
                  &result.error)) {
    return result;
  }

  double J[9][6];
  for (int col = 0; col < 6; col++) {
    for (int row = 0; row < 9; row++) {
      J[row][col] = result.axes[col].direction[row];
    }
  }

  double M0[6][9];
  if (!linalg::pinv9x6(J, M0)) {
    result.error =
        "axis directions are not independent; two gestures excited the same "
        "sensor pattern";
    return result;
  }

  // Conditioning diagnostic: sqrt of the eigenvalue spread of J^T J.
  {
    double jtj[9][9] = {};
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        double s = 0.0;
        for (int k = 0; k < 9; k++) s += J[k][i] * J[k][j];
        jtj[i][j] = s;
      }
    }
    double eigval[9];
    double eigvec[9][9];
    linalg::jacobiEigen(jtj, 6, eigval, eigvec);
    if (eigval[5] > 1e-15) {
      result.conditioning = std::sqrt(eigval[0] / eigval[5]);
    } else {
      result.conditioning = 1e9;
    }
  }

  // Scale rows so a full deflection produces axisLimit counts; device gains
  // then default to 1.0 and act as pure user speed multipliers.
  for (int i = 0; i < 6; i++) {
    const double rowScale = axisLimit / result.axes[i].fullScale;
    for (int j = 0; j < 9; j++) {
      result.matrix[i][j] = M0[i][j] * rowScale;
    }
  }

  result.ok = true;
  return result;
}

}  // namespace calibration
