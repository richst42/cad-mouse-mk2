#pragma once

#include <string>
#include <vector>

#include "LinAlg.h"

// Guided-sweep calibration solver.
//
// The user cannot produce pure single-axis motions on this device, and does
// not need to. Each guided gesture only has to excite a known 1- or 2-D
// subspace of the 9-D sensor-delta space:
//
//   - hold gestures give a direction (mean delta while held at full
//     deflection) and a full-scale amplitude,
//   - circle sweeps trace out a plane; PCA yields the two dominant
//     directions, the paired hold direction anchors the first axis inside
//     that plane, and the announced travel direction fixes the sign of the
//     second.
//
// The six unit directions become the columns of a 9x6 Jacobian J; the
// device matrix is its pseudo-inverse with rows scaled so a full deflection
// produces `axisLimit` counts. The pseudo-inverse (rather than per-axis
// projection) is what cancels the inter-axis coupling: it accounts for the
// directions not being orthogonal, which is exactly the gesture
// contamination (a twist that also translates, an off-center tilt circle).
namespace calibration {

using linalg::Vec9;

// Deltas are raw sensor samples minus the rest mean, captured by the wizard.
struct CaptureSet {
  std::vector<Vec9> rest;        // raw samples, hands off
  std::vector<Vec9> pressHold;   // held: pressed straight down       -> Tz
  std::vector<Vec9> twistHold;   // held: twisted clockwise (top view)-> Rz
  std::vector<Vec9> slideHold;   // held: slid toward the right       -> Tx
  std::vector<Vec9> slideSweep;  // slow horizontal circle, no tilt   -> Ty
  std::vector<Vec9> rimHold;     // held: rim pressed at the far edge -> Rx
  std::vector<Vec9> rimSweep;    // slow rim-press circle             -> Ry
};

struct AxisSolve {
  Vec9 direction;    // unit direction in sensor-delta space
  double fullScale;  // delta magnitude at full deflection
};

struct SolveResult {
  bool ok = false;
  std::string error;

  // Axis order: Tx Ty Tz Rx Ry Rz.
  AxisSolve axes[6];
  double matrix[6][9];  // upload-ready device matrix

  // Diagnostics.
  double restNoise = 0.0;        // RMS of rest samples about their mean
  double slidePlaneRatio = 0.0;  // lambda2 / lambda3 of the slide sweep
  double rimPlaneRatio = 0.0;    // lambda2 / lambda3 of the rim sweep
  double conditioning = 0.0;     // max/min singular value proxy of J
};

// axisLimit is the device's full-scale output (Config::AXIS_LIMIT, 350).
SolveResult solve(const CaptureSet& captures, double axisLimit);

}  // namespace calibration
