// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

// Route edges have already passed swept-body segment checks. Repeat controls
// only at route corners: each spline span stays on one checked edge, while
// single interior controls allow uninterrupted travel along that edge.
inline Eigen::MatrixXd safeSeedControls(const std::vector<Eigen::Vector3d>& route) {
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < route.size(); ++i) {
    if (i > 0) {
      const auto delta = (route[i] - route[i-1]).eval();
      const int pieces = std::max(2, static_cast<int>(std::ceil(delta.norm()/0.5)));
      for (int k = 1; k < pieces; ++k)
        points.push_back(route[i-1] + delta*(double(k)/pieces));
    }
    for (int k = 0; k < 3; ++k) points.push_back(route[i]);
  }
  Eigen::MatrixXd controls(3, points.size());
  for (size_t i = 0; i < points.size(); ++i) controls.col(i) = points[i];
  return controls;
}
