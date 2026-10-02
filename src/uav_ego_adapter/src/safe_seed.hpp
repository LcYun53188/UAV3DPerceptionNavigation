// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

// Route edges have already passed swept-body segment checks. The default
// repeats corner controls, keeping each span on a checked edge. Smooth mode
// rounds corners and requires an independent swept-curve collision check.
// Single interior controls allow uninterrupted travel along each edge.
inline Eigen::MatrixXd safeSeedControls(const std::vector<Eigen::Vector3d>& route,
                                       bool smooth_corners = false) {
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < route.size(); ++i) {
    if (i > 0) {
      const auto delta = (route[i] - route[i-1]).eval();
      const int pieces = std::max(2, static_cast<int>(std::ceil(delta.norm()/0.5)));
      for (int k = 1; k < pieces; ++k)
        points.push_back(route[i-1] + delta*(double(k)/pieces));
    }
    const int repeats = smooth_corners && i > 0 && i+1 < route.size() ? 1 : 3;
    for (int k = 0; k < repeats; ++k) points.push_back(route[i]);
  }
  Eigen::MatrixXd controls(3, points.size());
  for (size_t i = 0; i < points.size(); ++i) controls.col(i) = points[i];
  return controls;
}

// Uniform cubic endpoint equations. Reapply after every knot-interval change:
// retiming fixed controls would silently change the requested velocity/acceleration.
inline void setStartState(Eigen::MatrixXd& controls, double dt,
                          const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                          const Eigen::Vector3d& a) {
  controls.col(0) = p-v*dt+a*(dt*dt/3.0);
  controls.col(1) = p-a*(dt*dt/6.0);
  controls.col(2) = p+v*dt+a*(dt*dt/3.0);
}
