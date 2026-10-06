// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

// Each accepted midpoint box covers a whole swept interval. Refinement
// reduces sampling inflation, never the body radius or unknown-space guard.
template<class Collision>
bool sweptSegmentFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                      double radius, double resolution, Collision collision) {
  if (!a.allFinite() || !b.allFinite() || !std::isfinite(radius) || radius<0 ||
      !std::isfinite(resolution) || resolution<=0 ||
      collision(a,radius) || collision(b,radius)) return false;
  const double pieces=std::ceil((b-a).norm()/(resolution/2));
  if (!std::isfinite(pieces) || pieces>4094) return false;
  const int count=std::max(1,int(pieces));
  int checks=2;
  struct Interval { Eigen::Vector3d a,b; int depth; };
  for (int i=0;i<count;++i) {
    std::vector<Interval> stack{{a+(b-a)*(double(i)/count),
                               a+(b-a)*(double(i+1)/count),0}};
    while (!stack.empty()) {
      const auto interval=stack.back(); stack.pop_back();
      const Eigen::Vector3d middle=(interval.a+interval.b)/2;
      if (checks>=4096) return false;
      ++checks;
      if (!collision(middle,radius+(interval.b-interval.a).norm()/2)) continue;
      if (interval.depth>=6 || checks>=4096) return false;
      ++checks;
      if (collision(middle,radius)) return false;
      stack.push_back({middle,interval.b,interval.depth+1});
      stack.push_back({interval.a,middle,interval.depth+1});
    }
  }
  return true;
}
