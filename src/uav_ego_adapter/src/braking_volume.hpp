// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <utility>

// Body/tracking reserve is three dimensional; braking reserve is horizontal.
// Keep the same conservative voxel box as the flight gateway's BrakingGrid.
inline std::pair<Eigen::Vector3i,Eigen::Vector3i> brakingBounds(
    const Eigen::Vector3d& point,const Eigen::Vector3d& origin,
    double radius,double horizontal,double resolution) {
  const Eigen::Vector3d extent(radius+horizontal,radius+horizontal,radius);
  return {((point-extent-origin)/resolution).array().floor().cast<int>(),
          ((point+extent-origin)/resolution).array().floor().cast<int>()};
}
