#pragma once
#include "vio_sdk16_marginal_covariance.hpp"

namespace vio_covariance {
// Retain the SDK's pose CONDITIONED on nuisance states. Correct only its
// coordinates and rig reference point. This is NOT a full state posterior.
inline Matrix6 ConditionalRigWorld(const Matrix6& information,
    const Eigen::Matrix3d& world_from_imu, const Eigen::Vector3d& imu_to_rig) {
  const Matrix6 local = PositiveInverse<6>(information);
  const Matrix6 j = ImuLocalToRigWorldJacobian(world_from_imu, imu_to_rig);
  return j * local * j.transpose();
}
}
