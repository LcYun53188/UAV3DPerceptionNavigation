// SDK16.0.0 source returns [world Euler ZYX roll,pitch,yaw, world position].
// Convert to [ROS world position, world-fixed infinitesimal rotation], retaining
// all cross blocks. This is a coordinate Jacobian, not uncertainty rescaling.
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
inline Eigen::Matrix<double,6,6> Sdk16FixedCovariance(
  const Eigen::Matrix3d & native_rotation, const Eigen::Matrix<double,6,6> & native_covariance,
  const Eigen::Matrix3d & cv_to_ros)
{
  const double cp = std::hypot(native_rotation(0,0), native_rotation(1,0));
  if (cp < .5) throw std::runtime_error("VIO_SDK_EULER_SINGULARITY");
  const double yaw = std::atan2(native_rotation(1,0),native_rotation(0,0));
  const double sp = -native_rotation(2,0), cy=std::cos(yaw), sy=std::sin(yaw);
  // d(world small rotation)/d(native ZYX Euler angles).
  Eigen::Matrix3d euler_to_fixed;
  euler_to_fixed << cy*cp, -sy, 0, sy*cp, cy, 0, -sp, 0, 1;
  Eigen::Matrix<double,6,6> j=Eigen::Matrix<double,6,6>::Zero();
  j.block<3,3>(0,3)=cv_to_ros;
  j.block<3,3>(3,0)=cv_to_ros*euler_to_fixed;
  return j*((native_covariance+native_covariance.transpose())*.5)*j.transpose();
}
