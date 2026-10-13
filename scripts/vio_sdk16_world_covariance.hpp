#pragma once
#include <Eigen/Dense>
#include <stdexcept>

// SDK16 conditional-v2's patched native exporter is WORLD SMALL ROTATION,
// WORLD RIG POSITION in OpenCV axes. No Euler angles or absolute pose Jacobian.
inline Eigen::Matrix<double,6,6> Sdk16WorldFixedCovariance(
    const Eigen::Matrix<double,6,6>& native, const Eigen::Matrix3d& cv_to_ros) {
  Eigen::Matrix<double,6,6> j=Eigen::Matrix<double,6,6>::Zero();
  j.topRightCorner<3,3>()=cv_to_ros;
  j.bottomLeftCorner<3,3>()=cv_to_ros;
  return j*((native+native.transpose())*.5)*j.transpose();
}
