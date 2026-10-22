#pragma once
#include <Eigen/Dense>
// OpenVINS JPL error: IMU-local orientation, world position. ROS needs world
// position, world fixed-axis orientation at the physical base point.
inline Eigen::Matrix<double,6,6> OpenVinsBaseCovariance(
    const Eigen::Matrix<double,6,6>& imu_covariance, const Eigen::Matrix3d& world_from_imu,
    const Eigen::Vector3d& imu_to_base) {
  Eigen::Matrix3d s;s << 0.,-imu_to_base.z(),imu_to_base.y(),imu_to_base.z(),0.,-imu_to_base.x(),-imu_to_base.y(),imu_to_base.x(),0.;
  Eigen::Matrix<double,6,6> j=Eigen::Matrix<double,6,6>::Zero();
  j.topLeftCorner<3,3>()=-world_from_imu*s;j.topRightCorner<3,3>().setIdentity();
  j.bottomLeftCorner<3,3>()=world_from_imu;
  return j*((imu_covariance+imu_covariance.transpose())*.5)*j.transpose();
}
