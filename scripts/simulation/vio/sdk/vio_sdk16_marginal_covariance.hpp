#pragma once
#include <Eigen/Dense>
#include <stdexcept>

// SDK inertial Hessian order: right IMU rotation, right IMU translation,
// world velocity, gyro bias, accelerometer bias. Output: world rotation,
// world rig position. These are measurement-point errors, not SE(3) Log coordinates.
namespace vio_covariance {
using Matrix6 = Eigen::Matrix<double,6,6>;
using Matrix15 = Eigen::Matrix<double,15,15>;
inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s; s << 0.,-v.z(),v.y(),v.z(),0.,-v.x(),-v.y(),v.x(),0.; return s;
}
template<int N> Eigen::Matrix<double,N,N> PositiveInverse(const Eigen::Matrix<double,N,N>& value) {
  if (!value.allFinite() || (value.diagonal().array()<=0.).any())
    throw std::runtime_error("invalid covariance information");
  // Equilibration changes units only. No damping, clipping or variance floor.
  const auto scale=value.diagonal().array().sqrt().inverse().matrix().eval();
  Eigen::Matrix<double,N,N> h=scale.asDiagonal()*((value+value.transpose())*.5)*scale.asDiagonal();
  Eigen::LDLT<Eigen::Matrix<double,N,N>> ldlt(h);
  if (ldlt.info()!=Eigen::Success || (ldlt.vectorD().array()<=0.).any())
    throw std::runtime_error("non-positive covariance information");
  Eigen::Matrix<double,N,N> inverse=scale.asDiagonal()*ldlt.solve(Eigen::Matrix<double,N,N>::Identity())*scale.asDiagonal();
  if (!inverse.allFinite()) throw std::runtime_error("non-finite covariance inverse");
  return ((inverse+inverse.transpose())*.5).eval();
}
inline Matrix6 ImuLocalToRigWorldJacobian(const Eigen::Matrix3d& world_from_imu,
                                        const Eigen::Vector3d& imu_to_rig) {
  Matrix6 j=Matrix6::Zero();
  j.topLeftCorner<3,3>()=j.bottomRightCorner<3,3>()=world_from_imu;
  j.bottomLeftCorner<3,3>()=-world_from_imu*Skew(imu_to_rig);
  return j;
}
inline Matrix6 MarginalRigWorld(const Matrix15& information, const Eigen::Matrix3d& world_from_imu,
                               const Eigen::Vector3d& imu_to_rig) {
  Matrix6 local=PositiveInverse<15>(information).topLeftCorner<6,6>();
  const Matrix6 j=ImuLocalToRigWorldJacobian(world_from_imu,imu_to_rig);
  return j*local*j.transpose();
}
inline Matrix6 WorldToEuler(const Matrix6& covariance, const Eigen::Matrix3d& rotation) {
  const double cp=std::hypot(rotation(0,0),rotation(1,0));
  if (!std::isfinite(cp) || cp<.5) throw std::runtime_error("native Euler singularity");
  const double cy=rotation(0,0)/cp,sy=rotation(1,0)/cp,sp=-rotation(2,0);
  Eigen::Matrix3d e; e << cy*cp,-sy,0.,sy*cp,cy,0.,-sp,0.,1.;
  Matrix6 j=Matrix6::Identity(); j.topLeftCorner<3,3>()=e.inverse();
  return j*covariance*j.transpose();
}
} // namespace vio_covariance
