// Regression: a zero-residual pose must retain its real measurement information.
#include <cassert>
#include <cmath>
#include "imu/soft_inertial_pnp.h"

using namespace cuvslam;
using namespace cuvslam::sba_imu;
using ImuPose=cuvslam::sba_imu::Pose;

ImuPose solve(float image_information) {
  imu::ImuCalibration calibration(Isometry3T::Identity(),.000245f,.000019f,.001862f,.003f,250.f);
  StereoPnPInput input;
  input.gravity=Vector3T(0.f,9.81f,0.f);
  input.rig.num_cameras=2;
  input.rig.camera_from_rig[0]=Isometry3T::Identity();
  input.rig.camera_from_rig[1]=Isometry3T::Identity();
  input.rig.camera_from_rig[1].translation().x()=-.075f;
  input.prior_acc=input.prior_gyro=input.translation_constraint=0.f;
  input.freeze_bias=true; input.robustifier_scale_pose=-1.f;
  for(int x=-4;x<=4;++x)for(int y=-3;y<=3;++y) {
    const int id=input.points.size();
    const Vector3T point(.2f*x,.2f*y,2.5f+.01f*x*y);
    input.points.push_back(point);
    for(int cam=0;cam<2;++cam) {
      const Vector3T p=input.rig.camera_from_rig[cam]*point;
      input.point_ids.push_back(id);input.camera_ids.push_back(cam);
      input.observation_xys.push_back(Vector2T(p.x()/p.z(),p.y()/p.z()));
      input.observation_infos.push_back(Matrix2T::Identity()*image_information);
    }
  }
  ImuPose previous,current;
  previous.info.setIdentity();current.info.setIdentity();
  for(int i=0;i<10;++i) {
    imu::ImuMeasurement m{1000000000LL+int64_t(i)*4000000LL,Vector3T(0.f,-9.81f,0.f),Vector3T::Zero()};
    previous.preintegration.IntegrateNewMeasurement(calibration,m);
    current.preintegration.IntegrateNewMeasurement(calibration,m);
  }
  assert(SoftInertialPnP(calibration,input,previous,current,1e-3f));
  assert(current.w_from_imu.translation().norm()<1e-8);
  assert(current.velocity.norm()<1e-8);
  assert(previous.w_from_imu.translation().norm()<1e-8);
  assert(current.info.allFinite());
  assert((current.info-Matrix15T::Identity()).norm()>1000.f);
  return current;
}
int main() {
  const ImuPose weak=solve(10000.f),strong=solve(40000.f);
  assert(strong.info(0,0)>weak.info(0,0));
  assert(strong.info(3,3)>weak.info(3,3));
}
