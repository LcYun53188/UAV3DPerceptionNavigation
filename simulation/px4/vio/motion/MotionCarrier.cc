// Independent force-driven sensor fixture. Never subscribes to VIO or PX4.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

namespace uav::test
{
class MotionCarrier final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  gz::sim::Link link;
  // Smooth position and derivatives, initially zero velocity and acceleration.
  static void Wave(double t, double amplitude, double frequency,
                   double &p, double &v, double &a)
  {
    const double s = std::sin(frequency*t), c = std::cos(frequency*t);
    p = amplitude*s*s*s;
    v = 3*amplitude*frequency*s*s*c;
    a = 3*amplitude*frequency*frequency*(2*s*c*c-s*s*s);
  }
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    link = gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm, "base_link"));
    if (!link.Valid(ecm)) throw std::runtime_error("Missing carrier base_link");
    link.EnableVelocityChecks(ecm);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) return;
    const auto pose = link.WorldPose(ecm);
    const auto vel = link.WorldLinearVelocity(ecm);
    const auto omega = link.WorldAngularVelocity(ecm);
    if (!pose || !vel || !omega) return;
    const double t = std::max(0., std::chrono::duration<double>(info.simTime).count()-15.);
    gz::math::Vector3d p, v, a;
    Wave(t,.50,.45,p.X(),v.X(),a.X());
    Wave(t,.40,.55,p.Y(),v.Y(),a.Y());
    Wave(t,.30,.65,p.Z(),v.Z(),a.Z());
    p.Z() += 1.3;
    double yaw, yawRate, yawAcc;
    Wave(t,.40,.40,yaw,yawRate,yawAcc);
    // Compound mass = 1 kg body + 0.001 kg rig. Gravity is upstream 9.8 m/s^2.
    auto force = 1.001*(a + 16*(p-pose->Pos()) + 8*(v-*vel) + gz::math::Vector3d(0,0,9.8));
    const auto desired = gz::math::Quaterniond(0,0,yaw);
    auto error = desired*pose->Rot().Inverse();
    if (error.W() < 0) error = gz::math::Quaterniond(-error.W(),-error.X(),-error.Y(),-error.Z());
    auto torque = .0201*(gz::math::Vector3d(0,0,yawAcc)
        + 64*gz::math::Vector3d(error.X(),error.Y(),error.Z())
        + 8*(gz::math::Vector3d(0,0,yawRate)-*omega));
    // Bounded fixture actuation, including when the simulator stalls.
    for (unsigned i=0;i<3;++i)
    {
      force[i] = std::clamp(force[i],-20.,20.);
      torque[i] = std::clamp(torque[i],-2.,2.);
    }
    link.AddWorldWrench(ecm,force,torque);
  }
};
}
GZ_ADD_PLUGIN(uav::test::MotionCarrier, gz::sim::System,
              uav::test::MotionCarrier::ISystemConfigure,
              uav::test::MotionCarrier::ISystemPreUpdate)
