// Test-only wrench on the actual PX4 x500. Never arms or subscribes to VIO.
// The one-shot start signal permits motion only after ground EV initialization.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>

namespace uav::test
{
class SameAircraftMotion final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  gz::sim::Link link;
  gz::transport::Node transport;
  std::atomic<bool> requested{false};
  bool started{false};
  double startTime{0}, mass{0}, attitudeAmplitude{0};
  gz::math::Vector3d origin;
  static double Smooth(double t)
  {
    t=std::clamp(t,0.,1.);
    return t*t*t*(10+t*(-15+6*t));
  }
  static gz::math::Vector3d Position(double t)
  {
    if (t<10) return {0,0,Smooth(t/10)};
    if (t>=90) return {0,0,1-Smooth((t-90)/10)};
    const double u=t-10, e=Smooth(u/5)*Smooth((80-u)/5);
    const auto wave=[](double x){return std::pow(std::sin(x),3);};
    return {.50*e*wave(2*M_PI*u/20),.40*e*wave(2*M_PI*u/25),
             1+.20*e*wave(2*M_PI*u/16)};
  }
  static double Yaw(double t)
  {
    if(t<10 || t>=90) return 0;
    const double u=t-10;
    return .35*Smooth(u/5)*Smooth((80-u)/5)*std::pow(std::sin(2*M_PI*u/20),3);
  }
  gz::math::Quaterniond Attitude(double t) const
  {
    if(t<10 || t>=90) return {0,0,0};
    const double u=t-10, e=Smooth(u/5)*Smooth((80-u)/5);
    return {attitudeAmplitude*e*std::pow(std::sin(2*M_PI*u/17),3),
            attitudeAmplitude*e*std::pow(std::sin(2*M_PI*u/23),3),Yaw(t)};
  }
  gz::math::Vector3d AngularVelocity(double t) const
  {
    constexpr double h=.001;
    auto delta=Attitude(t+h)*Attitude(t-h).Inverse();
    if(delta.W()<0) delta=gz::math::Quaterniond(-delta.W(),-delta.X(),-delta.Y(),-delta.Z());
    gz::math::Vector3d vector(delta.X(),delta.Y(),delta.Z());
    const double norm=vector.Length();
    return norm<1e-12 ? gz::math::Vector3d::Zero :
        vector*(std::atan2(norm,delta.W())/(h*norm));
  }
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    link=gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm,"base_link"));
    if(!link.Valid(ecm)) throw std::runtime_error("Missing actual x500 base_link");
    mass=sdf->Get<double>("compound_mass");
    if(!(mass>2 && mass<3)) throw std::runtime_error("Unreviewed fixture mass");
    attitudeAmplitude=sdf->HasElement("attitude_amplitude") ? sdf->Get<double>("attitude_amplitude") : 0.;
    if(!std::isfinite(attitudeAmplitude) || attitudeAmplitude<0 || attitudeAmplitude>.12)
      throw std::runtime_error("Unreviewed fixture attitude amplitude");
    link.EnableVelocityChecks(ecm);
    transport.Subscribe("/vio/fixture/start",&SameAircraftMotion::Start,this);
  }
  void Start(const gz::msgs::Boolean &m) { if(m.data()) requested.store(true); }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if(info.paused || !requested.load()) return;
    const auto pose=link.WorldPose(ecm);
    const auto vel=link.WorldLinearVelocity(ecm);
    const auto omega=link.WorldAngularVelocity(ecm);
    if(!pose || !vel || !omega) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if(!started){started=true;startTime=now;origin=pose->Pos();}
    const double t=now-startTime;
    if(t>=100) return; // Physical ground contact, no wrench for final-rest window.
    constexpr double h=.001;
    const auto p=Position(t), before=Position(t-h), after=Position(t+h);
    const auto v=(after-before)/(2*h), a=(after-2*p+before)/(h*h);
    const auto angularVelocity=AngularVelocity(t);
    const auto angularAcceleration=(AngularVelocity(t+h)-AngularVelocity(t-h))/(2*h);
    auto force=mass*(a+16*(origin+p-pose->Pos())+8*(v-*vel)+gz::math::Vector3d(0,0,9.8));
    auto error=Attitude(t)*pose->Rot().Inverse();
    if(error.W()<0) error=gz::math::Quaterniond(-error.W(),-error.X(),-error.Y(),-error.Z());
    auto torque=gz::math::Vector3d(.025,.025,.045)*
        (angularAcceleration+64*gz::math::Vector3d(error.X(),error.Y(),error.Z())+
         8*(angularVelocity-*omega));
    for(unsigned i=0;i<3;++i){force[i]=std::clamp(force[i],-40.,40.);torque[i]=std::clamp(torque[i],-2.,2.);}
    link.AddWorldWrench(ecm,force,torque);
  }
};
}
GZ_ADD_PLUGIN(uav::test::SameAircraftMotion,gz::sim::System,
    uav::test::SameAircraftMotion::ISystemConfigure,uav::test::SameAircraftMotion::ISystemPreUpdate)
