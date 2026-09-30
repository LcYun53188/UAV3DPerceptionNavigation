// SPDX-License-Identifier: GPL-3.0-only
#include <bspline_opt/bspline_optimizer.h>
#include <uav_nav_interfaces/msg/map_snapshot.hpp>
#include <uav_nav_interfaces/msg/timed_trajectory.hpp>
#include <uav_nav_interfaces/msg/planner_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using V = Eigen::Vector3d;
using Snapshot = uav_nav_interfaces::msg::MapSnapshot;
using Trajectory = uav_nav_interfaces::msg::TimedTrajectory;
using ego_planner::UniformBspline;

class Planner : public rclcpp::Node {
 public:
  Planner() : Node("ego_nvblox_planner") {}
  void init() {
    radius_ = declare_parameter("body_radius", 0.3);
    vmax_ = declare_parameter("max_velocity", 0.5);
    amax_ = declare_parameter("max_acceleration", 1.0);
    jmax_ = declare_parameter("max_jerk", 2.0);
    map_timeout_ = declare_parameter("map_timeout", 2.0);
    managed_ = declare_parameter("managed_goals", false);
    map_ = std::make_shared<GridMap>();
    optimizer_.setParam(shared_from_this());
    optimizer_.setEnvironment(map_);
    optimizer_.setDroneId(0);
    optimizer_.setSwarmTrajs(&swarm_);
    optimizer_.a_star_ = std::make_shared<AStar>();
    optimizer_.a_star_->initGridMap(map_, Eigen::Vector3i(120, 120, 60));
    snapshots_ = create_subscription<Snapshot>("/uav/map/snapshot", rclcpp::QoS(1).transient_local(),
      [this](Snapshot::ConstSharedPtr m) {
        if(snapshot_ && (m->epoch<snapshot_->epoch ||
           (m->epoch==snapshot_->epoch && snapshot_->valid && m->valid && m->version<snapshot_->version))) return;
        snapshot_ = m; map_received_ = std::chrono::steady_clock::now(); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/uav/localization/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/uav/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        if (m->header.frame_id != "map") { report("REJECT_GOAL_FRAME"); return; }
        goal_ = V(m->pose.position.x, m->pose.position.y, m->pose.position.z);
        goal_stamp_ = m->header.stamp;
        pending_ = goal_.allFinite(); goal_active_=pending_;
      });
    state_sub_ = create_subscription<std_msgs::msg::String>("/uav/executor/state", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) { executing_ = m->data == "EXECUTING"; });
    event_sub_ = create_subscription<std_msgs::msg::String>("/uav/executor/event",10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        if(managed_) return; // The task manager owns retries and completion.
        if(m->data=="GOAL_REACHED" || m->data=="CANCELLED" || m->data=="MAP_SESSION_CHANGED") {
          goal_active_=false; pending_=false;
        } else if(goal_active_ && (m->data.rfind("REJECTED:",0)==0 || m->data=="STALE_MAP_OR_ODOMETRY" ||
                   m->data.rfind("MAP_RECHECK_FAILED",0)==0 || m->data=="MAP_INVALID")) {
          pending_=true; // Replan from a stopped state with the next valid map.
        }
      });
    trajectory_pub_ = create_publisher<Trajectory>("/uav/trajectory", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/uav/planned_path", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/uav/planner/state", 10);
    result_pub_ = create_publisher<uav_nav_interfaces::msg::PlannerStatus>("/uav/planner/result", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { plan(); });
  }
 private:
  void report(const std::string& state) {
    if(state != last_status_) {RCLCPP_INFO(get_logger(), "%s", state.c_str()); last_status_=state;}
    std_msgs::msg::String m; m.data=state; status_pub_->publish(m);
    uav_nav_interfaces::msg::PlannerStatus result;
    result.goal_stamp=goal_stamp_; result.state=state; result_pub_->publish(result);
    if(managed_ && state!="WAIT_MAP_OR_ODOMETRY" && state!="WAIT_STOPPED" &&
       state!="SAFE_SEED_FALLBACK") pending_=false;
  }
  bool collision(const Snapshot& s, const V& p, double r) {
    if (!p.allFinite()) return true;
    V origin(s.origin.x,s.origin.y,s.origin.z);
    Eigen::Vector3i lo=((p-V::Constant(r)-origin)/s.resolution).array().floor().cast<int>();
    Eigen::Vector3i hi=((p+V::Constant(r)-origin)/s.resolution).array().floor().cast<int>();
    Eigen::Vector3i idx=((p-origin)/s.resolution).array().floor().cast<int>();
    for (int a=0;a<3;++a) if(lo[a]<0 || hi[a]>=static_cast<int>(s.shape[a])) return true;
    auto address=[&s](int x,int y,int z) { return (static_cast<size_t>(x)*s.shape[1]+y)*s.shape[2]+z; };
    auto sum=[&](int x,int y,int z) -> int64_t {
      return prefix_[(static_cast<size_t>(x)*(s.shape[1]+1)+y)*(s.shape[2]+1)+z];
    };
    Eigen::Vector3i h=hi+Eigen::Vector3i::Ones();
    int64_t observed=sum(h.x(),h.y(),h.z())-sum(lo.x(),h.y(),h.z())-sum(h.x(),lo.y(),h.z())-sum(h.x(),h.y(),lo.z())
      +sum(lo.x(),lo.y(),h.z())+sum(lo.x(),h.y(),lo.z())+sum(h.x(),lo.y(),lo.z())-sum(lo.x(),lo.y(),lo.z());
    if(observed != static_cast<int64_t>((h-lo).prod())) return true;
    float d=s.distance[address(idx.x(),idx.y(),idx.z())];
    return !std::isfinite(d) || d<=r+std::sqrt(3.0)*s.resolution/2;
  }
  bool segment(const Snapshot& s,const V& a,const V& b,double r) {
    const double length=(b-a).norm();
    int n=std::max(1,static_cast<int>(std::ceil(length/(s.resolution*0.5))));
    for(int i=0;i<=n;++i) if(collision(s,a+(b-a)*(double(i)/n),r+length/n/2)) return true;
    return false;
  }
  void plan() {
    if (!pending_ || executing_ || !odom_ || !snapshot_) return;
    auto s=snapshot_; // Single-thread executor: frozen through the whole optimization.
    const double age=(now()-rclcpp::Time(s->header.stamp)).seconds();
    const double odom_age=(now()-rclcpp::Time(odom_->header.stamp)).seconds();
    const size_t n=static_cast<size_t>(s->shape[0])*s->shape[1]*s->shape[2];
    if (!s->valid || s->header.frame_id!="map" || n==0 || n>4000000 ||
        s->distance.size()!=n || s->observed.size()!=n || !V(s->origin.x,s->origin.y,s->origin.z).allFinite() || !std::isfinite(s->resolution) || s->resolution<=0 ||
        age<0 || age>map_timeout_ || odom_age<0 || odom_age>0.5 ||
        std::chrono::duration<double>(std::chrono::steady_clock::now()-map_received_).count()>map_timeout_) {
      report("WAIT_MAP_OR_ODOMETRY"); return;
    }
    if(odom_->header.frame_id!="odom") { report("REJECT_ODOM_FRAME"); return; }
    // Integral observed-volume mask makes conservative body queries O(1).
    prefix_.assign(static_cast<size_t>(s->shape[0]+1)*(s->shape[1]+1)*(s->shape[2]+1),0);
    auto at=[&](int x,int y,int z) -> uint32_t& {
      return prefix_[(static_cast<size_t>(x)*(s->shape[1]+1)+y)*(s->shape[2]+1)+z];
    };
    for(uint32_t x=1;x<=s->shape[0];++x) for(uint32_t y=1;y<=s->shape[1];++y) for(uint32_t z=1;z<=s->shape[2];++z)
      at(x,y,z)=s->observed[((x-1)*s->shape[1]+y-1)*s->shape[2]+z-1]+at(x-1,y,z)+at(x,y-1,z)+at(x,y,z-1)
        -at(x-1,y-1,z)-at(x-1,y,z-1)-at(x,y-1,z-1)+at(x-1,y-1,z-1);
    const auto& p=odom_->pose.pose.position;
    V start(p.x,p.y,p.z);
    if(collision(*s,start,radius_) || collision(*s,goal_,radius_)) { report("BLOCKED_START_OR_GOAL"); return; }
    if((start-goal_).norm()<0.15) { pending_=false; report("GOAL_REACHED"); return; }
    // Simulation adapter requires a stopped takeover. It does not splice moving trajectories.
    auto v=odom_->twist.twist.linear;
    if(V(v.x,v.y,v.z).norm()>0.05) { report("WAIT_STOPPED"); return; }
    map_->resolution=s->resolution;
    // Reserve room for the swept-curve validator's sampling padding.
    const double seed_radius=radius_+s->resolution*0.5;
    map_->collision=[s,this,seed_radius](const V& q){return collision(*s,q,seed_radius);};
    map_->segment_collision=[s,this,seed_radius](const V&a,const V&b){return segment(*s,a,b,seed_radius);};
    optimizer_.setLocalTargetPt(goal_);
    const auto wall_start=std::chrono::steady_clock::now();
    Eigen::MatrixXd controls;
    bool success=false;
    double dt=0.6;
    std::vector<V> seed;
    // Use EGO's own A* to initialize obstacle detours before rebound fitting.
    // This avoids spending the entire snapshot lifetime on repeated straight-line
    // rebound searches when a large obstacle separates the endpoints.
    {
      if(!segment(*s,start,goal_,seed_radius)) {
        int count=std::max(5,static_cast<int>(std::ceil((goal_-start).norm()/0.25)));
        for(int i=0;i<=count;++i) seed.push_back(start+(goal_-start)*double(i)/count);
      } else {
        if(!optimizer_.a_star_->AstarSearch(s->resolution*2,start,goal_)) {report("NO_PATH");return;}
        seed=optimizer_.a_star_->getPath();
        if(seed.size()<3) {report("NO_PATH");return;}
        seed.front()=start; seed.back()=goal_;
      }
      controls.resize(3,seed.size()+4);
      controls.col(0)=controls.col(1)=start;
      for(size_t i=0;i<seed.size();++i) controls.col(i+2)=seed[i];
      controls.col(controls.cols()-2)=controls.col(controls.cols()-1)=goal_;
      optimizer_.initControlPoints(controls,true);
      success=optimizer_.BsplineOptimizeTrajRebound(controls,dt);
    }
    auto valid_curve = [&]() {
      if(!controls.allFinite()) return false;
      dt=0.6;
      UniformBspline candidate(controls,3,dt);
      double scale=1.0;
      auto derivative=candidate;
      for(int d=1;d<=3;++d) {
        derivative=derivative.getDerivative();
        const double bound=derivative.getControlPoint().colwise().norm().maxCoeff();
        const double limit=d==1?vmax_:(d==2?amax_:jmax_);
        scale=std::max(scale,std::pow(bound/limit,1.0/d));
      }
      dt*=scale*1.01;
      candidate=UniformBspline(controls,3,dt);
      const double duration=candidate.getTimeSum();
      if(duration>600 || !std::isfinite(duration)) return false;
      const double bound=candidate.getDerivative().getControlPoint().colwise().norm().maxCoeff();
      int samples=std::max(1,static_cast<int>(std::ceil(duration*std::max(bound,0.01)/(s->resolution/2))));
      for(int i=0;i<=samples;++i)
        if(collision(*s,candidate.evaluateDeBoorT(duration*i/samples),radius_+bound*duration/samples/2)) return false;
      return true;
    };
    if(!success || !valid_curve()) {
      // Rebound smoothing can cut through a corner or unobserved volume even
      // when A* found a valid route. Keep that route and stop at its corners.
      // Three identical controls at each waypoint give zero velocity and
      // acceleration there; each connecting span stays on its checked segment.
      std::vector<V> route{start};
      for(size_t i=0;i+1<seed.size();) {
        size_t j=seed.size()-1;
        while(j>i+1 && segment(*s,seed[i],seed[j],seed_radius)) --j;
        if(segment(*s,seed[i],seed[j],seed_radius)) {report("NO_SAFE_SEED");return;}
        const V a=route.back(), b=seed[j];
        const int pieces=std::max(1,static_cast<int>(std::ceil((b-a).norm())));
        for(int k=1;k<=pieces;++k) route.push_back(a+(b-a)*(double(k)/pieces));
        i=j;
      }
      if(route.size()==2) route.insert(route.begin()+1,(start+goal_)/2);
      controls.resize(3,route.size()*3);
      for(size_t i=0;i<route.size();++i)
        for(int k=0;k<3;++k) controls.col(i*3+k)=route[i];
      if(!valid_curve()) {report("REJECT_CURVE_COLLISION");return;}
      report("SAFE_SEED_FALLBACK");
    }
    UniformBspline curve(controls,3,dt);
    const double duration=curve.getTimeSum();
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_start).count()>map_timeout_ ||
       (now()-rclcpp::Time(s->header.stamp)).seconds()>map_timeout_) {report("PLAN_EXPIRED");return;}
    Trajectory out;
    out.header.stamp=now();out.header.frame_id="odom";
    out.goal_stamp=goal_stamp_;
    out.map_id=s->map_id;out.epoch=s->epoch;out.map_version=s->version;out.trajectory_id=++id_;
    out.start_time=now()+rclcpp::Duration::from_seconds(0.5);out.knot_interval=dt;
    out.max_velocity=vmax_;out.max_acceleration=amax_;out.max_jerk=jmax_;
    for(int i=0;i<controls.cols();++i) { geometry_msgs::msg::Point pt;pt.x=controls(0,i);pt.y=controls(1,i);pt.z=controls(2,i);out.control_points.push_back(pt); }
    trajectory_pub_->publish(out);
    nav_msgs::msg::Path path;path.header=out.header;
    for(double t=0;t<=duration;t+=0.1) {
      V q=curve.evaluateDeBoorT(t);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=q.x();pose.pose.position.y=q.y();pose.pose.position.z=q.z();pose.pose.orientation.w=1;path.poses.push_back(pose);
    }
    path_pub_->publish(path);pending_=false;report("TRAJECTORY_PUBLISHED");
  }
  std::vector<uint32_t> prefix_;
  std::string last_status_;
  double radius_,vmax_,amax_,jmax_,map_timeout_;
  bool managed_=false;
  builtin_interfaces::msg::Time goal_stamp_;
  uint64_t id_=0; bool pending_=false,executing_=false,goal_active_=false;V goal_;
  GridMap::Ptr map_; ego_planner::BsplineOptimizer optimizer_;ego_planner::SwarmTrajData swarm_;
  Snapshot::ConstSharedPtr snapshot_;nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::chrono::steady_clock::time_point map_received_;
  rclcpp::Subscription<Snapshot>::SharedPtr snapshots_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_,event_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<uav_nav_interfaces::msg::PlannerStatus>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);auto node=std::make_shared<Planner>();node->init();rclcpp::spin(node);rclcpp::shutdown();}
