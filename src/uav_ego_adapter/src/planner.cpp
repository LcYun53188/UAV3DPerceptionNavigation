// SPDX-License-Identifier: GPL-3.0-only
#include <bspline_opt/bspline_optimizer.h>
#include <uav_nav_interfaces/msg/map_snapshot.hpp>
#include <uav_nav_interfaces/msg/timed_trajectory.hpp>
#include <uav_nav_interfaces/msg/planner_status.hpp>
#include <uav_nav_interfaces/msg/trajectory_request.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include "safe_seed.hpp"
#include "swept_segment.hpp"
#include "free_volume.hpp"

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
    continuous_ = declare_parameter("continuous_navigation", true);
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
        moving_.reset();
        pending_ = goal_.allFinite(); goal_active_=pending_;
      });
    moving_sub_ = create_subscription<uav_nav_interfaces::msg::TrajectoryRequest>("/uav/replan_request", 10,
      [this](uav_nav_interfaces::msg::TrajectoryRequest::ConstSharedPtr m) {
        if (!managed_) return;
        moving_ = m;
        goal_stamp_ = m->header.stamp;
        goal_ = V(m->goal.x,m->goal.y,m->goal.z);
        pending_ = true;
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
    if(!free_volume_.isFree(lo,hi)) return true;
    float d=s.distance[address(idx.x(),idx.y(),idx.z())];
    return !std::isfinite(d) || d<=r+std::sqrt(3.0)*s.resolution/2;
  }
  bool segment(const Snapshot& s,const V& a,const V& b,double r) {
    return !sweptSegmentFree(a,b,r,s.resolution,
        [this,&s](const V& p,double radius){return collision(s,p,radius);});
  }
  void plan() {
    if (!pending_ || (executing_ && !moving_) || !odom_ || !snapshot_) return;
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
    V start_velocity=V::Zero(), start_acceleration=V::Zero();
    if(moving_) {
      const auto& v=moving_->start_velocity; const auto& a=moving_->start_acceleration;
      start_velocity=V(v.x,v.y,v.z); start_acceleration=V(a.x,a.y,a.z);
      const double lead=(rclcpp::Time(moving_->start_time)-now()).seconds();
      if(moving_->header.frame_id!="map" || moving_->parent_trajectory_id==0 ||
         moving_->map_id!=s->map_id || moving_->epoch!=s->epoch || !goal_.allFinite() ||
         !start_velocity.allFinite() || !start_acceleration.allFinite() ||
         start_velocity.norm()>vmax_ || start_acceleration.norm()>amax_ || lead<0.1 || lead>2.0) {
        report("REJECT_HANDOVER_REQUEST"); return;
      }
    }
    free_volume_.update(s->shape,s->distance,s->observed);
    const auto& p=odom_->pose.pose.position;
    V start(p.x,p.y,p.z);
    if(moving_) { const auto& q=moving_->start_position; start=V(q.x,q.y,q.z); }
    if(collision(*s,start,radius_) || collision(*s,goal_,radius_)) { report("BLOCKED_START_OR_GOAL"); return; }
    if((start-goal_).norm()<0.15) { pending_=false; report("GOAL_REACHED"); return; }
    auto v=odom_->twist.twist.linear;
    if(!moving_ && V(v.x,v.y,v.z).norm()>0.05) { report("WAIT_STOPPED"); return; }
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
        // A coarse lattice can miss a corridor present in the voxel map.
        // Retry at map resolution before declaring it disconnected.
        if(!optimizer_.a_star_->AstarSearch(s->resolution*2,start,goal_) &&
           !optimizer_.a_star_->AstarSearch(s->resolution,start,goal_)) {report("NO_PATH");return;}
        seed=optimizer_.a_star_->getPath();
        if(seed.empty()) {report("NO_PATH");return;}
        // Preserve the checked endpoint-to-grid connectors. Replacing the
        // snapped endpoints would create unchecked shortcuts to their neighbors.
        seed.insert(seed.begin(),start); seed.push_back(goal_);
      }
      controls.resize(3,seed.size()+4);
      controls.col(0)=controls.col(1)=start;
      for(size_t i=0;i<seed.size();++i) controls.col(i+2)=seed[i];
      controls.col(controls.cols()-2)=controls.col(controls.cols()-1)=goal_;
      setStartState(controls,dt,start,start_velocity,start_acceleration);
      optimizer_.initControlPoints(controls,true);
      success=optimizer_.BsplineOptimizeTrajRebound(controls,dt);
    }
    auto valid_curve = [&]() {
      if(!controls.allFinite()) return false;
      dt=0.6;
      bool limited=false;
      for(int attempt=0;attempt<24;++attempt) {
        setStartState(controls,dt,start,start_velocity,start_acceleration);
        UniformBspline derivative(controls,3,dt);
        double scale=1.0;
        for(int d=1;d<=3;++d) {
          derivative=derivative.getDerivative();
          const double bound=derivative.getControlPoint().colwise().norm().maxCoeff();
          const double limit=d==1?vmax_:(d==2?amax_:jmax_);
          scale=std::max(scale,std::pow(bound/limit,1.0/d));
        }
        if(scale<=1.0000001) {limited=true;break;}
        dt*=scale*1.01;
        if(!std::isfinite(dt) || dt>30) return false;
      }
      if(!limited) return false;
      UniformBspline candidate(controls,3,dt);
      const double duration=candidate.getTimeSum();
      if(duration>600 || !std::isfinite(duration)) return false;
      const double bound=candidate.getDerivative().getControlPoint().colwise().norm().maxCoeff();
      int samples=std::max(1,static_cast<int>(std::ceil(duration*std::max(bound,0.01)/(s->resolution/2))));
      for(int i=0;i<=samples;++i)
        if(collision(*s,candidate.evaluateDeBoorT(duration*i/samples),radius_+bound*duration/samples/2)) return false;
      return true;
    };
    if(!success || !valid_curve()) {
      // Rebound smoothing can cut through an obstacle or unobserved volume.
      // Shortcut the checked A* route before trying a rounded route fallback.
      std::vector<V> route{start};
      for(size_t i=0;i+1<seed.size();) {
        size_t j=seed.size()-1;
        while(j>i+1 && segment(*s,seed[i],seed[j],seed_radius)) --j;
        if(segment(*s,seed[i],seed[j],seed_radius)) {report("NO_SAFE_SEED");return;}
        route.push_back(seed[j]);
        i=j;
      }
      // Try continuous corners, but accept them only after a full swept-body
      // check. A tight/unknown corner retains the checked stopping fallback.
      controls=safeSeedControls(route,continuous_);
      if(!valid_curve()) {
        controls=safeSeedControls(route);
        if(!valid_curve()) {report("REJECT_CURVE_COLLISION");return;}
      }
      report("SAFE_SEED_FALLBACK");
    }
    UniformBspline curve(controls,3,dt);
    const double duration=curve.getTimeSum();
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_start).count()>map_timeout_ ||
       (now()-rclcpp::Time(s->header.stamp)).seconds()>map_timeout_) {report("PLAN_EXPIRED");return;}
    Trajectory out;
    if(moving_ && (rclcpp::Time(moving_->start_time)-now()).seconds()<0.1) {
      report("HANDOVER_EXPIRED");return;
    }
    out.header.stamp=now();out.header.frame_id="odom";
    out.goal_stamp=goal_stamp_;
    out.map_id=s->map_id;out.epoch=s->epoch;out.map_version=s->version;out.trajectory_id=++id_;
    out.start_time=moving_ ? moving_->start_time : static_cast<builtin_interfaces::msg::Time>(now()+rclcpp::Duration::from_seconds(0.5));
    out.parent_trajectory_id=moving_ ? moving_->parent_trajectory_id : 0;
    out.knot_interval=dt;
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
  FreeVolume free_volume_;
  std::string last_status_;
  double radius_,vmax_,amax_,jmax_,map_timeout_;
  bool managed_=false,continuous_=true;
  builtin_interfaces::msg::Time goal_stamp_;
  uint64_t id_=0; bool pending_=false,executing_=false,goal_active_=false;V goal_;
  GridMap::Ptr map_; ego_planner::BsplineOptimizer optimizer_;ego_planner::SwarmTrajData swarm_;
  Snapshot::ConstSharedPtr snapshot_;nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  uav_nav_interfaces::msg::TrajectoryRequest::ConstSharedPtr moving_;
  std::chrono::steady_clock::time_point map_received_;
  rclcpp::Subscription<Snapshot>::SharedPtr snapshots_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<uav_nav_interfaces::msg::TrajectoryRequest>::SharedPtr moving_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_,event_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<uav_nav_interfaces::msg::PlannerStatus>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);auto node=std::make_shared<Planner>();node->init();rclcpp::spin(node);rclcpp::shutdown();}
