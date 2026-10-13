// Native OpenVINS pixels + IMU estimator. Publishes raw posterior pose/state only.
// Source normalization, launch alignment, and PX4 admission are separate consumers.
#include <core/VioManager.h>
#include <track/TrackBase.h>
#include <feat/FeatureDatabase.h>
#include <feat/Feature.h>
#include <state/State.h>
#include <state/StateHelper.h>
#include <update/UpdaterZeroVelocity.h>
#include <utils/opencv_yaml_parse.h>
#include <utils/print.h>
#include <utils/sensor_data.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <deque>
#include <map>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <Eigen/Geometry>
#include "openvins_pose_covariance.hpp"
using Json=nlohmann::json;
class DiagnosticManager:public ov_msckf::VioManager {
public:
  using VioManager::VioManager;
  // Public initialized() also requires a visual update. At rest, accepted ZUPT
  // updates return before that flag is set. Export the initialized inertial
  // posterior, and require its actual timestamp and covariance below.
  bool inertial_ready() const { return is_initialized_vio && thread_init_success; }
  void feed_buffered_imu(const ov_core::ImuData& sample) {
    VioManager::feed_measurement_imu(sample);
    // Native beginning-only mode otherwise stops buffering this updater after
    // movement. Retain its bounded IMU window so a later observed rest can be
    // checked without restarting or resetting the estimator.
    if(is_initialized_vio && updaterZUPT && params.zupt_only_at_beginning && has_moved_since_zupt)
      updaterZUPT->feed_imu(sample,state->margtimestep());
  }
  void stationary_pixels(bool quiet) {
    // Strict simulation-only eligibility; native IMU innovation and velocity
    // checks still decide whether a zero-motion update is actually accepted.
    if(is_initialized_vio)has_moved_since_zupt=!quiet;
  }
  void frontend_diagnostic(double time){
    auto features=trackFEATS->get_feature_database()->get_internal_data();
    size_t measurements=0,old=0;double maximum_age=0.;
    for(const auto& f:features)for(const auto& ts:f.second->timestamps){
      measurements+=ts.second.size();
      if(!ts.second.empty()){maximum_age=std::max(maximum_age,time-ts.second.front());if(ts.second.front()<time-1.)++old;}
    }
    std::cerr<<"FRONTEND time="<<time<<" features="<<features.size()<<" measurements="<<measurements
      <<" maximum_age="<<maximum_age<<" older_than_1s="<<old<<'\n';
  }
};
class OpenVinsNode:public rclcpp::Node {
  using Image=sensor_msgs::msg::Image;
  std::shared_ptr<DiagnosticManager> app;
  Eigen::Matrix3d imu_from_base;Eigen::Vector3d imu_to_base;
  std::array<Image::ConstSharedPtr,2> images;
  std::array<std::map<int64_t,Image::ConstSharedPtr>,2> image_queues;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue;
  std::mutex queue_mutex;std::condition_variable queue_event;std::thread worker;
  std::atomic<bool> stopping{false},failed{false};
  int64_t last_imu=-1,last_image=-1;
  size_t processed_frames=0;
  std::array<cv::Mat,2> previous_pixels;
  int64_t last_visual_change=-1;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::array<rclcpp::Subscription<Image>::SharedPtr,2> image_subs;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poses;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr states;
  static int64_t ns(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
  cv::Mat gray(const Image& m){
    if(m.encoding=="mono8")return cv::Mat(m.height,m.width,CV_8UC1,const_cast<unsigned char*>(m.data.data()),m.step).clone();
    if(m.encoding!="rgb8")throw std::runtime_error("Unreviewed stereo encoding");
    cv::Mat rgb(m.height,m.width,CV_8UC3,const_cast<unsigned char*>(m.data.data()),m.step),out;
    cv::cvtColor(rgb,out,cv::COLOR_RGB2GRAY);return out;
  }
  void estimate(){
    try{while(!stopping && rclcpp::ok()){
      std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> measurements;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_event.wait_for(lock,std::chrono::milliseconds(10));
        if(stopping)break;
        while(!image_queues[0].empty()&&!image_queues[1].empty()){
          auto l=image_queues[0].begin()->first,r=image_queues[1].begin()->first;
          if(l==r)break;image_queues[l<r?0:1].erase(image_queues[l<r?0:1].begin());
        }
        if(image_queues[0].empty()||image_queues[1].empty()||imu_queue.empty())continue;
        auto t=image_queues[0].begin()->first;
        if(ns(imu_queue.back()->header.stamp)<=t)continue;
        for(int i=0;i<2;++i){images[i]=image_queues[i].begin()->second;image_queues[i].erase(image_queues[i].begin());}
        while(!imu_queue.empty()){
          auto m=imu_queue.front();imu_queue.pop_front();measurements.push_back(m);
          if(ns(m->header.stamp)>t)break;
        }
      }
      for(const auto& m:measurements){
        ov_core::ImuData v;v.timestamp=ns(m->header.stamp)/1e9;
        v.wm << m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z;
        v.am << m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z;
        app->feed_buffered_imu(v);
      }
      const auto begin=std::chrono::steady_clock::now();
      drain();
      const double compute_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      if(std::getenv("OPENVINS_FEATURE_DIAGNOSTIC") && processed_frames%25==0)app->frontend_diagnostic(last_image/1e9);
      if(++processed_frames%100==0)RCLCPP_INFO(get_logger(),"processed_frames=%zu original_image_s=%.3f compute_ms=%.3f inertial_ready=%d visual_ready=%d",processed_frames,last_image/1e9,compute_ms,app->inertial_ready(),app->initialized());
    }}catch(const std::exception& e){failed=true;RCLCPP_ERROR(get_logger(),"Estimator retired: %s",e.what());rclcpp::shutdown();}
  }
  void drain(){
    if(!images[0]||!images[1])return;
    auto t=ns(images[0]->header.stamp),other=ns(images[1]->header.stamp);
    if(t!=other){images[t<other?0:1].reset();return;}
    // The worker has delivered the following IMU endpoint before this frame.
    if(t<=last_image)throw std::runtime_error("VIO image timestamp reset");
    ov_core::CameraData camera;camera.timestamp=t/1e9;camera.sensor_ids={0,1};
    bool identical=true;
    for(int i=0;i<2;++i){
      camera.images.push_back(gray(*images[i]));camera.masks.push_back(cv::Mat::zeros(camera.images.back().size(),CV_8UC1));
      identical &= !previous_pixels[i].empty() && cv::norm(previous_pixels[i],camera.images.back(),cv::NORM_INF)==0.;
      previous_pixels[i]=camera.images.back();
    }
    if(!identical || last_visual_change<0)last_visual_change=t;
    app->stationary_pixels(t-last_visual_change>=2000000000LL);
    auto stamp=images[0]->header.stamp;images={};last_image=t;
    app->feed_measurement_camera(camera);
    std_msgs::msg::Float64MultiArray health;health.data={double(t),0.};
    if(app->inertial_ready()){
      auto state=app->get_state();
      if(std::abs(state->_timestamp-camera.timestamp)>1e-7)throw std::runtime_error("Stale OpenVINS posterior");
      health.data={double(t),1.,0.,0.,-9.8};
      for(const auto& v:{state->_imu->vel(),state->_imu->bias_g(),state->_imu->bias_a()})
        for(int i=0;i<3;++i)health.data.push_back(v(i));
      const Eigen::Matrix3d r=state->_imu->Rot().transpose();
      const Eigen::Vector3d p=state->_imu->pos()+r*imu_to_base;Eigen::Quaterniond q(r*imu_from_base);q.normalize();
      std::vector<std::shared_ptr<ov_type::Type>> order{state->_imu->pose()->q(),state->_imu->pose()->p()};
      Eigen::Matrix<double,6,6> marginal=ov_msckf::StateHelper::get_marginal_covariance(state,order);
      const auto c=OpenVinsBaseCovariance(marginal,r,imu_to_base);
      if(!p.allFinite() || !q.coeffs().allFinite() || !c.allFinite() ||
          !state->_imu->vel().allFinite() || !state->_imu->bias_g().allFinite() || !state->_imu->bias_a().allFinite())
        throw std::runtime_error("Nonfinite OpenVINS posterior");
      geometry_msgs::msg::PoseWithCovarianceStamped m;m.header.stamp=stamp;m.header.frame_id="odom";
      m.pose.pose.position.x=p.x();m.pose.pose.position.y=p.y();m.pose.pose.position.z=p.z();
      m.pose.pose.orientation.x=q.x();m.pose.pose.orientation.y=q.y();m.pose.pose.orientation.z=q.z();m.pose.pose.orientation.w=q.w();
      for(int i=0;i<6;++i)for(int j=0;j<6;++j)m.pose.covariance[i*6+j]=c(i,j);
      states->publish(health);poses->publish(m);
      nav_msgs::msg::Odometry o;o.header=m.header;o.child_frame_id="base_link";o.pose=m.pose;
      // This interface does not export a base-point twist uncertainty contract.
      const double unknown=std::numeric_limits<double>::quiet_NaN();
      o.twist.twist.linear.x=o.twist.twist.linear.y=o.twist.twist.linear.z=unknown;
      o.twist.twist.angular.x=o.twist.twist.angular.y=o.twist.twist.angular.z=unknown;o.twist.covariance.fill(unknown);
      odometry->publish(o);
    }else states->publish(health);
  }
public:
  explicit OpenVinsNode(const std::string& directory):Node("openvins"){
    auto parser=std::make_shared<ov_core::YamlParser>(directory+"/estimator.yaml");
    ov_core::Printer::setPrintLevel("WARNING");ov_msckf::VioManagerOptions params;params.print_and_load(parser);
    params.use_multi_threading_subs=false;params.use_multi_threading_pubs=false;
    if(!parser->successful())throw std::runtime_error("Invalid OpenVINS configuration");
    app=std::make_shared<DiagnosticManager>(params);
    declare_parameter("source_commit","69488123ed9362dd44b6f28e7f4680abbff1442b");
    declare_parameter("calibration_id","");declare_parameter("use_fej",params.state_options.do_fej);
    declare_parameter("num_cameras",params.state_options.num_cameras);
    declare_parameter("zero_motion_detection","identical_stereo_pixels_2s_plus_native_imu_checks_disarmed_simulation");
    std::ifstream file(directory+"/imu_from_base.json");auto tf=Json::parse(file);
    for(int i=0;i<3;++i){imu_to_base(i)=tf[i][3].get<double>();for(int j=0;j<3;++j)imu_from_base(i,j)=tf[i][j].get<double>();}
    poses=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/openvins/pose",10);
    states=create_publisher<std_msgs::msg::Float64MultiArray>("/openvins/state",10);
    odometry=create_publisher<nav_msgs::msg::Odometry>("/openvins/odometry",10);
    imu_sub=create_subscription<sensor_msgs::msg::Imu>("/vio/imu",rclcpp::SensorDataQoS().keep_last(1000),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(queue_mutex);auto t=ns(m->header.stamp);
        if(t<=last_imu || m->header.frame_id!="vio_imu")throw std::runtime_error("VIO IMU identity/timestamp invalid");
        if(imu_queue.size()>=1000)throw std::runtime_error("VIO IMU queue overflow");
        last_imu=t;imu_queue.push_back(m);queue_event.notify_one();});
    for(int i=0;i<2;++i)image_subs[i]=create_subscription<Image>(i==0?"/vio/left/image":"/vio/right/image",
      rclcpp::QoS(8).reliable(),[this,i](Image::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(queue_mutex);auto t=ns(m->header.stamp);
        auto& queue=image_queues[i];
        if(queue.size()>=4)throw std::runtime_error("VIO image queue overflow");
        if(!queue.empty()&&t<=queue.rbegin()->first)throw std::runtime_error("VIO image timestamp invalid");
        queue.emplace(t,m);queue_event.notify_one();});
    worker=std::thread([this]{estimate();});
  }
  ~OpenVinsNode(){stopping=true;queue_event.notify_all();if(worker.joinable())worker.join();}
  bool failure()const{return failed;}
};
int main(int argc,char** argv){try{if(argc<2)throw std::runtime_error("OpenVINS configuration directory required");
rclcpp::init(argc,argv);auto node=std::make_shared<OpenVinsNode>(argv[1]);rclcpp::spin(node);rclcpp::shutdown();return node->failure()?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
