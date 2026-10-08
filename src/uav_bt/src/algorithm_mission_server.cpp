#include <algorithm>
#include <functional>
#include <iostream>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <unordered_map>
#include <openssl/sha.h>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <uav_nav_interfaces/action/execute_mission.hpp>
#include <uav_nav_interfaces/action/navigate_to_pose3_d.hpp>
#include <uav_nav_interfaces/msg/control_status.hpp>
#include <uav_nav_interfaces/msg/mission_progress.hpp>
#include <uav_nav_interfaces/srv/manage_control_session.hpp>
#include <uav_nav_interfaces/srv/pause_mission.hpp>
#include <uav_nav_interfaces/srv/resume_mission.hpp>

using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;
using Mission = uav_nav_interfaces::action::ExecuteMission;
using Root = rclcpp_action::ServerGoalHandle<Mission>;
using Navigate = uav_nav_interfaces::action::NavigateToPose3D;
using Child = rclcpp_action::ClientGoalHandle<Navigate>;
using Manage = uav_nav_interfaces::srv::ManageControlSession;
using Pause = uav_nav_interfaces::srv::PauseMission;
using Resume = uav_nav_interfaces::srv::ResumeMission;
using Json = nlohmann::json;
static volatile std::sig_atomic_t stopping = 0;
static void signal_stop(int) { stopping = 1; }
static double seconds(Steady::time_point from) {
  return std::chrono::duration<double>(Steady::now()-from).count();
}
static std::string digest(const std::string & input) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.data()), input.size(), hash);
  std::ostringstream stream;
  for (auto byte : hash) { stream << std::hex << std::setfill('0') << std::setw(2) << int(byte); }
  return stream.str();
}

class Waypoints : public BT::StatefulActionNode {
public:
  Waypoints(const std::string & name, const BT::NodeConfig & config,
    std::function<BT::NodeStatus()> step, std::function<void()> cancel)
  : BT::StatefulActionNode(name, config), step_(step), cancel_(cancel) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return step_(); }
  BT::NodeStatus onRunning() override { return step_(); }
  void onHalted() override { cancel_(); }
private:
  std::function<BT::NodeStatus()> step_;
  std::function<void()> cancel_;
};

// One executor thread owns the root, BT, child callbacks and pause checkpoint.
// A halted/paused tree never destroys a client waiting for late acceptance.
class AlgorithmMissionServer {
public:
  explicit AlgorithmMissionServer(rclcpp::Node::SharedPtr node) : node_(node) {
    std::random_device random;
    instance_ = "algorithm-"+std::to_string(random())+"-"+std::to_string(random());
    rcl_interfaces::msg::ParameterDescriptor identity_descriptor;
    identity_descriptor.read_only = true;
    node_->declare_parameter<std::string>("coordinator_instance", instance_, identity_descriptor, true);
    pause_limit_ = node_->declare_parameter<double>("pause_timeout_s", 60.);
    if (!std::isfinite(pause_limit_) || pause_limit_ <= 0. || pause_limit_ > 60.) {
      throw std::runtime_error("pause_timeout_s must be in (0,60]");
    }
    checkpoint_file_ = node_->declare_parameter<std::string>("checkpoint_file", "");
    nav_ = rclcpp_action::create_client<Navigate>(node_, "/uav/algorithm/navigate");
    manage_ = node_->create_client<Manage>("/uav/algorithm/control_session");
    progress_ = node_->create_publisher<uav_nav_interfaces::msg::MissionProgress>(
      "/uav/algorithm/mission_progress", 10);
    control_ = node_->create_subscription<uav_nav_interfaces::msg::ControlStatus>(
      "/uav/algorithm/control_status", 10, [this](uav_nav_interfaces::msg::ControlStatus::ConstSharedPtr status) {
        if (root_ && status->session == session_ && status->coordinator_instance == instance_) {
          control_value_ = *status; control_at_ = Steady::now(); control_seen_ = true;
        }
      });
    pause_ = node_->create_service<Pause>("/uav/algorithm/pause_mission",
      [this](std::shared_ptr<Pause::Request> request, std::shared_ptr<Pause::Response> response) {
        command(*request, *response, true);
      });
    resume_ = node_->create_service<Resume>("/uav/algorithm/resume_mission",
      [this](std::shared_ptr<Resume::Request> request, std::shared_ptr<Resume::Response> response) {
        command(*request, *response, false);
      });
    server_ = rclcpp_action::create_server<Mission>(node_, "/uav/algorithm/execute_mission",
      [this](auto, const std::shared_ptr<const Mission::Goal> goal) {
        if (stopping || root_ || reserved_ || blocked_ || !valid(*goal)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        reserved_ = true; reserved_at_ = Steady::now();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [this](std::shared_ptr<Root> handle) {
        if (handle != root_ || !ending_code_.empty()) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        end("CANCELED", "CANCEL_REQUESTED");
        return rclcpp_action::CancelResponse::ACCEPT;
      }, [this](std::shared_ptr<Root> handle) { accepted(handle); });
    factory_.registerBuilder<Waypoints>("ExecuteWaypoints",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<Waypoints>(name, config,
          [this]() { return step(); }, [this]() { cancel_child(); });
      });
    tree_path_ = ament_index_cpp::get_package_share_directory("uav_bt")+
      "/trees/algorithm_waypoints.xml";
  }

  bool active() const { return root_ != nullptr; }
  bool blocked() const { return blocked_; }
  void tick() {
    if (!root_) {
      if (reserved_ && seconds(reserved_at_) >= 5.) {
        blocked_ = true; reserved_ = false;
        RCLCPP_ERROR(node_->get_logger(), "ACCEPTANCE_UNCONFIRMED_RESTART_REQUIRED");
      }
      return;
    }
    if (remaining() <= 0.) { complete("ABORTED", "MISSION_DEADLINE_STOP_UNCONFIRMED", false); return; }
    if (stopping) { end("ABORTED", "SERVER_EXIT_REQUESTED"); }
    if (remaining() <= 7.) { end("ABORTED", "MISSION_TIMEOUT"); }
    tree_->tickOnce();
    if (!root_) { return; }
    // Published only after this root's BT tick. No independent lease timer.
    if (acquired_) {
      uav_nav_interfaces::msg::MissionProgress progress;
      progress.mission_uuid = session_.session_id;
      progress.coordinator_instance = instance_;
      progress.tick_sequence = ++ticks_;
      progress_->publish(progress);
    }
    auto feedback = std::make_shared<Mission::Feedback>();
    auto & status = feedback->status;
    status.header.stamp = node_->now();
    status.mission_uuid.uuid = root_->get_goal_id();
    status.coordinator_instance = instance_;
    status.event_sequence = ++events_;
    status.phase = phase_;
    status.reason = ending_reason_;
    status.control_session = session_;
    status.map_session = config_.at("map_session").get<std::string>();
    status.total_remaining_s = std::max(0., remaining());
    status.has_child = child_ != nullptr && !child_done_;
    if (status.has_child) { status.child_uuid.uuid = child_->get_goal_id(); }
    feedback->tree_node = "ExecuteWaypoints["+std::to_string(index_)+"]";
    feedback->fault = ending_code_ == "ABORTED" ? ending_reason_ : "";
    root_->publish_feedback(feedback);
  }

private:
  bool valid(const Mission::Goal & goal) {
    try {
      if (goal.backend != "ALGORITHM" || goal.mission_type != "WAYPOINTS" ||
          !std::isfinite(goal.timeout_s) || goal.timeout_s < 20. || goal.timeout_s > 600.) { return false; }
      const auto config = Json::parse(goal.parameters_json);
      if (config.at("coordinator_instance").get<std::string>() != instance_) { return false; }
      const auto & waypoints = config.at("waypoints");
      if (!waypoints.is_array() || waypoints.size() < 2 || waypoints.size() > 32 ||
          config.at("map_session").get<std::string>().empty()) { return false; }
      for (const auto & point : waypoints) {
        if (!point.is_array() || point.size() != 3) { return false; }
        for (const auto & value : point) {
          if (!value.is_number() || !std::isfinite(value.get<double>())) { return false; }
        }
      }
      const double budget = config.value("navigation_timeout_s", 90.);
      return std::isfinite(budget) && budget >= 1. && budget <= 120.;
    } catch (const std::exception &) { return false; }
  }

  void accepted(const std::shared_ptr<Root> & handle) {
    if (tree_) { tree_->haltTree(); tree_.reset(); }
    root_ = handle;
    config_ = Json::parse(handle->get_goal()->parameters_json);
    definition_hash_ = digest(handle->get_goal()->backend+handle->get_goal()->mission_type+config_.dump());
    started_ = Steady::now();
    phase_ = "ACQUIRING";
    ending_code_.clear(); ending_reason_.clear();
    acquired_ = acquire_pending_ = release_pending_ = control_seen_ = false;
    acquire_sent_ = false; ticks_ = events_ = 0; index_ = 0;
    decisions_.clear(); reset_child(); motion_used_ = 0.;
    session_.session_id.uuid = handle->get_goal_id();
    session_.generation = ++generation_;
    session_.owner = instance_;
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_path_));
    checkpoint();
  }

  double remaining() const { return root_->get_goal()->timeout_s-seconds(started_); }
  bool control_healthy() const {
    return control_seen_ && !control_value_.mock && seconds(control_at_) < .5 && control_value_.reason.empty() &&
      control_value_.lease_remaining_s > 0. &&
      std::find(control_value_.allowed_operations.begin(), control_value_.allowed_operations.end(),
                "NAVIGATE") != control_value_.allowed_operations.end();
  }
  void end(const std::string & code, const std::string & reason) {
    if (!root_) { return; }
    if (ending_code_.empty() || (code == "ABORTED" &&
        (ending_code_ != "ABORTED" || ending_reason_ == "CONTROL_STATUS_STALE"))) {
      ending_code_ = code; ending_reason_ = reason; phase_ = "STOPPING"; checkpoint();
    }
    cancel_child();
  }

  template<class Request, class Response>
  void command(const Request & request, Response & response, bool pause) {
    response.phase = phase_;
    if (!root_ || request.mission_uuid.uuid != root_->get_goal_id() ||
        request.coordinator_instance != instance_) { response.reason = "MISSION_IDENTITY_MISMATCH"; return; }
    bool nonzero = false;
    std::string key;
    for (auto byte : request.request_id.uuid) { nonzero |= byte != 0; key += char(byte); }
    if (!nonzero) { response.reason = "INVALID_REQUEST_ID"; return; }
    auto previous = decisions_.find(key);
    if (previous != decisions_.end()) {
      if (previous->second.pause != pause) { response.reason = "REQUEST_ID_REUSED"; return; }
      response.accepted = previous->second.accepted;
      response.reason = previous->second.reason;
      response.phase = previous->second.phase;
      return;
    }
    if (decisions_.size() >= 256) { response.reason = "REQUEST_LIMIT"; return; }
    if (!ending_code_.empty()) { response.reason = "STOPPING"; }
    else if (pause && phase_ == "RUNNING") {
      response.accepted = true; phase_ = "PAUSING"; cancel_child(); checkpoint();
    } else if (!pause && phase_ == "PAUSED" && control_healthy() && remaining() > 8.) {
      response.accepted = true; phase_ = "RESUMING"; checkpoint();
    } else { response.reason = "INVALID_PHASE_OR_HEALTH"; }
    response.phase = phase_;
    decisions_[key] = {pause, response.accepted, response.reason, response.phase};
  }

  BT::NodeStatus step() {
    if (!root_) { return BT::NodeStatus::SUCCESS; }
    if (acquired_ && phase_ != "ACQUIRING" && !control_healthy() &&
        !(release_pending_ && control_seen_ && control_value_.reason == "RELEASED")) {
      end("ABORTED", control_seen_ && !control_value_.reason.empty() ?
          control_value_.reason : "CONTROL_STATUS_STALE");
    }
    if (!ending_code_.empty()) { cleanup(); return root_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS; }
    if (phase_ == "ACQUIRING") {
      if (!acquire_sent_ && nav_->action_server_is_ready() && manage_->service_is_ready()) { acquire(); }
      if (acquired_ && control_healthy()) { phase_ = "RUNNING"; checkpoint(); }
      if (seconds(started_) > 5. && phase_ == "ACQUIRING") { end("ABORTED", "SESSION_HANDSHAKE_TIMEOUT"); }
      return BT::NodeStatus::RUNNING;
    }
    if (phase_ == "PAUSED") {
      if (seconds(paused_at_) >= pause_limit_) { end("ABORTED", "PAUSE_TIMEOUT"); }
      return BT::NodeStatus::RUNNING;
    }
    if (child_done_) {
      const bool confirmed = child_result_.result && child_result_.result->cleanup_confirmed && !child_result_.result->mock;
      const bool success = confirmed && child_result_.code == rclcpp_action::ResultCode::SUCCEEDED &&
        child_result_.result->result_code == "SUCCEEDED";
      const bool canceled = confirmed && child_result_.code == rclcpp_action::ResultCode::CANCELED &&
        child_result_.result->result_code == "CANCELED";
      motion_used_ += seconds(child_started_);
      if (success) { ++index_; motion_used_ = 0.; }
      else if (!(phase_ == "PAUSING" && canceled)) {
        end("ABORTED", child_result_.result ? child_result_.result->reason : "CHILD_REJECTED");
        return BT::NodeStatus::RUNNING;
      }
      reset_child(); checkpoint();
    }
    if (phase_ == "PAUSING") {
      cancel_child();
      if (!child_sent_) { phase_ = "PAUSED"; paused_at_ = Steady::now(); checkpoint(); }
      return BT::NodeStatus::RUNNING;
    }
    if (phase_ == "RESUMING") { reset_child(); phase_ = "RUNNING"; checkpoint(); }
    if (index_ == config_["waypoints"].size()) { end("SUCCEEDED", "WAYPOINTS_COMPLETE"); return BT::NodeStatus::RUNNING; }
    if (!child_sent_) { dispatch(); }
    return BT::NodeStatus::RUNNING;
  }

  void acquire() {
    acquire_sent_ = acquire_pending_ = true;
    auto request = std::make_shared<Manage::Request>();
    request->operation = "ACQUIRE"; request->control_session = session_;
    request->map_session = config_.at("map_session").get<std::string>();
    manage_->async_send_request(request, [this](rclcpp::Client<Manage>::SharedFuture future) {
      acquire_pending_ = false;
      auto response = future.get();
      if (response->accepted) { acquired_ = true; }
      else { end("ABORTED", "SESSION_REJECTED:"+response->reason); }
    });
  }

  void reset_child() {
    child_rejected_ = false;
    child_.reset(); child_sent_ = child_done_ = child_cancel_sent_ = child_cancel_requested_ = false;
    child_result_ = Child::WrappedResult{};
  }
  void cancel_child() {
    child_cancel_requested_ = true;
    if (child_ && !child_done_ && !child_cancel_sent_) {
      child_cancel_sent_ = true;
      nav_->async_cancel_goal(child_);
    }
  }
  void dispatch() {
    const double budget = std::min(config_.value("navigation_timeout_s", 90.)-motion_used_, remaining()-7.);
    if (budget < 1.) { end("ABORTED", "NAVIGATION_BUDGET_EXHAUSTED"); return; }
    child_sent_ = true; child_started_ = Steady::now();
    Navigate::Goal goal;
    goal.goal.header.frame_id = "map"; goal.goal.header.stamp = node_->now();
    const auto & point = config_["waypoints"][index_];
    goal.goal.pose.position.x = point[0]; goal.goal.pose.position.y = point[1]; goal.goal.pose.position.z = point[2];
    goal.goal.pose.orientation.w = 1.; goal.position_tolerance_m = .2;
    goal.stopped_speed_mps = .05; goal.stable_duration_s = .6; goal.timeout_s = budget;
    goal.control_session = session_; goal.map_session = config_["map_session"];
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    const auto root_id = root_->get_goal_id();
    options.goal_response_callback = [this, root_id](Child::SharedPtr handle) {
      if (!root_ || root_->get_goal_id() != root_id) {
        if (handle) { nav_->async_cancel_goal(handle); } return;
      }
      if (!handle) { child_rejected_ = child_done_ = true; return; }
      child_ = handle;
      if (child_cancel_requested_) { cancel_child(); }
      checkpoint();
    };
    options.feedback_callback = [this, root_id](Child::SharedPtr handle, auto feedback) {
      if (!root_ || root_->get_goal_id() != root_id) { return; }
      if (feedback->status.control_session != session_ ||
          feedback->status.mission_uuid.uuid != handle->get_goal_id()) {
        end("ABORTED", "CHILD_FEEDBACK_IDENTITY_MISMATCH");
      }
    };
    options.result_callback = [this, root_id](const Child::WrappedResult & result) {
      if (!root_ || root_->get_goal_id() != root_id) { return; }
      if (!child_ || result.goal_id != child_->get_goal_id()) { end("ABORTED", "INVALID_CHILD_RESULT"); return; }
      child_result_ = result; child_done_ = true;
    };
    nav_->async_send_goal(goal, options);
  }

  void cleanup() {
    cancel_child();
    if (acquire_pending_ || (child_sent_ && !child_done_)) { return; }
    if (child_done_ && !child_rejected_ && (!child_result_.result || !child_result_.result->cleanup_confirmed || child_result_.result->mock)) {
      complete("ABORTED", "CHILD_STOP_UNCONFIRMED", false); return;
    }
    if (child_done_ && !child_rejected_) {
      const auto & value = *child_result_.result;
      const bool consistent =
        (child_result_.code == rclcpp_action::ResultCode::SUCCEEDED && value.result_code == "SUCCEEDED") ||
        (child_result_.code == rclcpp_action::ResultCode::CANCELED && value.result_code == "CANCELED") ||
        (child_result_.code == rclcpp_action::ResultCode::ABORTED && value.result_code == "ABORTED");
      if (!consistent) { end("ABORTED", "INVALID_CHILD_RESULT"); }
    }
    if (child_done_ && child_result_.code == rclcpp_action::ResultCode::ABORTED) {
      end("ABORTED", child_result_.result->reason);
    }
    if (!acquired_) { complete(ending_code_, ending_reason_, true); return; }
    if (release_pending_) { return; }
    release_pending_ = true;
    auto request = std::make_shared<Manage::Request>();
    request->operation = "RELEASE"; request->control_session = session_;
    manage_->async_send_request(request, [this](rclcpp::Client<Manage>::SharedFuture future) {
      release_pending_ = false;
      if (!root_) { return; }
      const auto response = future.get();
      if (response->accepted) {
        if (!response->reason.empty()) { end("ABORTED", response->reason); }
        acquired_ = false; complete(ending_code_, ending_reason_, true);
      }
      else if (response->reason != "STOP_UNCONFIRMED") { complete("ABORTED", "RELEASE_FAILED:"+response->reason, false); }
    });
  }
  void complete(const std::string & code, const std::string & reason, bool confirmed) {
    if (!root_) { return; }
    auto result = std::make_shared<Mission::Result>();
    result->result_code = code; result->reason = reason; result->cleanup_confirmed = confirmed;
    phase_ = code; checkpoint();
    if (code == "SUCCEEDED") { root_->succeed(result); }
    else if (code == "CANCELED" && root_->is_canceling()) { root_->canceled(result); }
    else { result->result_code = "ABORTED"; root_->abort(result); }
    RCLCPP_INFO(node_->get_logger(), "MISSION_RESULT %s %s cleanup=%d waypoint=%zu ticks=%lu",
      result->result_code.c_str(), reason.c_str(), confirmed, index_, ticks_);
    blocked_ |= !confirmed; root_.reset(); reserved_ = false;
  }
  void checkpoint() {
    if (checkpoint_file_.empty() || !root_) { return; }
    Json value = {{"mission_uuid", rclcpp_action::to_string(root_->get_goal_id())},
      {"coordinator_instance", instance_}, {"definition_sha256", definition_hash_},
      {"definition", config_}, {"waypoint_index", index_}, {"phase", phase_},
      {"remaining_total_s", std::max(0., remaining())},
      {"remaining_navigation_s", config_.value("navigation_timeout_s", 90.)-motion_used_},
      {"control_generation", session_.generation}, {"restart_resume_supported", false}};
    if (child_) { value["child_uuid"] = rclcpp_action::to_string(child_->get_goal_id()); }
    const std::filesystem::path path(checkpoint_file_);
    if (path.has_parent_path()) { std::filesystem::create_directories(path.parent_path()); }
    std::ofstream output(checkpoint_file_+".tmp"); output << value.dump(2) << "\n"; output.close();
    if (!output) { throw std::runtime_error("Cannot write mission checkpoint"); }
    std::filesystem::rename(checkpoint_file_+".tmp", checkpoint_file_);
  }

  struct Decision { bool pause, accepted; std::string reason, phase; };
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Mission>::SharedPtr server_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_;
  rclcpp::Client<Manage>::SharedPtr manage_;
  rclcpp::Service<Pause>::SharedPtr pause_;
  rclcpp::Service<Resume>::SharedPtr resume_;
  rclcpp::Publisher<uav_nav_interfaces::msg::MissionProgress>::SharedPtr progress_;
  rclcpp::Subscription<uav_nav_interfaces::msg::ControlStatus>::SharedPtr control_;
  std::shared_ptr<Root> root_;
  Child::SharedPtr child_;
  Child::WrappedResult child_result_;
  uav_nav_interfaces::msg::ControlSession session_;
  uav_nav_interfaces::msg::ControlStatus control_value_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  Json config_;
  std::unordered_map<std::string, Decision> decisions_;
  std::string instance_, checkpoint_file_, tree_path_, definition_hash_, phase_, ending_code_, ending_reason_;
  Steady::time_point reserved_at_, started_, control_at_, child_started_, paused_at_;
  size_t index_ = 0;
  uint64_t generation_ = 0, ticks_ = 0, events_ = 0;
  double motion_used_ = 0., pause_limit_ = 60.;
  bool reserved_ = false, blocked_ = false, acquired_ = false, control_seen_ = false;
  bool acquire_sent_ = false, acquire_pending_ = false, release_pending_ = false;
  bool child_rejected_ = false;
  bool child_sent_ = false, child_done_ = false, child_cancel_requested_ = false, child_cancel_sent_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_stop); std::signal(SIGTERM, signal_stop);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("algorithm_mission_server");
    AlgorithmMissionServer server(node);
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    auto next = Steady::now();
    while (!stopping || server.active()) {
      executor.spin_once(5ms);
      if (Steady::now() >= next) { next = Steady::now()+100ms; server.tick(); }
    }
    const auto drain_until = Steady::now()+200ms;
    while (Steady::now() < drain_until) { executor.spin_once(5ms); }
    if (server.blocked()) { result = 1; }
  } catch (const std::exception & error) {
    std::cerr << "MISSION_SERVER_ERROR " << error.what() << std::endl; result = 1;
  }
  rclcpp::shutdown(); return result;
}
