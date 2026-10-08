#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <nlohmann/json.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_nav_interfaces/action/execute_mission.hpp>
#include <uav_nav_interfaces/msg/mission_progress.hpp>

using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;
using Mission = uav_nav_interfaces::action::ExecuteMission;
using Handle = rclcpp_action::ClientGoalHandle<Mission>;
static volatile std::sig_atomic_t stop_requested = 0;
static void request_stop(int) { stop_requested = 1; }

// All callbacks, tree ticks, progress and cleanup run on the same executor thread.
// The client outlives halted tree nodes, including late goal responses.
class MissionClient
{
public:
  explicit MissionClient(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    action_ = rclcpp_action::create_client<Mission>(node_, "/uav/px4/execute_mission");
    progress_ = node_->create_publisher<uav_nav_interfaces::msg::MissionProgress>(
      "/uav/px4/mission_progress", 10);
    instance_ = node_->declare_parameter<std::string>("coordinator_instance", "");
    const auto path = node_->declare_parameter<std::string>("parameters_file", "");
    std::ifstream input(path);
    if (!input || instance_.empty()) {
      throw std::runtime_error("parameters_file and coordinator_instance are required");
    }
    parameters_ = std::string(std::istreambuf_iterator<char>(input), {});
    const auto parameters = nlohmann::json::parse(parameters_);
    if (!parameters.value("runner_progress_required", false) ||
        parameters.value("coordinator_instance", "") != instance_) {
      throw std::runtime_error("Mission must require progress from the current coordinator");
    }
    budget_ = node_->declare_parameter<double>("timeout_s", 210.0);
    if (!std::isfinite(budget_) || budget_ < 20.0 || budget_ > 240.0) {
      throw std::runtime_error("timeout_s must be 20..240 seconds");
    }
  }

  void start()
  {
    if (started_) { throw std::runtime_error("Root mission must never be dispatched twice"); }
    started_ = true;
    discovery_deadline_ = Steady::now() + 5s;
    mission_deadline_ = Steady::now() + std::chrono::duration_cast<Steady::duration>(
      std::chrono::duration<double>(budget_ + 5.0));
  }

  BT::NodeStatus poll()
  {
    if (!sent_ && !done_) {
      if (cancel_requested_) { finish("CANCELED", "BEFORE_DISPATCH", true, 130); }
      else if (action_->action_server_is_ready()) { send(); }
      else if (Steady::now() > discovery_deadline_) {
        finish("ABORTED", "SERVER_UNAVAILABLE", false, 1);
      }
    }
    if (!done_ && Steady::now() > mission_deadline_) { cancel(); }
    if (!done_) { return BT::NodeStatus::RUNNING; }
    return exit_code_ == 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void renew_after_tick()
  {
    if (!handle_ || done_ || cancel_requested_) { return; }
    uav_nav_interfaces::msg::MissionProgress progress;
    progress.mission_uuid.uuid = handle_->get_goal_id();
    progress.coordinator_instance = instance_;
    progress.tick_sequence = ++tick_sequence_;
    progress_->publish(progress);
  }

  void cancel()
  {
    cancel_requested_ = true;
    if (!sent_) { finish("CANCELED", "BEFORE_DISPATCH", true, 130); return; }
    // A response can arrive after halt(); preserve the client and cancel then.
    if (handle_ && !cancel_sent_ && !done_) {
      cancel_sent_ = true;
      action_->async_cancel_goal(handle_, [this](auto response) {
        cancel_rejected_ = response->goals_canceling.empty();
        RCLCPP_INFO(node_->get_logger(), "Cancel %s; awaiting terminal result",
          cancel_rejected_ ? "rejected" : "accepted");
      });
    }
  }

  bool done() const { return done_; }
  int exit_code() const { return exit_code_; }
  bool landing() const { return phase_ == "LAND_REQUEST" || phase_ == "LANDING"; }
  Steady::time_point deadline() const { return mission_deadline_ + 10s; }

  void cleanup_timeout() { finish("ABORTED", "CLEANUP_UNCONFIRMED", false, 1); }

private:
  void send()
  {
    sent_ = true;
    Mission::Goal goal;
    goal.backend = "PX4_KNOWN_REGION";
    goal.mission_type = "FLIGHT_SEQUENCE";
    goal.parameters_json = parameters_;
    goal.timeout_s = budget_;
    rclcpp_action::Client<Mission>::SendGoalOptions options;
    options.goal_response_callback = [this](Handle::SharedPtr handle) {
      if (!handle) { finish("REJECTED", "GOAL_REJECTED", false, 1); return; }
      handle_ = handle;
      RCLCPP_INFO(node_->get_logger(), "Root mission accepted");
      if (cancel_requested_) { cancel(); }
    };
    options.feedback_callback = [this](Handle::SharedPtr, auto feedback) {
      if (!handle_ || feedback->status.coordinator_instance != instance_ ||
          feedback->status.mission_uuid.uuid != handle_->get_goal_id()) {
        cancel();
        return;
      }
      phase_ = feedback->status.phase;
    };
    options.result_callback = [this](const Handle::WrappedResult & result) {
      if (!result.result || !handle_ || result.goal_id != handle_->get_goal_id()) {
        finish("ABORTED", "INVALID_RESULT", false, 1); return;
      }
      const auto & value = *result.result;
      const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        value.result_code == "SUCCEEDED" && value.cleanup_confirmed && !value.mock;
      const bool canceled = result.code == rclcpp_action::ResultCode::CANCELED &&
        value.result_code == "CANCELED" && value.cleanup_confirmed && !value.mock;
      finish(value.result_code, value.reason, value.cleanup_confirmed,
        success ? 0 : canceled ? 130 : 1);
    };
    action_->async_send_goal(goal, options);
    RCLCPP_INFO(node_->get_logger(), "Dispatched root mission once");
  }

  void finish(const std::string & code, const std::string & reason, bool confirmed, int exit_code)
  {
    if (done_) { return; }
    done_ = true;
    exit_code_ = exit_code;
    // Machine-readable evidence; quoted() also escapes exception-like reasons.
    std::cout << "BT_RESULT " << std::quoted(code) << " " << std::quoted(reason)
              << " cleanup=" << confirmed << " dispatches=" << (sent_ ? 1 : 0)
              << " ticks=" << tick_sequence_ << " exit=" << exit_code_ << std::endl;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Mission>::SharedPtr action_;
  rclcpp::Publisher<uav_nav_interfaces::msg::MissionProgress>::SharedPtr progress_;
  Handle::SharedPtr handle_;
  std::string instance_, parameters_, phase_;
  double budget_ = 210.0;
  bool started_ = false, sent_ = false, done_ = false;
  bool cancel_requested_ = false, cancel_sent_ = false, cancel_rejected_ = false;
  int exit_code_ = 1;
  uint64_t tick_sequence_ = 0;
  Steady::time_point discovery_deadline_, mission_deadline_;
};

class ExecuteFlightMission : public BT::StatefulActionNode
{
public:
  ExecuteFlightMission(const std::string & name, const BT::NodeConfig & config, MissionClient & client)
  : BT::StatefulActionNode(name, config), client_(client) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { client_.start(); return client_.poll(); }
  BT::NodeStatus onRunning() override { return client_.poll(); }
  void onHalted() override { client_.cancel(); }
private:
  MissionClient & client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int exit_code = 1;
  try {
    auto node = std::make_shared<rclcpp::Node>("uav_bt_runner");
    MissionClient client(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<ExecuteFlightMission>("ExecuteFlightMission",
      [&client](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<ExecuteFlightMission>(name, config, client);
      });
    const auto default_tree = ament_index_cpp::get_package_share_directory("uav_bt") +
      "/trees/px4_flight.xml";
    auto tree = factory.createTreeFromFile(node->declare_parameter<std::string>("tree_file", default_tree));
    bool halted = false;
    auto next_tick = Steady::now();
    while (!client.done()) {
      // Drain Action events continuously. A single spin_some per 100 ms can
      // starve goal/cancel replies behind a 50 Hz feedback subscription.
      executor.spin_once(5ms);
      if (Steady::now() < next_tick) { continue; }
      next_tick = Steady::now() + 100ms;
      if (stop_requested && !halted && !client.landing()) {
        tree.haltTree();
        client.cancel();
        halted = true;
      }
      if (!halted) {
        const auto state = tree.tickOnce();
        if (state == BT::NodeStatus::RUNNING) { client.renew_after_tick(); }
        else if (!client.done()) { tree.haltTree(); client.cancel(); halted = true; }
      } else {
        client.poll();
      }
      if (Steady::now() > client.deadline()) { client.cleanup_timeout(); }
    }
    exit_code = client.exit_code();
    // No further ticks after terminal result. Pending cleanup has finished above.
  } catch (const std::exception & error) {
    std::cerr << "BT_ERROR " << error.what() << std::endl;
  }
  rclcpp::shutdown();
  return exit_code;
}
