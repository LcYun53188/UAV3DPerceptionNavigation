cmake_minimum_required(VERSION 3.8)
project(uav_nav_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/StateDimension.msg" "msg/AircraftState.msg" "msg/MapSnapshot.msg" "msg/TimedTrajectory.msg" "msg/PlannerStatus.msg" "msg/TrajectoryRequest.msg"

  "msg/ControlSession.msg" "msg/ControlStatus.msg" "msg/TaskStatus.msg" "msg/MissionProgress.msg"
  "action/ExecuteMission.action" "action/NavigateToPose3D.action"
  "srv/PauseMission.srv" "srv/ResumeMission.srv" "srv/ManageControlSession.srv" "srv/AdvanceFlightStep.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces unique_identifier_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
