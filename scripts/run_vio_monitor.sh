#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -f .deps/mission-install/local_setup.bash ]]; then
  echo 'Run scripts/build_px4_flight.sh first.' >&2
  exit 1
fi
exec ./scripts/with_px4_sim.sh bash -e -c '
  source .deps/mission-install/local_setup.bash
  interface_setup=install_uav/isaac_ros_visual_slam_interfaces/share/isaac_ros_visual_slam_interfaces/local_setup.bash
  if [[ ! -f "$interface_setup" ]]; then
    echo "Missing built isaac_ros_visual_slam_interfaces; build the existing VIO stack first." >&2
    exit 1
  fi
  source "$interface_setup"
  exec ros2 run px4_comm_bridge vio_input_node "$@"
' bash "$@"
