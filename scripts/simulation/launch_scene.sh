#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# All bundled worlds and bridges use the same global ROS / Gazebo topic names.
# Running two copies makes odometry and TF alternate between two robots, which
# looks like violent model jitter in RViz. Hold this descriptor across exec so
# only one stack can run unless the caller explicitly opts out.
if [ "${ALLOW_MULTIPLE_GAZEBO_SIM:-false}" != "true" ]; then
  SIM_LOCK_FILE="/tmp/uav_nav_gazebo_harmonic_${UID:-user}.lock"
  exec 9>"$SIM_LOCK_FILE"
  if ! flock -n 9; then
    echo "A UAV Gazebo simulation is already running." >&2
    echo "Stop it before starting another instance." >&2
    exit 1
  fi

  if pgrep -f '[r]os2 launch uav_bringup gazebo_harmonic_nav.launch.py' >/dev/null \
    || pgrep -f '[g]z sim.*uav_nav_ws.*\.sdf' >/dev/null; then
    echo "An existing UAV Gazebo process was found." >&2
    echo "Wait for it to exit before starting another instance." >&2
    exit 1
  fi
fi

exec "$WS_DIR/scripts/with_venv.sh" ros2 launch uav_bringup gazebo_harmonic_nav.launch.py "$@"
