#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
exec ./scripts/with_px4_sim.sh colcon --log-base .cache/simulation/mission-colcon-log build \
  --base-paths src/uav_nav_interfaces src/px4_comm_bridge src/uav_mission src/uav_bt \
  --build-base .deps/mission-build --install-base .deps/mission-install \
  --symlink-install --parallel-workers 1 "$@"
