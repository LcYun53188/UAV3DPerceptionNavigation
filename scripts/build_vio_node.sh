#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
# Build the VIO component against installed dependencies. This does not build
# the optional image preprocessing pipeline or claim its CV-CUDA dependency is fixed.
exec env CMAKE_BUILD_PARALLEL_LEVEL=2 ./scripts/with_venv.sh colcon \
  --log-base .cache/simulation/vio-node-colcon-log build --base-paths src \
  --build-base build_uav --install-base install_uav \
  --packages-select isaac_ros_visual_slam --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
