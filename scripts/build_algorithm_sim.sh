#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
./scripts/prepare_ego_vendor.sh
uv pip install --python .venv/bin/python -r requirements/algorithm-sim.txt
cuda_dir="${CUDA_HOME:-/usr/local/cuda-13.2}"
# Existing Isaac sources remain intact; build only nvblox dependencies.
env SKIP_WS_SETUP=true CMAKE_BUILD_PARALLEL_LEVEL=2 CUDACXX="$cuda_dir/bin/nvcc" CUDAToolkit_ROOT="$cuda_dir" \
 ./scripts/with_venv.sh colcon build --build-base build_uav --install-base install_uav --symlink-install \
 --parallel-workers 2 --cmake-clean-cache --packages-up-to nvblox_ros nvblox_rviz_plugin uav_ego_adapter uav_nav_sim \
 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_COMPILER="$cuda_dir/bin/nvcc" \
 -DCUDAToolkit_ROOT="$cuda_dir" -DCMAKE_CUDA_ARCHITECTURES=89 -DUSE_SYSTEM_EIGEN=ON
# Bringup is a resource package; do not build optional hardware localization stacks.
./scripts/with_venv.sh colcon build --base-paths src/uav_bringup --build-base build_uav \
 --install-base install_uav --packages-select uav_bringup --symlink-install --cmake-args -DBUILD_TESTING=OFF
