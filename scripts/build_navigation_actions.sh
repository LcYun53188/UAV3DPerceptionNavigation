#!/usr/bin/env bash
# Incremental first-party build; does not rebuild CUDA/nvblox or vendor packages.
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"
SKIP_WS_SETUP=true ./scripts/with_venv.sh colcon build \
  --build-base build_uav --install-base install_uav --symlink-install \
  --base-paths src/uav_nav_interfaces src/uav_nav_sim src/uav_bt \
  --packages-select uav_nav_interfaces uav_nav_sim uav_bt "$@"
