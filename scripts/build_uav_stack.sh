#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Dedicated output avoids sourcing stale ground packages from the old install tree.
exec env SKIP_WS_SETUP=true "$WS_DIR/scripts/with_venv.sh" colcon build \
  --build-base build_uav --install-base install_uav --symlink-install \
  --packages-up-to uav_bringup "$@"
