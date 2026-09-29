#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$WS_DIR/scripts/apply_vendor_patches.sh" --check
"$WS_DIR/scripts/install_vendor_lfs_assets.sh" --check
# Dedicated output avoids sourcing stale ground packages from the old install tree.
exec env SKIP_WS_SETUP=true "$WS_DIR/scripts/with_venv.sh" colcon build \
  --build-base build_uav --install-base install_uav --symlink-install \
  --packages-up-to uav_bringup "$@"
