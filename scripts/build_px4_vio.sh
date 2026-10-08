#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
exec ./scripts/with_px4_sim.sh env GZ_DISTRO=harmonic python scripts/build_px4_vio.py "$@"
