#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
exec ./scripts/with_venv.sh bash -e -c '
  source .deps/px4-msgs-install/setup.bash
  source .deps/mission-install/local_setup.bash
  export LD_LIBRARY_PATH="$PWD/.deps/microxrce-install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  exec python scripts/run_px4_vio_sensors.py "$@"
' bash "$@"
