#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -f .deps/px4-msgs-install/setup.bash ]]; then
  echo 'Missing isolated px4_msgs install; run scripts/build_px4_sim.sh' >&2
  exit 1
fi
exec env SKIP_WS_SETUP=true ./scripts/with_venv.sh bash -e -c '
  source .deps/px4-msgs-install/setup.bash
  export LD_LIBRARY_PATH="$PWD/.deps/microxrce-install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export GZ_DISTRO=harmonic
  exec "$@"
' bash "$@"
