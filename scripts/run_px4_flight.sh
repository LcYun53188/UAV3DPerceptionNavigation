#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -f .deps/mission-install/local_setup.bash ]]; then
  echo 'Run scripts/build_px4_sim.sh, then scripts/build_px4_flight.sh first.' >&2
  exit 1
fi
exec ./scripts/with_px4_sim.sh bash -e -c '
  source .deps/mission-install/local_setup.bash
  exec python scripts/run_px4_sitl_smoke.py --duration 20 --flight "$@"
' bash "$@"
