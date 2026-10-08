#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "px4-flight" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_flight.sh" "$@"
fi
if [[ "${1:-}" == "px4-depth" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" python "$SCRIPT_DIR/run_px4_sitl_smoke.py" --depth-camera "$@"
fi
if [[ "${1:-}" == "px4-vision-audit" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" bash -e -c '
    source .deps/mission-install/local_setup.bash
    exec python scripts/run_px4_sitl_smoke.py --vision-fusion-smoke "$@"
  ' bash "$@"
fi
exec python3 "$SCRIPT_DIR/sim_control.py" "$@"
