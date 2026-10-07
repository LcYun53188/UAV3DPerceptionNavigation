#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "px4-flight" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_flight.sh" "$@"
fi
exec python3 "$SCRIPT_DIR/sim_control.py" "$@"
