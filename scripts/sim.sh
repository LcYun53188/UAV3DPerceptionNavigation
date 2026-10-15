#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "pocket-panel" ]]; then
  shift
  exec python3 "$SCRIPT_DIR/pocket_panel.py" "$@"
fi
if [[ "${1:-}" == "px4-vio-pocket" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_vio_sensors.sh" --experimental-chain --flight sequence --pocket-session --normalize --mtf01 --scene warehouse --warehouse-variant avoidance --warehouse-floor-texture --quality-policy bounded_gap --real-time-factor 0.8 --render-device nvidia --sdk-image-depth 1 --headless-rendering --sdk-executor single_thread --sdk-cpu-list 0-15 --image-resolution 480x300 --ui --diagnostic-ui --duration 120 "$@"
fi
if [[ "${1:-}" == "px4-flow-pocket" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" bash -e -c 'source .deps/mission-install/local_setup.bash; exec python scripts/run_px4_mtf01.py --usb-joystick --ui "$@"' bash "$@"
fi
if [[ "${1:-}" == "px4-vio-mtf01" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_vio_sensors.sh" --mtf01 --normalize --fuse-pose --scene warehouse --warehouse-floor-texture --duration 120 "$@"
fi
if [[ "${1:-}" == "px4-flow-local" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" bash -e -c 'source .deps/mission-install/local_setup.bash; exec python scripts/run_px4_mtf01.py --flight "$@"' bash "$@"
fi
if [[ "${1:-}" == "px4-mtf01" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" python "$SCRIPT_DIR/run_px4_mtf01.py" "$@"
fi
if [[ "${1:-}" == "px4-vio-sensors" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_vio_sensors.sh" "$@"
fi
if [[ "${1:-}" == "px4-flight" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_flight.sh" "$@"
fi
if [[ "${1:-}" == "px4-observe" ]]; then
  shift
  exec "$SCRIPT_DIR/run_px4_flight.sh" --bt --navigation-backend EGO --depth-observation "$@"
fi
if [[ "${1:-}" == "px4-depth-plan" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" python "$SCRIPT_DIR/run_px4_sitl_smoke.py" --depth-reference --depth-mapping --depth-planning "$@"
fi
if [[ "${1:-}" == "px4-depth-map" ]]; then
  shift
  exec "$SCRIPT_DIR/with_px4_sim.sh" python "$SCRIPT_DIR/run_px4_sitl_smoke.py" --depth-reference --depth-mapping "$@"
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
