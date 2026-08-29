#!/usr/bin/env bash
set -euo pipefail

PORT="${PX4_SERIAL_PORT:-/dev/ttyUSB0}"
BAUD="${PX4_SERIAL_BAUD:-921600}"
AGENT_BIN="${MICROXRCE_AGENT_BIN:-MicroXRCEAgent}"

usage() {
  cat <<'EOF'
Usage:
  scripts/run_px4_microxrce.sh [options]

Starts the Micro XRCE-DDS Agent over the PX4 serial link.

Environment overrides:
  PX4_SERIAL_PORT      default: /dev/ttyUSB0
  PX4_SERIAL_BAUD      default: 921600
  MICROXRCE_AGENT_BIN  default: MicroXRCEAgent

Options:
  -p, --port <device>   Override serial device path
  -b, --baud <rate>     Override serial baud rate
  --agent <binary>      Override MicroXRCEAgent binary
  -h, --help            Show this help

Examples:
  scripts/run_px4_microxrce.sh
  PX4_SERIAL_PORT=/dev/ttyTHS1 scripts/run_px4_microxrce.sh
  scripts/run_px4_microxrce.sh --port /dev/ttyUSB0 --baud 921600
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -p|--port)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      PORT="$2"
      shift 2
      ;;
    -b|--baud)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      BAUD="$2"
      shift 2
      ;;
    --agent)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      AGENT_BIN="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v "$AGENT_BIN" >/dev/null 2>&1; then
  echo "Micro XRCE-DDS Agent not found: $AGENT_BIN" >&2
  echo "Install it or set MICROXRCE_AGENT_BIN to the binary path." >&2
  exit 1
fi

if [[ ! -e "$PORT" ]]; then
  echo "Serial device not found: $PORT" >&2
  echo "Check the USB-TTL adapter or the Jetson UART port path." >&2
  exit 1
fi

echo "Starting Micro XRCE-DDS Agent for PX4..."
echo "  device: $PORT"
echo "  baud:   $BAUD"
echo "  binary: $AGENT_BIN"

echo "Recommended PX4 settings: MAV_1_CONFIG=0, UXRCE_DDS_CFG=TELEM2, SER_TEL2_BAUD=921600"
exec "$AGENT_BIN" serial --dev "$PORT" -b "$BAUD"
