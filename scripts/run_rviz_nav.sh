#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$WS_DIR/scripts/nav_launch.env" ]]; then
  source "$WS_DIR/scripts/nav_launch.env"
fi
if [[ -f "$WS_DIR/scripts/nav_launch.local.env" ]]; then
  source "$WS_DIR/scripts/nav_launch.local.env"
fi

FIXED_FRAME="${RVIZ_FIXED_FRAME:-map}"

usage() {
  cat <<'EOF'
Usage:
  scripts/run_rviz_nav.sh [options] [rviz_arg ...]

Defaults are read from scripts/nav_launch.env and optional
scripts/nav_launch.local.env. Command-line options override variables.

Options:
  --fixed-frame <frame>
      Frame to print as the recommended RViz fixed frame. Default: map.

  -h, --help
      Show this help.
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    echo "$option requires a value" >&2
    exit 2
  fi
}

RVIZ_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fixed-frame)
      require_value "$1" "${2:-}"
      FIXED_FRAME="$2"
      shift 2
      ;;
    --fixed-frame=*)
      FIXED_FRAME="${1#*=}"
      shift
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    --)
      shift
      RVIZ_ARGS+=("$@")
      break
      ;;
    *)
      RVIZ_ARGS+=("$1")
      shift
      ;;
  esac
done

cat <<EOF
RViz navigation view:
  recommended Fixed Frame: $FIXED_FRAME
  add displays:
    TF
    /nav/cmd_vel
    /nav/emergency
    /nav/safety_status
    /oakd/points_filtered
    /mid360/points
    /perception/obstacle_points
EOF

exec "$WS_DIR/scripts/with_venv.sh" rviz2 "${RVIZ_ARGS[@]}"
