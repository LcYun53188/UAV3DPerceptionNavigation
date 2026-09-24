#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$WS_DIR/simulation/scripts/run_gazebo_harmonic_nav.sh" arena:=uav_obstacle_course "$@"
