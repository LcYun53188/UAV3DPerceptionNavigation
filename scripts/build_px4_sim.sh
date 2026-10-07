#!/usr/bin/env bash
# Isolated host SITL/Agent/messages build; no CUDA stack or global installation.
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: scripts/build_px4_sim.sh [--prepare-only] [--jobs N]'
  exit 0
fi
jobs=4
prepare_only=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --prepare-only) prepare_only=true; shift ;;
    --jobs) jobs="${2:?Missing job count}"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'jobs must be a positive integer' >&2; exit 2; }
./scripts/with_venv.sh python scripts/prepare_px4_sim.py
./scripts/with_venv.sh python scripts/check_px4_interfaces.py
if [[ "$prepare_only" == true ]]; then exit 0; fi
if [[ ! -x .deps/px4-venv/bin/python ]]; then uv venv --python 3.11 .deps/px4-venv; fi
uv pip install --python .deps/px4-venv/bin/python -r requirements/px4-sim.txt
# Source ROS/Gazebo package paths but keep the PX4 Python interpreter isolated.
SKIP_WS_SETUP=true ./scripts/with_venv.sh bash -e -c '
  unset PYTHONPATH VIRTUAL_ENV
  export PATH="$PWD/.deps/px4-venv/bin:$PATH"
  export GZ_DISTRO=harmonic
  make -C .deps/PX4-Autopilot px4_sitl_default -j"$1" PYTHON_EXECUTABLE="$PWD/.deps/px4-venv/bin/python"
' bash "$jobs"
SKIP_WS_SETUP=true ./scripts/with_venv.sh bash -e -c '
  cmake -S .deps/Micro-XRCE-DDS-Agent -B .deps/agent-build \
    -DCMAKE_INSTALL_PREFIX="$PWD/.deps/microxrce-install" -DCMAKE_BUILD_TYPE=Release \
    -DUAGENT_P2P_PROFILE=OFF -DUAGENT_CED_PROFILE=OFF
  cmake --build .deps/agent-build --parallel "$1"
  cmake --install .deps/agent-build
' bash "$jobs"
CMAKE_BUILD_PARALLEL_LEVEL="$jobs" SKIP_WS_SETUP=true ./scripts/with_venv.sh colcon \
  --log-base .cache/simulation/px4-msgs-colcon-log build --base-paths .deps/px4_msgs \
  --build-base .deps/px4-msgs-build --install-base .deps/px4-msgs-install --packages-select px4_msgs

./scripts/with_venv.sh python scripts/prepare_px4_sim.py --check --check-external
