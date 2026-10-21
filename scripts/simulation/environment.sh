#!/usr/bin/env bash
# ROS setup files do not support nounset; load them with the existing wrappers.
set -eo pipefail
workspace_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
cd "$workspace_dir"
environment_kind="${1:-}"
if [[ "$#" -lt 2 ]]; then
  echo 'Usage: environment.sh <px4|mission|vio> <command> [arguments...]' >&2
  exit 2
fi
shift
export PX4_PARAM_EKF2_GPS_CTRL=0
export GZ_SIM_SERVER_CONFIG_PATH="$workspace_dir/simulation/px4/server_control.config"
case "$environment_kind" in
  px4)
    exec ./scripts/with_px4_sim.sh "$@"
    ;;
  mission|vio)
    for required_setup in .deps/px4-msgs-install/setup.bash .deps/mission-install/local_setup.bash; do
      if [[ ! -f "$required_setup" ]]; then
        echo "Missing $required_setup; run scripts/build/build_px4_sim.sh and scripts/build/build_px4_flight.sh." >&2
        exit 1
      fi
    done
    if [[ "$environment_kind" == mission ]]; then
      exec ./scripts/with_px4_sim.sh bash -e -c '
        source .deps/mission-install/local_setup.bash
        exec "$@"
      ' bash "$@"
    fi
    # VIO needs the perception overlay in install_uav as well as isolated PX4.
    exec ./scripts/with_venv.sh bash -e -c '
      source .deps/px4-msgs-install/setup.bash
      source .deps/mission-install/local_setup.bash
      export LD_LIBRARY_PATH="$PWD/.deps/microxrce-install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
      exec env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$@"
    ' bash "$@"
    ;;
  *)
    echo "Unknown simulation environment: $environment_kind" >&2
    exit 2
    ;;
esac
