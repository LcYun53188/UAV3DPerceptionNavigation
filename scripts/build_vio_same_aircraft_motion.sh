#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
exec ./scripts/with_venv.sh bash -e -c '
  cmake -S simulation/px4/vio/same_aircraft_motion -B .deps/vio-same-aircraft-build -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build .deps/vio-same-aircraft-build --parallel 2
  python scripts/record_vio_same_aircraft_build.py
'
