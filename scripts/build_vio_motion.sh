#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
exec ./scripts/with_venv.sh bash -e -c '
  cmake -S simulation/px4/vio/motion -B .deps/vio-motion-build -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build .deps/vio-motion-build --parallel 2
  python scripts/record_vio_motion_build.py
'
