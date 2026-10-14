#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
cd "$workspace_dir"
exec ./scripts/with_venv.sh python scripts/build_vio_sdk16_default.py "$@"
