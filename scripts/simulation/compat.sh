#!/usr/bin/env bash
# Invoked through historical symlinks; route by the original filename.
set -euo pipefail
workspace_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
exec "$workspace_dir/scripts/sim.sh" compat "$(basename "$0")" "$@"
