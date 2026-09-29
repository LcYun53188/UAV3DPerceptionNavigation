#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! git lfs version >/dev/null 2>&1; then
  echo "Git LFS is required. Install git-lfs before running this script." >&2
  exit 1
fi
exec python3 "$WS_DIR/scripts/vendor_lfs.py" "$@"
