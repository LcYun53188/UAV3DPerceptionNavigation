#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$WS_DIR/scripts/vendor_patches.py" "$@"
