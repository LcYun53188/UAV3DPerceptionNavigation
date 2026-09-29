#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$workspace_dir/.deps/ego_planner_src"
readarray -t source_info < <(python3 - "$workspace_dir" <<'PYINFO'
import json, sys
from pathlib import Path
entries = json.loads((Path(sys.argv[1]) / 'patches/vendor/sources.json').read_text())['dependencies']
entry = next(e for e in entries if e['path'] == '.deps/ego_planner_src')
print(entry['url'])
print(entry['commit'])
PYINFO
)
if [ ! -e "$source_dir" ]; then
  mkdir -p "$(dirname "$source_dir")"
  git clone --no-checkout "${source_info[0]}" "$source_dir"
  git -C "$source_dir" checkout --detach "${source_info[1]}"
fi
exec python3 "$workspace_dir/scripts/vendor_patches.py" --ego --apply
