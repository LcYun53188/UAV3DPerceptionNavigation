#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$workspace_dir/.deps/ego_planner_src"
commit=23a8d5a191711dd65633df689bd00f55d4dea8f9
patch_file="$workspace_dir/patches/vendor/ego_planner.patch"
if [ ! -d "$source_dir/.git" ]; then
  mkdir -p "$workspace_dir/.deps"
  git clone --no-checkout https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git "$source_dir"
  git -C "$source_dir" checkout --detach "$commit"
fi
if [ "$(git -C "$source_dir" rev-parse HEAD)" != "$commit" ]; then
  echo "EGO revision mismatch; refusing to overwrite checkout" >&2
  exit 2
fi
if git -C "$source_dir" apply --reverse --check "$patch_file" 2>/dev/null; then
  echo "EGO patch already applied: $commit"
else
  git -C "$source_dir" apply --check "$patch_file"
  git -C "$source_dir" apply "$patch_file"
fi
if ! cmp -s <(git -C "$source_dir" diff --binary --full-index) "$patch_file"; then
  echo "EGO checkout has changes outside the recorded patch; refusing reproducible build" >&2
  exit 2
fi
