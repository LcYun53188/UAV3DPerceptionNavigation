#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
SDK_SRC="$WS_DIR/third_party/Livox-SDK2"
SDK_INSTALL="${LIVOX_SDK2_ROOT:-$WS_DIR/.deps/livox_sdk2}"
SDK_BUILD="$WS_DIR/.deps/build/livox_sdk2"

if [ ! -d "$SDK_SRC" ]; then
  echo "Missing Livox-SDK2 source at $SDK_SRC" >&2
  echo "Run: git submodule update --init --recursive third_party/Livox-SDK2" >&2
  exit 1
fi

cmake -S "$SDK_SRC" -B "$SDK_BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SDK_INSTALL"
cmake --build "$SDK_BUILD" --parallel
cmake --install "$SDK_BUILD"

echo "Livox-SDK2 installed to $SDK_INSTALL"
