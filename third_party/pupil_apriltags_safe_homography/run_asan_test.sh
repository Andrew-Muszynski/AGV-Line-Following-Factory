#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bash "$SCRIPT_DIR/build_wheel.sh"
SOURCE_DIR=$(find "$SCRIPT_DIR/build" -maxdepth 1 -type d \
  -name 'source.*' -printf '%T@ %p\n' | sort -n | tail -n1 | cut -d' ' -f2-)
ASAN_DIR="$SOURCE_DIR/asan"

cmake -S "$SOURCE_DIR/apriltags-source" -B "$ASAN_DIR" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_FLAGS='-fsanitize=address -fno-omit-frame-pointer' \
  -DCMAKE_SHARED_LINKER_FLAGS='-fsanitize=address'
cmake --build "$ASAN_DIR" -j4
gcc "$SCRIPT_DIR/backend_safety_test.c" \
  -I"$SOURCE_DIR/apriltags-source" -L"$ASAN_DIR/lib" \
  -Wl,-rpath,"$ASAN_DIR/lib" -lapriltag \
  -fsanitize=address -fno-omit-frame-pointer \
  -o "$ASAN_DIR/backend_safety_test"
ASAN_OPTIONS=detect_leaks=1 "$ASAN_DIR/backend_safety_test"
