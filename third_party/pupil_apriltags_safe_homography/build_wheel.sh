#!/usr/bin/env bash
set -euo pipefail

# Reproducibly builds the exact wrapper/source revision audited for this AGV.
WRAPPER_COMMIT=f5334c6e007dc7256386e30e948d63fef5dbc264
APRILTAG_SOURCE_COMMIT=1a0d17fb4031d70fca303d81c494fad7cfdcf0d8
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BUILD_ROOT=${AGV_APRILTAG_BUILD_ROOT:-"$SCRIPT_DIR/build"}
DIST_DIR="$BUILD_ROOT/dist"

mkdir -p "$BUILD_ROOT" "$DIST_DIR"
SOURCE_DIR=$(mktemp -d "$BUILD_ROOT/source.XXXXXX")
git clone --no-checkout https://github.com/pupil-labs/apriltags.git "$SOURCE_DIR"
git -C "$SOURCE_DIR" checkout --detach "$WRAPPER_COMMIT"
git -C "$SOURCE_DIR" submodule update --init --recursive --force

actual_source=$(git -C "$SOURCE_DIR/apriltags-source" rev-parse HEAD)
if [[ "$actual_source" != "$APRILTAG_SOURCE_COMMIT" ]]; then
  echo "unexpected apriltags-source commit: $actual_source" >&2
  exit 1
fi
git -C "$SOURCE_DIR/apriltags-source" apply --check "$SCRIPT_DIR/safe_homography.patch"
git -C "$SOURCE_DIR/apriltags-source" apply "$SCRIPT_DIR/safe_homography.patch"

SETUPTOOLS_SCM_PRETEND_VERSION=1.0.4.post11+agvsafe1 \
  python3 -m pip wheel --no-deps --no-cache-dir \
  --wheel-dir "$DIST_DIR" "$SOURCE_DIR"

wheel=$(find "$DIST_DIR" -maxdepth 1 -type f -name '*agvsafe1*.whl' -print -quit)
if [[ -z "$wheel" ]]; then
  echo "patched wheel was not created" >&2
  exit 1
fi
sha256sum "$wheel"
echo "Built $wheel"
echo "Install (Ubuntu 24.04 user site): python3 -m pip install --user --break-system-packages --force-reinstall --no-deps '$wheel'"
