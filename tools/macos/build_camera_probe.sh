#!/bin/bash
# Build only; never opens a camera. Keep binary and raw data outside Git.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
output=${1:?usage: build_camera_probe.sh OUTPUT_BINARY}
# Use one Xcode installation for both compiler and SDK to avoid mixed toolchains.
developer_dir=${DEVELOPER_DIR:-$(xcode-select -p)}
sdk=$(DEVELOPER_DIR="$developer_dir" xcrun --sdk macosx --show-sdk-path)
DEVELOPER_DIR="$developer_dir" xcrun --sdk macosx clang -isysroot "$sdk" \
  -fobjc-arc -O2 -Wall -Wextra -Wno-unused-parameter \
  -framework Foundation -framework AVFoundation -framework CoreMedia -framework CoreVideo \
  -sectcreate __TEXT __info_plist "$root/tools/macos/CameraProbe-Info.plist" \
  "$root/tools/mac_camera_capture.m" -o "$output"
