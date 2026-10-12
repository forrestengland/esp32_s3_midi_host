#!/usr/bin/env bash
set -euo pipefail

SKETCH_DIR="${1:-$HOME}/programming_2026/esp32_s3_midi_host/"
FQBN="esp32:esp32:esp32s3"
BUILD_DIR="$SKETCH_DIR/build"

if [[ ! -f "$SKETCH_DIR/$(basename "$SKETCH_DIR").ino" ]]; then
    echo "Error: Sketch file not found in $SKETCH_DIR"
    exit 1
fi

arduino-cli compile \
	    -v \
    --fqbn "$FQBN" \
    --build-path "$BUILD_DIR" \
    "$SKETCH_DIR"

echo "Compile successful."
