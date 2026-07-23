#!/bin/bash
set -e

# ---- CONFIG ----
FQBN="arduino:renesas_uno:unor4wifi"
BASE_DIR="/home/haptic/kinova_haptic_ws/arduino"
PORT=""   # leave empty to auto-detect
# -----------------

echo "== Arduino flash script =="

# Find all valid sketch folders (folder containing a matching .ino file)
mapfile -t SKETCHES < <(find "$BASE_DIR" -mindepth 1 -maxdepth 2 -type f -name "*.ino" | while read -r ino; do
    dir=$(dirname "$ino")
    name=$(basename "$dir")
    if [ -f "$dir/$name.ino" ]; then
        echo "$name"
    fi
done)

if [ ${#SKETCHES[@]} -eq 0 ]; then
    echo "No valid sketches found under $BASE_DIR"
    exit 1
fi

# Determine which sketch to flash
if [ -n "$1" ]; then
    SKETCH_NAME="$1"
else
    echo "Available sketches:"
    select choice in "${SKETCHES[@]}"; do
        if [ -n "$choice" ]; then
            SKETCH_NAME="$choice"
            break
        else
            echo "Invalid selection, try again."
        fi
    done
fi

SKETCH_DIR="$BASE_DIR/$SKETCH_NAME"

if [ ! -d "$SKETCH_DIR" ] || [ ! -f "$SKETCH_DIR/$SKETCH_NAME.ino" ]; then
    echo "Sketch '$SKETCH_NAME' not found (expected $SKETCH_DIR/$SKETCH_NAME.ino)"
    echo "Available sketches: ${SKETCHES[*]}"
    exit 1
fi

# Auto-detect port if not set
if [ -z "$PORT" ]; then
    PORT=$(arduino-cli board list | grep -E 'ttyACM|ttyUSB' | awk '{print $1}' | head -n1)
    if [ -z "$PORT" ]; then
        echo "No Arduino board detected. Check the USB connection."
        exit 1
    fi
    echo "Detected board on port: $PORT"
fi

echo "Compiling sketch: $SKETCH_DIR"
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"

echo "Uploading to $PORT"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"

echo "Done."