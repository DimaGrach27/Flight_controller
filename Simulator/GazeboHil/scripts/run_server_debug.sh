#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build/debug"
WORLD_FILE="$PROJECT_DIR/worlds/one_axis_hil.sdf"

PLUGIN_DIR="$BUILD_DIR/plugins"

if [ ! -d "$PLUGIN_DIR" ]; then
    echo "[server] Plugin dir not found: $PLUGIN_DIR"
    echo "[server] Run scripts/build.sh first"
    exit 1
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "[server] World file not found: $WORLD_FILE"
    exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_DIR:$GZ_SIM_SYSTEM_PLUGIN_PATH"

echo "[server] Plugin path: $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "[server] World:       $WORLD_FILE"

gz sim -s -v 4 "$WORLD_FILE"
#add -r to run on start