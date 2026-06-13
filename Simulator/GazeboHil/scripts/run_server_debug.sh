#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build/debug"
WORLD_FILE="$PROJECT_DIR/worlds/baylands.sdf"
#WORLD_FILE="$PROJECT_DIR/worlds/quadcopter_hil.sdf"

PLUGIN_DIR="$BUILD_DIR/plugins"
MODELS_DIR="$PROJECT_DIR/models"

if [ ! -d "$PLUGIN_DIR" ]; then
    echo "[server] Plugin dir not found: $PLUGIN_DIR"
    echo "[server] Run scripts/build.sh first"
    exit 1
fi

if [ ! -d "$MODELS_DIR" ]; then
    echo "[server] Models dir not found: $MODELS_DIR"
    exit 1
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "[server] World file not found: $WORLD_FILE"
    exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$MODELS_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

echo "[server] Plugin path:   $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "[server] Resource path: $GZ_SIM_RESOURCE_PATH"
echo "[server] World:         $WORLD_FILE"

gz sim -s -r -v 4 "$WORLD_FILE"
