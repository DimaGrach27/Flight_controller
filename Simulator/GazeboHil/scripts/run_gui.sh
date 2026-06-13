#!/usr/bin/env bash
set -e

echo "[gui] Starting Gazebo GUI"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build/debug"

GUI_CONFIG="$PROJECT_DIR/gui/flight_controler_debug.gui"
PLUGIN_DIR="$BUILD_DIR/plugins"

if [ ! -f "$GUI_CONFIG" ]; then
    echo "[gui] GUI config file not found: $GUI_CONFIG"
    exit 1
fi

if [ ! -d "$PLUGIN_DIR" ]; then
    echo "[gui] Plugin dir not found: $PLUGIN_DIR"
    echo "[gui] Run cmake --build $BUILD_DIR first"
    exit 1
fi

export GZ_GUI_PLUGIN_PATH="$PLUGIN_DIR${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"

echo "[gui] GUI: $GUI_CONFIG"
echo "[gui] Plugin path: $GZ_GUI_PLUGIN_PATH"
gz sim -g -v 4 --gui-config "$GUI_CONFIG"
