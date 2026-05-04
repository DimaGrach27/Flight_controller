#!/usr/bin/env bash
set -e

echo "[gui] Starting Gazebo GUI"

#SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
#
#GUI_CONFIG="$PROJECT_DIR/gui/flight_controler_debug.gui"
#
#if [ ! -f "$GUI_CONFIG" ]; then
#    echo "[server] GUI Config file not found: $GUI_CONFIG"
#    exit 1
#fi
#
#echo "[server] GUI:       $GUI_CONFIG"
gz sim -g -v 4
#gz sim -g -v 4 --gui-config "$GUI_CONFIG"