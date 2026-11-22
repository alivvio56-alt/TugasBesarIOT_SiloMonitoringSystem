#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
CAM_IDX="${1:-0}"
PORT="${2:-/dev/serial0}"
echo "[SILO] Starting raspi_qr_bridge.py on /dev/video${CAM_IDX} & ${PORT} (640x360, focus=255, MQTT=127.0.0.1:1883)..."
exec python3 -u raspi_qr_bridge.py --cam "$CAM_IDX" --width 640 --height 360 --port "$PORT" --focus -1 --mqtt-host 127.0.0.1 --mqtt-port 1883
