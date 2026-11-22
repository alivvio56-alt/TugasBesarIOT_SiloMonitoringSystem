#!/usr/bin/env bash
set -e
BASE="$HOME/Silo_ESP_RASPI"
mkdir -p "$BASE/logs" "$BASE/backend/data/snapshots"

echo "[1/3] Cek MQTT broker Mosquitto..."
systemctl is-active --quiet mosquitto || sudo systemctl start mosquitto

echo "[2/3] Jalankan Go API + SQLite (port 3000)..."
pkill -f "silo-api-arm64" 2>/dev/null || true
sleep 0.5
(cd "$BASE/backend" && setsid -f ./silo-api-arm64 </dev/null > "$BASE/logs/backend.log" 2>&1 &)
sleep 2

echo "[3/3] Jalankan Laravel Dashboard (port 8000)..."
pkill -f "artisan serve" 2>/dev/null || true
sleep 0.5
(cd "$BASE/frontend" && setsid -f php artisan serve --host=0.0.0.0 --port=8000 </dev/null > "$BASE/logs/frontend.log" 2>&1 &)
sleep 2

IP=$(hostname -I | awk '{print $1}')
echo "=================================================="
echo "  DASHBOARD WEB : http://${IP}:8000"
echo "  TRACKING BOX  : http://${IP}:8000/tracking"
echo "  API GO        : http://${IP}:3000/api/health"
echo "  MQTT BROKER   : tcp://${IP}:1883"
echo "  SQLITE DB     : $BASE/backend/data/silo.db"
echo "=================================================="
