#!/usr/bin/env bash
pkill -f "raspi_qr_bridge.py" 2>/dev/null || true
pkill -f "silo-api-arm64" 2>/dev/null || true
pkill -f "artisan serve" 2>/dev/null || true
echo "[SILO] Semua service (bridge, API Go, Laravel) dihentikan."
