# language: Python, file: raspi_qr_bridge.py, target: Raspberry Pi 3 (CLI) & Windows
# dependencies (Raspi 3): sudo apt install -y python3-opencv python3-numpy python3-serial python3-pyzbar libzbar0
# dependencies (Windows): pip install opencv-python numpy pyserial

import argparse
import base64
import csv
import glob
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None

try:
    from pyzbar import pyzbar
except ImportError:
    pyzbar = None

try:
    import paho.mqtt.client as paho_mqtt
except ImportError:
    paho_mqtt = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
JSONL_LOG_PATH = os.path.join(SCRIPT_DIR, "scan_history.jsonl")
CSV_LOG_PATH = os.path.join(SCRIPT_DIR, "scan_history.csv")
BG_MODEL_PATH = os.path.join(SCRIPT_DIR, "conveyor_bg.npz")

# ROI default lebih lebar ke kiri-kanan agar barang lebih besar dari sabuk tetap tercakup penuh
DEFAULT_ROI = (0.33, 0.20, 0.67, 0.90)
ROI_FEAT_W, ROI_FEAT_H = 160, 220

CSV_HEADERS = [
    "timestamp", "scan_id", "status", "box_id", "lot", "variety",
    "process", "mc", "grade", "weight", "target_silo", "reason",
    "temp", "hum", "gas",
]

DEFAULT_TELEM = {
    "state": "OFFLINE",
    "scan_id": 0,
    "present": 0,
    "dist": -1.0,
    "pwm": 0,
    "temp": -1.0,
    "hum": -1.0,
    "gas": 0,
    "fan": 0,
    "vent": 0,
    "led_g": 0,
    "led_r": 0,
}

HTML_DASHBOARD = r"""<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#a78bfa">
<title>Live Kamera Vision · Silo SCM</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Bricolage+Grotesque:opsz,wght@12..96,600;700&family=Plus+Jakarta+Sans:wght@400;500;600;700&family=IBM+Plex+Mono:wght@400;500;600&display=swap" rel="stylesheet">
<style>
  :root {
    --ink:#3b2a5c; --muted:#8c7aa8; --line:#f0e7fb;
    --purple:#a78bfa; --purple-d:#7c3aed; --pink:#f9a8d4; --pink-d:#db2777;
    --card:rgba(255,255,255,.78); --shadow:0 14px 34px rgba(124,58,237,.10);
    color-scheme:light;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body {
    font-family:"Plus Jakarta Sans",system-ui,sans-serif; color:var(--ink); font-size:14px; min-height:100vh;
    background:
      radial-gradient(900px 600px at 0% 0%,#ede4ff 0,transparent 60%),
      radial-gradient(700px 520px at 100% 35%,#fde2f0 0,transparent 60%),
      radial-gradient(640px 480px at 40% 110%,#fef6cf 0,transparent 60%),
      #fbf7ff;
    padding:20px;
  }
  .wrap { max-width:1320px; margin:0 auto; display:flex; flex-direction:column; gap:20px; }
  .topbar { display:flex; justify-content:space-between; align-items:flex-end; flex-wrap:wrap; gap:14px; }
  .brand-row { display:flex; align-items:center; gap:12px; }
  .logo {
    width:44px; height:44px; border-radius:15px; display:grid; place-items:center;
    background:linear-gradient(135deg,#a78bfa,#f9a8d4 60%,#fde68a); color:#fff; flex:none;
    box-shadow:0 8px 18px rgba(167,139,250,.38); font-weight:800; font-size:18px;
  }
  .kicker { font-size:11px; letter-spacing:.08em; color:var(--muted); font-weight:700; text-transform:uppercase; }
  h1 { font-family:"Bricolage Grotesque",sans-serif; font-size:26px; letter-spacing:-.02em; line-height:1.15; margin-top:2px; }
  .top-tools { display:flex; align-items:center; gap:8px; flex-wrap:wrap; }
  .pill {
    display:inline-flex; align-items:center; gap:7px; padding:8px 14px; border-radius:999px;
    background:#fff; border:1.5px solid #dfd0fa; box-shadow:0 4px 12px rgba(124,58,237,.08); font-weight:700; font-size:12px; color:var(--ink);
  }
  .dot { width:9px; height:9px; border-radius:50%; background:#d8c8f0; }
  .dot.on { background:#10b981; box-shadow:0 0 0 4px rgba(16,185,129,.22); }
  .dot.off { background:#f43f5e; }
  .nav-btn {
    display:inline-flex; align-items:center; gap:6px; padding:9px 15px; border-radius:999px;
    background:#fff; border:1.5px solid #d5c1f7; color:var(--ink); font-size:13px; font-weight:700; text-decoration:none;
    box-shadow:0 4px 14px rgba(124,58,237,.10); transition:transform .15s, box-shadow .15s;
  }
  .nav-btn:hover { transform:translateY(-1px); border-color:#a78bfa; box-shadow:0 8px 20px rgba(124,58,237,.16); }
  .nav-btn.primary { background:linear-gradient(135deg,#c4b5fd,#f9a8d4 60%,#fde68a); border-color:#c084fc; }
  .grid { display:grid; grid-template-columns:minmax(320px,1.55fr) minmax(300px,1fr); gap:20px; align-items:start; }
  @media(max-width:940px){ .grid{ grid-template-columns:1fr; } body{ padding:14px; } h1{ font-size:22px; } }
  .card {
    background:rgba(255,255,255,.76); border:1.5px solid #e2d4fb; border-radius:24px;
    padding:20px; box-shadow:var(--shadow); backdrop-filter:blur(14px); min-width:0;
  }
  .card-head { display:flex; justify-content:space-between; align-items:center; gap:10px; margin-bottom:14px; }
  .card-head h2 { font-size:16px; margin-top:2px; }
  .video-frame { border-radius:0; overflow:hidden; background:#1c1230; border:2px solid #d5c1f7; box-shadow:0 10px 26px rgba(59,42,92,.16); }
  .video-frame img { width:100%; display:block; }
  .alert-box {
    display:none; margin-top:14px; padding:12px 16px; border-radius:16px;
    background:#ffe4e6; border:1.5px solid #fb7185; color:#9f1239; font-weight:700; font-size:13px;
  }
  .alert-box.show { display:block; }
  .qr-banner {
    margin-top:14px; padding:14px 16px; border-radius:18px; background:#f5effe;
    border:1.5px solid #d8c5fa; box-shadow:0 4px 12px rgba(124,58,237,.06);
  }
  .qr-banner.VALID { background:#dcfce7; border-color:#34d399; }
  .qr-banner.INVALID { background:#ffe4e6; border-color:#fb7185; }
  .kv { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
  .box { background:#f5effe; border:1.5px solid #dfd0fa; border-radius:16px; padding:12px 14px; }
  .lbl { font-size:10px; letter-spacing:.07em; color:#6d5696; font-weight:700; text-transform:uppercase; }
  .val { font-size:15px; font-weight:700; margin-top:4px; word-break:break-word; color:var(--ink); }
  .mono { font-family:"IBM Plex Mono",monospace; font-size:12px; }
  .leds { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:12px; }
  .led {
    text-align:center; padding:10px 12px; border-radius:14px; background:#f1eafc;
    border:1.5px solid #d8c7f7; color:#6e5a94; font-weight:700; font-size:12px; opacity:.85; transition:all .2s;
  }
  .led.g.on { background:#bbf7d0; border-color:#10b981; color:#065f46; opacity:1; box-shadow:0 6px 16px rgba(16,185,129,.25); }
  .led.r.on { background:#fecdd3; border-color:#f43f5e; color:#9f1239; opacity:1; box-shadow:0 6px 16px rgba(244,63,94,.25); }
  .btns { display:grid; grid-template-columns:1fr 1fr; gap:9px; margin-top:14px; }
  button {
    padding:10px 12px; border-radius:14px; border:1.5px solid #d4bff7; background:#f4edfe; color:var(--ink);
    font-family:inherit; font-size:12.5px; font-weight:700; cursor:pointer;
    box-shadow:0 3px 10px rgba(124,58,237,.08); transition:transform .12s, box-shadow .12s, background .12s, border-color .12s;
  }
  button:hover { transform:translateY(-1px); background:#ebe0fd; border-color:#a78bfa; box-shadow:0 6px 14px rgba(124,58,237,.15); }
  button:active { transform:translateY(0); }
  button.ok { background:#d1fae5; border-color:#34d399; color:#065f46; }
  button.ok:hover { background:#a7f3d0; border-color:#10b981; }
  button.bad { background:#ffe4e6; border-color:#fb7185; color:#9f1239; }
  button.bad:hover { background:#fecdd3; border-color:#f43f5e; }
  button.warn { background:linear-gradient(135deg,#c4b5fd,#f9a8d4 60%,#fde68a); border-color:#c084fc; color:var(--ink); }
  .tbl-wrap {
    overflow-x:auto; margin-top:16px; background:#f5effe; border:1.5px solid #dfd0fa;
    border-radius:16px; padding:10px 12px;
  }
  table { width:100%; border-collapse:separate; border-spacing:0 6px; font-size:12px; }
  th { font-size:10px; letter-spacing:.06em; text-transform:uppercase; color:#6d5696; text-align:left; padding:4px 10px 6px; font-weight:700; border-bottom:1px solid #dfd0fa; }
  td { background:#fff; padding:9px 10px; border-top:1px solid #e4d7fb; border-bottom:1px solid #e4d7fb; color:var(--ink); }
  td:first-child { border-left:1px solid #e4d7fb; border-radius:10px 0 0 10px; }
  td:last-child { border-right:1px solid #e4d7fb; border-radius:0 10px 10px 0; }
  .st-ok { display:inline-block; padding:3px 9px; border-radius:999px; background:#d1fae5; border:1px solid #6ee7b7; color:#065f46; font-weight:700; font-size:11px; }
  .st-bad { display:inline-block; padding:3px 9px; border-radius:999px; background:#ffe4e6; border:1px solid #fda4af; color:#9f1239; font-weight:700; font-size:11px; }
</style>
</head>
<body>
<div class="wrap">
  <header class="topbar">
    <div class="brand-row">
      <span class="logo">QR</span>
      <div>
        <span class="kicker">Silo 01 · Computer Vision &amp; ROI</span>
        <h1>Live Kamera Conveyor</h1>
      </div>
    </div>
    <div class="top-tools">
      <a id="link-dash" class="nav-btn primary" href="../"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/></svg>Dashboard</a>
      <span class="pill"><span id="uart-dot" class="dot"></span><span id="badge">Menghubungkan…</span></span>
      <span id="fps" class="pill mono">0.0 FPS</span>
    </div>
  </header>

  <div class="grid">
    <section class="card">
      <div class="card-head">
        <div><span class="kicker">Stream langsung</span><h2>Area Deteksi Conveyor &amp; QR</h2></div>
        <span class="pill" id="belt-pill">KOSONG</span>
      </div>
      <div class="video-frame">
        <img src="video_feed" alt="Camera Stream">
      </div>
      <div id="alertbox" class="alert-box"></div>
      <div id="qrbox" class="qr-banner">
        <div class="lbl">Status Area Conveyor &amp; QR</div>
        <div class="val" id="qrtitle">Conveyor kosong — siap memindai…</div>
        <div style="font-size:12.5px;color:var(--muted);margin-top:4px;" id="qrmeta">-</div>
      </div>
    </section>

    <aside class="card">
      <div class="card-head">
        <div><span class="kicker">Telemetri &amp; kalibrasi</span><h2>Status Hardware &amp; Lensa</h2></div>
      </div>
      <div class="kv">
        <div class="box"><div class="lbl">State ESP32</div><div class="val" id="state">-</div></div>
        <div class="box"><div class="lbl">Jarak / PWM</div><div class="val mono" id="dist">-</div></div>
        <div class="box"><div class="lbl">Deteksi Belt ROI</div><div class="val" id="belt">-</div></div>
        <div class="box"><div class="lbl">Valid / Invalid</div><div class="val" id="counts">0 / 0</div></div>
      </div>
      <div class="leds">
        <div id="ledg" class="led g">LED HIJAU · VALID</div>
        <div id="ledr" class="led r">LED MERAH · INVALID</div>
      </div>
      <div class="btns">
        <button class="warn" onclick="camCtrl('calibrate_bg')">Kalibrasi Belt Kosong</button>
        <button onclick="sendCmd('CONV_AUTO')">Reset AUTO</button>
        <button class="ok" onclick="sendCmd('SCAN_VALID')">Paksa VALID</button>
        <button class="bad" onclick="sendCmd('SCAN_INVALID')">Paksa INVALID</button>
        <button onclick="camCtrl('bright_up')">Kamera Terang +</button>
        <button onclick="camCtrl('bright_down')">Kamera Gelap -</button>
        <button onclick="camCtrl('focus_down')">Fokus Dekat -</button>
        <button onclick="camCtrl('focus_up')">Fokus Jauh +</button>
        <button onclick="camCtrl('focus_auto')">Fokus Auto</button>
        <button onclick="camCtrl('driver_panel')">Panel Driver (Win)</button>
      </div>
      <div class="tbl-wrap">
        <table>
          <thead><tr><th>Waktu</th><th>ID Box</th><th>MC</th><th>Status</th></tr></thead>
          <tbody id="hist"></tbody>
        </table>
      </div>
    </aside>
  </div>
</div>
<script>
(function () {
  const isCamPrefix = window.location.pathname.startsWith('/cam');
  const dashBase = isCamPrefix
    ? '/'
    : (window.location.port === '5000'
        ? (window.location.protocol + '//' + window.location.hostname + ':8000/')
        : '/');
  document.getElementById('link-dash').href = dashBase;
})();
async function sendCmd(cmd) {
  await fetch('api/cmd', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({cmd}) });
}
async function camCtrl(action) {
  await fetch('api/cam', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action}) });
}
async function poll() {
  try {
    const r = await fetch('api/status');
    const d = await r.json();
    const t = d.telemetry;
    document.getElementById('uart-dot').className = 'dot ' + (d.uart_connected ? 'on' : 'off');
    document.getElementById('badge').textContent = d.uart_connected ? 'ESP32 Online' : 'Kamera Saja';
    document.getElementById('fps').textContent = d.fps.toFixed(1) + ' FPS';
    document.getElementById('state').textContent = t.state + ' (#' + d.active_scan_id + ')';
    document.getElementById('dist').textContent = t.dist + ' cm · PWM ' + t.pwm;
    document.getElementById('belt').textContent = d.belt_status;
    document.getElementById('belt-pill').textContent = d.belt_status;
    document.getElementById('counts').textContent = d.valid_count + ' / ' + d.invalid_count;
    document.getElementById('ledg').className = 'led g' + (t.led_g ? ' on' : '');
    document.getElementById('ledr').className = 'led r' + (t.led_r ? ' on' : '');

    const ab = document.getElementById('alertbox');
    if (d.alert_msg) { ab.textContent = 'PERINGATAN: ' + d.alert_msg; ab.className = 'alert-box show'; }
    else { ab.textContent = ''; ab.className = 'alert-box'; }

    const qr = d.last_result;
    const qb = document.getElementById('qrbox');
    qb.className = 'qr-banner' + (qr && qr.status ? (' ' + qr.status) : '');
    if (qr && qr.box_id) {
      document.getElementById('qrtitle').textContent = '[' + qr.status + '] ' + qr.box_id + ' → ' + qr.target_silo;
      document.getElementById('qrmeta').textContent =
        'Lot: ' + qr.lot + ' · Var: ' + qr.variety + ' (' + qr.process + ') · MC: ' + qr.mc + '% · Grade: ' + qr.grade;
    } else if (qr && qr.status) {
      document.getElementById('qrtitle').textContent = '[' + qr.status + '] Barang Tidak Valid';
      document.getElementById('qrmeta').textContent = qr.reason || '-';
    } else {
      document.getElementById('qrtitle').textContent = d.belt_status;
      document.getElementById('qrmeta').textContent = 'Hanya area dalam kotak ROI sabuk konveyor yang dipindai.';
    }

    const tbody = document.getElementById('hist');
    const hist = d.history || [];
    tbody.innerHTML = hist.length ? hist.map(h =>
      '<tr><td class="mono">' + h.timestamp.slice(11,19) + '</td><td class="mono">' + (h.box_id || '-') + '</td><td>' + (h.mc || '-') + '</td><td><span class="' +
      (h.status==='VALID' ? 'st-ok' : 'st-bad') + '">' + h.status + '</span></td></tr>'
    ).join('') : '<tr><td colspan="4" style="text-align:center;color:var(--muted);">Belum ada riwayat scan sesi ini.</td></tr>';
  } catch (e) {
    document.getElementById('uart-dot').className = 'dot off';
    document.getElementById('badge').textContent = 'Terputus';
  }
}
setInterval(poll, 450);
poll();
</script>
</body>
</html>
"""


def parse_roi(roi_str):
    parts = [float(x.strip()) for x in roi_str.split(",")]
    if len(parts) != 4:
        raise ValueError("Format --roi harus 'x1,y1,x2,y2' (0.0..1.0)")
    x1, y1, x2, y2 = parts
    return (max(0.0, min(0.95, x1)), max(0.0, min(0.95, y1)), max(0.05, min(1.0, x2)), max(0.05, min(1.0, y2)))


def roi_to_pixels(shape, roi_norm):
    h, w = shape[:2]
    x1 = int(round(w * roi_norm[0]))
    y1 = int(round(h * roi_norm[1]))
    x2 = int(round(w * roi_norm[2]))
    y2 = int(round(h * roi_norm[3]))
    return x1, y1, max(x1 + 10, x2), max(y1 + 10, y2)


def measure_ambient_lab(frame, roi_norm):
    """Mengukur luminans (L) & white-balance (a, b) ruangan dari area kiri/kanan di LUAR ROI."""
    h, w = frame.shape[:2]
    y1 = max(0, int(h * roi_norm[1]))
    y2 = min(h, int(h * roi_norm[3]))
    lx2 = max(12, int(w * max(0.05, roi_norm[0] - 0.05)))
    rx1 = min(w - 12, int(w * min(0.95, roi_norm[2] + 0.05)))
    patches = []
    if lx2 > 8:
        patches.append(cv2.cvtColor(cv2.resize(frame[y1:y2, 4:lx2], (40, 60)), cv2.COLOR_BGR2LAB).astype(np.float32))
    if w - rx1 > 8:
        patches.append(cv2.cvtColor(cv2.resize(frame[y1:y2, rx1:w - 4], (40, 60)), cv2.COLOR_BGR2LAB).astype(np.float32))
    if not patches:
        return np.array([80.0, 128.0, 128.0], dtype=np.float32)
    merged = np.concatenate([p.reshape(-1, 3) for p in patches], axis=0)
    return np.median(merged, axis=0).astype(np.float32)


def measure_ambient_luma(frame, roi_norm):
    return float(max(10.0, measure_ambient_lab(frame, roi_norm)[0]))


def extract_roi_features(roi_bgr):
    """Ekstraksi fitur CIE-Lab (L, a, b), gradien Sobel, HSV (S, V), dan peta anyaman silau sabuk."""
    small = cv2.resize(roi_bgr, (ROI_FEAT_W, ROI_FEAT_H), interpolation=cv2.INTER_AREA)
    gray_raw = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
    # Detektor anyaman silau sabuk (alternating ridge-valley 2D):
    # Sabuk hitam mengilap saat kena senter menghasilkan puncak terang (>+8) & lembah gelap (<-8)
    # berselang-seling rapat di jendela 13x13 (weave_alt ~0.25..0.44), sedangkan tepi benda solid
    # (TWS hitam / kardus putih) hanya transisi 1D sehingga weave_alt < 0.15!
    hp = gray_raw - cv2.GaussianBlur(gray_raw, (7, 7), 0)
    pos_f = cv2.boxFilter((hp > 8.0).astype(np.float32), -1, (13, 13))
    neg_f = cv2.boxFilter((hp < -8.0).astype(np.float32), -1, (13, 13))
    weave_alt = np.minimum(pos_f, neg_f)

    blur_bgr = cv2.GaussianBlur(small, (5, 5), 0)
    lab = cv2.cvtColor(blur_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv = cv2.cvtColor(blur_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    L = lab[:, :, 0]
    gx = cv2.Sobel(L, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(L, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)
    return small, lab, L, mag, hsv, weave_alt


class ConveyorROIDetector:
    """
    Deteksi barang khusus di area konveyor (ROI) tahan senter / perubahan cahaya ruangan:
    - Kompensasi eksposur multiplikatif & white-balance diukur dari area luar ROI.
    - Menolak silau spekular (glare) tekstur anyaman sabuk hitam saat disorot senter / lampu terang
      memakai peta tekstur mikro frekuensi tinggi (weave_tex) khusus piksel netral.
    - Mengabaikan celah/rel samping konveyor & roller atas untuk deteksi blob, sambil tetap
      mempertahankan kotak ROI scan yang lebar agar barang besar & QR tetap terbaca penuh.
    - Filter tangan dual-space (YCrCb + HSV Hue kulit) + syarat menyentuh tepi ROI.
    """

    def __init__(self, roi_norm=DEFAULT_ROI, auto_load_saved=False):
        self.roi_norm = roi_norm
        self.bg_lab = None
        self.bg_L = None
        self.bg_mag = None
        self.bg_sat = None
        self.bg_amb_lab = np.array([80.0, 128.0, 128.0], dtype=np.float32)
        self.prev_L = None
        self.smooth_bbox = None
        if auto_load_saved:
            self._load_initial_bg()

    def has_bg(self):
        return self.bg_lab is not None

    def _load_initial_bg(self):
        if os.path.isfile(BG_MODEL_PATH):
            try:
                data = np.load(BG_MODEL_PATH)
                saved_roi = data["roi_norm"] if "roi_norm" in data else None
                roi_match = (saved_roi is not None and len(saved_roi) == 4 and np.allclose(saved_roi, self.roi_norm, atol=1e-3))
                if "source" in data and str(data["source"]) == "live_lab_v6" and roi_match:
                    self.bg_lab = data["bg_lab"]
                    self.bg_L = data["bg_L"]
                    self.bg_mag = data["bg_mag"]
                    self.bg_sat = data["bg_sat"]
                    self.bg_amb_lab = data["bg_amb_lab"]
                    print(f"[ROI] Background Lab+HSV belt kosong dimuat dari {os.path.basename(BG_MODEL_PATH)}")
            except Exception:
                pass

    def calibrate_from_frames(self, frames, save=True):
        labs, Ls, mags, sats, ambs = [], [], [], [], []
        for f in frames:
            ambs.append(measure_ambient_lab(f, self.roi_norm))
            x1, y1, x2, y2 = roi_to_pixels(f.shape, self.roi_norm)
            _, lab, L, m, hsv, _ = extract_roi_features(f[y1:y2, x1:x2])
            labs.append(lab)
            Ls.append(L)
            mags.append(m)
            sats.append(hsv[:, :, 1])
        self.bg_amb_lab = np.median(ambs, axis=0).astype(np.float32)
        self.bg_lab = np.median(labs, axis=0).astype(np.float32)
        self.bg_L = np.median(Ls, axis=0).astype(np.float32)
        self.bg_mag = np.median(mags, axis=0).astype(np.float32)
        self.bg_sat = np.median(sats, axis=0).astype(np.float32)
        self.smooth_bbox = None
        if save:
            np.savez(
                BG_MODEL_PATH,
                source="live_lab_v6",
                roi_norm=np.array(self.roi_norm, dtype=np.float32),
                bg_amb_lab=self.bg_amb_lab,
                bg_lab=self.bg_lab,
                bg_L=self.bg_L,
                bg_mag=self.bg_mag,
                bg_sat=self.bg_sat,
            )

    def analyze(self, frame, has_qr_in_roi=False, allow_adapt=True):
        amb_lab = measure_ambient_lab(frame, self.roi_norm)
        amb_now = float(max(10.0, amb_lab[0]))
        x1, y1, x2, y2 = roi_to_pixels(frame.shape, self.roi_norm)
        roi_crop = frame[y1:y2, x1:x2]
        small, lab, L, mag, hsv, weave_alt = extract_roi_features(roi_crop)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        if self.bg_lab is None:
            self.bg_amb_lab = amb_lab.copy()
            self.bg_lab, self.bg_L, self.bg_mag, self.bg_sat = lab.copy(), L.copy(), mag.copy(), sat.copy()

        # 1. Hitung gerak antar-frame (dinormalisasi terhadap pergeseran eksposur global)
        motion = 0.0
        if self.prev_L is not None:
            diff_frame = L - self.prev_L
            motion = float(np.mean(np.abs(diff_frame - np.median(diff_frame))))
        self.prev_L = L

        # 2. Kompensasi eksposur multiplikatif berbasis persentil-35 sabuk inti (tahan lampu/lantai luar ROI
        #    yang memantul non-linear & tahan auto-exposure webcam), + kompensasi white-balance dari luar ROI!
        core_L = L[14:-8, 30:-30]
        core_bgL = self.bg_L[14:-8, 30:-30]
        k_L = float(np.clip(max(15.0, float(np.percentile(core_bgL, 35))) / max(15.0, float(np.percentile(core_L, 35))), 0.55, 1.65))
        L_adj = L * k_L
        dL_pos = L_adj - self.bg_L
        dL_neg = self.bg_L - L_adj
        da = (lab[:, :, 1] - (amb_lab[1] - self.bg_amb_lab[1])) - self.bg_lab[:, :, 1]
        db = (lab[:, :, 2] - (amb_lab[2] - self.bg_amb_lab[2])) - self.bg_lab[:, :, 2]
        dC = np.sqrt(da * da + db * db)
        # dE satu arah (mag baru LEBIH kuat dari background): jika lampu dimatikan sehingga sabuk
        # jadi lebih halus daripada saat terang, mag turun (< bg_mag) -> dE <= 0, TIDAK false positive!
        dE = (mag * k_L) - self.bg_mag
        dS = sat - self.bg_sat

        # 2b. Deteksi silau spekular / pantulan senter pada anyaman sabuk hitam:
        #     PENTING: Masker anti-glare HANYA aktif saat sabuk benar-benar tersorot senter/lampu keras
        #     (>=20% luas sabuk berkilau anyaman -> roi_glare_p80 >= 0.21; kondisi ruang normal/HC-SR04 <= 0.14!).
        #     Dengan gate ini, benda bertekstur kasa/logam seperti sensor HC-SR04 atau PCB TIDAK pernah
        #     terhapus/kedip-kedip di pencahayaan normal, sementara senter tetap 100% diblokir!
        neutral_pix = (dC < 7.5) & ((sat < 34.0) | (val < 42.0))
        roi_glare_p80 = float(np.percentile(weave_alt[14:-8, 30:-30], 80))
        roi_glare_p90 = float(np.percentile(weave_alt[14:-8, 30:-30], 90))
        scene_has_glare = (not has_qr_in_roi) and (roi_glare_p80 >= 0.21)
        if scene_has_glare:
            woven_glare = neutral_pix & (weave_alt > 0.21)
            woven_glare_u8 = cv2.dilate(woven_glare.astype(np.uint8) * 255, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
            is_glare = (woven_glare_u8 > 0) & neutral_pix
        else:
            is_glare = np.zeros_like(neutral_pix, dtype=bool)

        # Sabuk sudah hitam; tidak ada benda yang 38 unit lebih gelap dari sabuk hitam.
        # dL_neg sengaja tidak dipakai agar background yang pernah dikalibrasi terang/senter tidak pernah
        # membuat sabuk gelap kosong meluber masuk ke kotak benda saat lampu dimatikan!
        luma_edge_fg = ((dL_pos > 22.0) | (dE > 15.5)) & (~is_glare)
        dark_color_fg = (dS > 24.0) & (val >= 30.0) & (dC > 4.2)
        color_fg = (dC > 8.5) | dark_color_fg

        fg_mask = (luma_edge_fg | color_fg).astype(np.uint8) * 255

        # Abaikan celah/rel samping konveyor (30 px kiri-kanan = tepat batas tepi sabuk hitam),
        # roller atas (14 px), & bibir bawah (8 px), sementara ROI scan & QR tetap lebar.
        fg_mask[:, :30] = 0
        fg_mask[:, -30:] = 0
        fg_mask[:14, :] = 0
        fg_mask[-8:, :] = 0

        k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, k_open)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, k_close)

        fg_pixels = int(np.count_nonzero(fg_mask))

        # 3. Gabungkan kontur berdekatan milik benda yang sama (agar benda gelap terbingkai penuh,
        #    tetapi kilau terpisah jauh di ujung bawah belt tidak ikut disambung ke box!)
        bbox_frame = None
        largest_pct = 0.0
        valid_blob = False
        touches_border = False
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cand = []
        for c in contours:
            a = cv2.contourArea(c)
            if a < 35:
                continue
            cx, cy, cw, ch = cv2.boundingRect(c)
            # Tolak garis tipis vertikal/horizontal lipatan sabuk / kilau sempit
            if (cw < 16 and ch > cw * 3.5) or (ch < 12 and cw > ch * 4.0):
                continue
            # Saat senter/lampu keras menyala, buang fragmen bintik silau netral SEBELUM merge kontur
            # agar kotak barang (mis. kubus hijau) tetap rapat seukuran barang dan tidak tersambung ke silau sabuk!
            if scene_has_glare and not has_qr_in_roi:
                sub_c = fg_mask[cy:cy + ch, cx:cx + cw] > 0
                if np.any(sub_c):
                    c_dC = float(np.mean(dC[cy:cy + ch, cx:cx + cw][sub_c]))
                    c_L50 = float(np.percentile(L[cy:cy + ch, cx:cx + cw][sub_c], 50))
                    c_alt = float(np.median(weave_alt[cy:cy + ch, cx:cx + cw][sub_c]))
                    if c_dC < 7.0 and not (c_L50 > 130.0 and c_alt < 0.05):
                        continue
            cand.append((a, cx, cy, cx + cw, cy + ch))

        if cand:
            cand.sort(reverse=True)
            tot_a, ux1, uy1, ux2, uy2 = cand[0]
            # Iteratif gabungkan fragmen benda yang berjarak <= 28 px (tapi tidak sambung glare jauh)
            merged_any = True
            used = {0}
            while merged_any:
                merged_any = False
                for idx in range(1, len(cand)):
                    if idx in used:
                        continue
                    a, gx1, gy1, gx2, gy2 = cand[idx]
                    if gx1 <= ux2 + 28 and gx2 >= ux1 - 28 and gy1 <= uy2 + 28 and gy2 >= uy1 - 28:
                        ux1, uy1 = min(ux1, gx1), min(uy1, gy1)
                        ux2, uy2 = max(ux2, gx2), max(uy2, gy2)
                        tot_a += a
                        used.add(idx)
                        merged_any = True

            bw, bh = ux2 - ux1, uy2 - uy1
            largest_pct = (tot_a / float(ROI_FEAT_W * ROI_FEAT_H)) * 100.0
            min_w = int(ROI_FEAT_W * 0.18)
            min_h = int(ROI_FEAT_H * 0.12)

            # Verifikasi anti-glare tingkat blob: jika seluruh blob netral (kroma Lab dC rendah)
            # berada di dalam/dikelilingi anyaman sabuk silau, atau saat sabuk sedang disorot senter
            # (roi_glare_p90 > 0.24) dan blob bukan benda solid terang mulus (seperti kardus putih),
            # tolak sebagai pantulan lampu / lipatan sabuk!
            is_blob_glare = False
            is_flat_empty_recovery = False
            if largest_pct >= 1.2 and bw >= min_w and bh >= min_h and not has_qr_in_roi:
                sub_fg = fg_mask[uy1:uy2, ux1:ux2] > 0
                if np.any(sub_fg):
                    sub_L = L[uy1:uy2, ux1:ux2][sub_fg]
                    sub_mag = mag[uy1:uy2, ux1:ux2][sub_fg]
                    blob_dC = float(np.mean(dC[uy1:uy2, ux1:ux2][sub_fg]))
                    blob_L_p25 = float(np.percentile(sub_L, 25))
                    blob_L_p50 = float(np.percentile(sub_L, 50))
                    blob_L_p10 = float(np.percentile(sub_L, 10))
                    blob_L_p90 = float(np.percentile(sub_L, 90))
                    blob_mag_p90 = float(np.percentile(sub_mag, 90))
                    blob_alt = float(np.mean(weave_alt[uy1:uy2, ux1:ux2][sub_fg]))
                    blob_alt_med = float(np.median(weave_alt[uy1:uy2, ux1:ux2][sub_fg]))
                    blob_dLpos_p90 = float(np.percentile(dL_pos[uy1:uy2, ux1:ux2][sub_fg], 90))
                    blob_dE_p90 = float(np.percentile(dE[uy1:uy2, ux1:ux2][sub_fg], 90))
                    hy1, hy2 = max(14, uy1 - 14), min(ROI_FEAT_H - 8, uy2 + 14)
                    hx1, hx2 = max(30, ux1 - 14), min(ROI_FEAT_W - 30, ux2 + 14)
                    halo_alt = float(np.percentile(weave_alt[hy1:hy2, hx1:hx2], 90)) if hy2 > hy1 and hx2 > hx1 else 0.0
                    is_smooth_bright_solid = (blob_L_p50 > 130.0 and blob_alt_med < 0.05)
                    # Jika seluruh blob di frame mentah datar gelap polos tanpa tepi/warna (mis. lampu terang baru dimatikan)
                    is_flat_dark_belt = (
                        blob_dC < 5.0 and blob_L_p50 < 68.0
                        and (blob_L_p90 - blob_L_p10) < 15.0 and blob_mag_p90 < 15.0
                    )
                    is_pure_darkening = (blob_dC < 5.0 and blob_dLpos_p90 < 8.0 and blob_dE_p90 < 10.0)
                    if is_flat_dark_belt or is_pure_darkening:
                        is_blob_glare = True
                        is_flat_empty_recovery = is_flat_dark_belt
                    elif scene_has_glare and blob_dC < 6.8 and not is_smooth_bright_solid:
                        if blob_alt > 0.12 or halo_alt > 0.24 or roi_glare_p90 > 0.24:
                            is_blob_glare = True

            if largest_pct >= 1.2 and bw >= min_w and bh >= min_h and not is_blob_glare:
                valid_blob = True
                # Tahap 2 (Hysteresis Expansion): setelah objek terkonfirmasi oleh strong seed,
                # perluas ke bagian gelap/redup benda yang menyambung langsung (mis. sisi kiri matte
                # casing TWS hitam). Dimatikan saat senter keras (scene_has_glare) agar kotak barang
                # tidak ikut merambat ke lipatan sabuk yang berkilau!
                if not has_qr_in_roi and not scene_has_glare:
                    seed_mask = np.zeros_like(fg_mask)
                    seed_mask[uy1:uy2, ux1:ux2] = fg_mask[uy1:uy2, ux1:ux2]
                    weak_fg = ((((dL_pos > 16.0) | (dE > 9.0)) & (~is_glare)) | (dC > 5.5)).astype(np.uint8) * 255
                    weak_fg[:, :30] = 0
                    weak_fg[:, -30:] = 0
                    weak_fg[:14, :] = 0
                    weak_fg[-8:, :] = 0
                    weak_fg = cv2.morphologyEx(weak_fg, cv2.MORPH_OPEN, k_open)
                    weak_fg = cv2.morphologyEx(weak_fg, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)))
                    num_wl, w_labels = cv2.connectedComponents(weak_fg)
                    grown = seed_mask.copy()
                    for wlbl in range(1, num_wl):
                        comp = (w_labels == wlbl)
                        if np.any(comp & (seed_mask > 0)):
                            grown[comp] = 255
                    g_ys, g_xs = np.where(grown > 0)
                    if len(g_xs) > 0:
                        ux1, uy1 = int(g_xs.min()), int(g_ys.min())
                        ux2, uy2 = int(g_xs.max()) + 1, int(g_ys.max()) + 1
                        largest_pct = (len(g_xs) / float(ROI_FEAT_W * ROI_FEAT_H)) * 100.0
                        fg_mask = grown
                        fg_pixels = int(len(g_xs))

                touches_border = (
                    ux1 <= 34 or ux2 >= ROI_FEAT_W - 34
                    or uy1 <= 18 or uy2 >= ROI_FEAT_H - 12
                )
                sx = (x2 - x1) / float(ROI_FEAT_W)
                sy = (y2 - y1) / float(ROI_FEAT_H)
                raw_bbox = np.array([
                    x1 + ux1 * sx,
                    y1 + uy1 * sy,
                    x1 + ux2 * sx,
                    y1 + uy2 * sy,
                ], dtype=np.float32)
                if self.smooth_bbox is None:
                    self.smooth_bbox = raw_bbox
                else:
                    delta = np.abs(raw_bbox - self.smooth_bbox)
                    if float(np.max(delta)) >= 8.0:
                        self.smooth_bbox = 0.72 * self.smooth_bbox + 0.28 * raw_bbox
                bbox_frame = tuple(int(round(v)) for v in self.smooth_bbox)

        if not valid_blob and not has_qr_in_roi:
            self.smooth_bbox = None
            # PENTING: JANGAN pernah adaptasi background saat sabuk sedang kena silau senter (roi_glare_p90 >= 0.18
            # atau is_glare >= 2%)! Jika silau senter terserap ke bg_L/bg_mag, nanti saat senter dimatikan
            # sabuk kosong normal malah dikira barang!
            # Sebaliknya, jika sabuk datar gelap kosong (is_flat_empty_recovery), izinkan pemulihan cepat.
            glare_active = (roi_glare_p90 >= 0.18) or (float(np.mean(is_glare)) >= 0.02)
            can_adapt_bg = (largest_pct < 0.5) or (cand and is_flat_empty_recovery)
            if allow_adapt and motion < 1.8 and can_adapt_bg and not glare_active:
                a = 0.08 if (cand and is_flat_empty_recovery) else 0.04
                self.bg_lab = (1.0 - a) * self.bg_lab + a * lab
                self.bg_L = (1.0 - a) * self.bg_L + a * L
                self.bg_mag = (1.0 - a) * self.bg_mag + a * mag
                self.bg_sat = (1.0 - a) * self.bg_sat + a * sat
                self.bg_amb_lab = (1.0 - a) * self.bg_amb_lab + a * amb_lab
            return {
                "status": "EMPTY",
                "occupied": False,
                "bbox_frame": None,
                "score_pct": 0.0,
                "skin_pct": 0.0,
                "motion": motion,
                "ambient_luma": amb_now,
            }

        # 4. Deteksi tangan (dual-space YCrCb + HSV Hue kulit + wajib masuk dari tepi ROI)
        #    Dikalibrasi untuk kulit di ruang redup maupun terang (Cr 131..173, Cb 92..130, S 16..150),
        #    tetap menolak barang kuning (H~20..38), merah pekat (S>150), putih/abu (S<16), dan benda di tengah belt!
        skin_pct = 0.0
        if fg_pixels > 0 and not has_qr_in_roi and touches_border:
            ycrcb = cv2.cvtColor(small, cv2.COLOR_BGR2YCrCb)
            ycrcb_skin = cv2.inRange(ycrcb, (35, 131, 92), (255, 173, 130))
            hsv_u8 = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
            h_ch, s_ch, v_ch = hsv_u8[:, :, 0], hsv_u8[:, :, 1], hsv_u8[:, :, 2]
            hsv_skin = (((h_ch <= 17) | (h_ch >= 168)) & (s_ch >= 16) & (s_ch <= 150) & (v_ch >= 42)).astype(np.uint8) * 255
            skin_mask = cv2.bitwise_and(ycrcb_skin, hsv_skin)
            skin_in_fg = cv2.bitwise_and(skin_mask, fg_mask)
            skin_pct = (np.count_nonzero(skin_in_fg) / float(fg_pixels)) * 100.0
            if skin_pct >= 28.0:
                return {
                    "status": "HAND",
                    "occupied": False,
                    "bbox_frame": bbox_frame,
                    "score_pct": largest_pct,
                    "skin_pct": skin_pct,
                    "motion": motion,
                    "ambient_luma": amb_now,
                }

        # 5. Cek gerak tinggi (tangan/benda sedang bergerak melintas ROI)
        if motion >= 5.5 and not has_qr_in_roi:
            return {
                "status": "MOVING",
                "occupied": False,
                "bbox_frame": bbox_frame,
                "score_pct": largest_pct,
                "skin_pct": skin_pct,
                "motion": motion,
                "ambient_luma": amb_now,
            }

        return {
            "status": "OBJECT",
            "occupied": True,
            "bbox_frame": bbox_frame,
            "score_pct": largest_pct,
            "skin_pct": skin_pct,
            "motion": motion,
            "ambient_luma": amb_now,
        }


def evaluate_qr_payload(raw_text, max_mc=12.5):
    """Parse JSON box kopi & tentukan VALID vs INVALID."""
    res = {
        "raw": raw_text,
        "box_id": "",
        "lot": "-",
        "variety": "-",
        "process": "-",
        "mc": 0.0,
        "grade": "-",
        "weight": "-",
        "target_silo": "-",
        "status": "INVALID",
        "valid": False,
        "reason": "QR bukan format JSON box silo",
    }
    try:
        obj = json.loads(raw_text)
    except json.JSONDecodeError:
        return res

    if not isinstance(obj, dict) or "id" not in obj:
        return res

    mc_val = float(obj.get("mc", 0.0))
    grd = str(obj.get("grd", "-"))
    silo = str(obj.get("silo", "-"))

    res.update({
        "box_id": str(obj.get("id", "")),
        "lot": str(obj.get("lot", "-")),
        "variety": str(obj.get("var", "-")),
        "process": str(obj.get("prc", "-")),
        "mc": mc_val,
        "grade": grd,
        "weight": str(obj.get("wt", "-")),
        "target_silo": silo,
    })

    reasons = []
    if "HOLD" in grd.upper():
        reasons.append(f"Grade={grd}")
    if "QUARANTINE" in silo.upper():
        reasons.append(f"Silo={silo}")
    if mc_val > max_mc:
        reasons.append(f"MC={mc_val}%>{max_mc}%")

    if reasons:
        res["status"] = "INVALID"
        res["valid"] = False
        res["reason"] = ", ".join(reasons)
    else:
        res["status"] = "VALID"
        res["valid"] = True
        res["reason"] = f"OK -> {silo}"
    return res


class QRScanner:
    """Scanner QR multi-engine tahan-crash yang HANYA memindai area ROI konveyor."""

    def __init__(self, max_mc=12.5):
        self.max_mc = max_mc
        self.cv_qr = cv2.QRCodeDetector()
        self.cv_aruco = cv2.QRCodeDetectorAruco() if hasattr(cv2, "QRCodeDetectorAruco") else None
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        eng = []
        if pyzbar is not None:
            eng.append("pyzbar")
        if self.cv_aruco is not None:
            eng.append("OpenCV-Aruco")
        eng.append("OpenCV-QR")
        print(f"[QR] Engine aktif: {' + '.join(eng)} (max_mc={max_mc}%)")

    def _safe_pts(self, pts, offset_x=0, offset_y=0):
        if pts is None:
            return None
        arr = np.asarray(pts, dtype=np.int32).reshape(-1, 2)
        if len(arr) < 3:
            return None
        arr[:, 0] += offset_x
        arr[:, 1] += offset_y
        return arr

    def _try_opencv(self, img, ox, oy):
        if self.cv_aruco is not None:
            try:
                txt, pts, _ = self.cv_aruco.detectAndDecode(img)
                if txt:
                    return [(txt, self._safe_pts(pts, ox, oy))]
            except cv2.error:
                pass
        try:
            ok, texts, points, _ = self.cv_qr.detectAndDecodeMulti(img)
            if ok and texts is not None:
                hits = []
                for idx, txt in enumerate(texts):
                    if txt:
                        p = points[idx] if points is not None and idx < len(points) else None
                        hits.append((txt, self._safe_pts(p, ox, oy)))
                if hits:
                    return hits
        except cv2.error:
            pass
        try:
            txt, pts, _ = self.cv_qr.detectAndDecode(img)
            if txt:
                return [(txt, self._safe_pts(pts, ox, oy))]
        except cv2.error:
            pass
        return []

    def _scan_gray(self, g, ox, oy, fast_only=False):
        if pyzbar is not None:
            try:
                hits = []
                for obj in pyzbar.decode(g):
                    txt = obj.data.decode("utf-8", errors="ignore").strip()
                    if txt:
                        pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=np.int32) if obj.polygon else None
                        hits.append((txt, self._safe_pts(pts, ox, oy)))
                if hits:
                    return hits
            except Exception:
                pass
            if fast_only:
                return []
        return self._try_opencv(g, ox, oy)

    def detect_in_roi(self, frame, roi_norm, fast_only=False):
        """
        Crop dengan margin ekstra 5% agar quiet-zone putih di tepi QR tidak terpotong,
        namun tetap memvalidasi titik tengah QR wajib berada di dalam ROI konveyor.
        """
        h, w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = roi_to_pixels(frame.shape, roi_norm)

        pad_x = int(w * 0.05)
        pad_y = int(h * 0.05)
        cx1 = max(0, rx1 - pad_x)
        cy1 = max(0, ry1 - pad_y)
        cx2 = min(w, rx2 + pad_x)
        cy2 = min(h, ry2 + pad_y)

        roi_frame = frame[cy1:cy2, cx1:cx2]
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

        # Pass 1: Grayscale asli (paling tajam & cepat untuk kertas QR putih)
        hits = self._scan_gray(gray, cx1, cy1, fast_only=fast_only)

        if not fast_only:
            # Pass 2: Sharpen ringan (mengatasi sedikit blur/fokus kamera webcam)
            if not hits:
                blur_g = cv2.GaussianBlur(gray, (0, 0), 1.5)
                sharp = cv2.addWeighted(gray, 1.6, blur_g, -0.6, 0)
                hits = self._scan_gray(sharp, cx1, cy1)

            # Pass 3: Adaptive Gaussian Threshold (tahan bayangan tidak merata pada stiker QR)
            if not hits:
                adap = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 7)
                hits = self._scan_gray(adap, cx1, cy1)

            # Pass 4: Otsu / CLAHE fallback
            if not hits:
                _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                hits = self._scan_gray(otsu, cx1, cy1)
            if not hits:
                hits = self._scan_gray(self.clahe.apply(gray), cx1, cy1)

        out = []
        for txt, pts in hits:
            # Pastikan pusat QR berada di dalam kotak ROI konveyor (QR di luar konveyor tetap ditolak!)
            if pts is not None and len(pts) >= 3:
                qx = float(np.mean(pts[:, 0]))
                qy = float(np.mean(pts[:, 1]))
                if not (rx1 <= qx <= rx2 and ry1 <= qy <= ry2):
                    continue
            info = evaluate_qr_payload(txt, max_mc=self.max_mc)
            lbl = f"{info['box_id']} | MC:{info['mc']}% [{info['status']}]" if info["box_id"] else "BARANG GAK VALID (QR ASING)"
            out.append({"pts": pts, "label": lbl, "info": info})
        return out


def save_scan_record(record):
    """Simpan hasil scan ke JSONL & CSV lokal (siap diteruskan ke MQTT dashboard nanti)."""
    with open(JSONL_LOG_PATH, "a", encoding="utf-8") as fj:
        fj.write(json.dumps(record, ensure_ascii=False) + "\n")

    write_header = not os.path.exists(CSV_LOG_PATH)
    with open(CSV_LOG_PATH, "a", newline="", encoding="utf-8") as fc:
        w = csv.DictWriter(fc, fieldnames=CSV_HEADERS)
        if write_header:
            w.writeheader()
        w.writerow({k: record.get(k, "") for k in CSV_HEADERS})


class MqttBridge:
    """Integrasi MQTT lokal ke backend Go + SQLite + dashboard Laravel (telemetri, scan+screenshot, kontrol)."""

    TOPIC_TELEM = "silo/scm/telemetry/data"
    TOPIC_SCAN = "silo/scm/scan"
    TOPIC_CONTROL = "silo/scm/telemetry/control"

    def __init__(self, state, host="127.0.0.1", port=1883, device_id="silo-01", enabled=True):
        self.state = state
        self.host = host
        self.port = int(port)
        self.device_id = device_id
        self.enabled = enabled and (paho_mqtt is not None)
        self.client = None
        self.connected = False
        self._last_telem_pub = 0.0
        self._manual_mode = False
        self._lock = threading.Lock()

        if enabled and paho_mqtt is None:
            print("[MQTT] paho-mqtt belum terpasang (sudo apt install -y python3-paho-mqtt) -> MQTT nonaktif.")

    def start(self):
        if not self.enabled:
            return
        try:
            cid = f"raspi-bridge-{int(time.time()) % 100000}"
            if hasattr(paho_mqtt, "CallbackAPIVersion"):
                c = paho_mqtt.Client(paho_mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
            else:
                c = paho_mqtt.Client(client_id=cid)
            c.on_connect = self._on_connect
            c.on_disconnect = self._on_disconnect
            c.on_message = self._on_message
            c.reconnect_delay_set(min_delay=2, max_delay=15)
            c.connect_async(self.host, self.port, keepalive=30)
            c.loop_start()
            self.client = c
            print(f"[MQTT] Menghubungkan ke broker tcp://{self.host}:{self.port} (device={self.device_id})")
        except Exception as exc:
            print(f"[MQTT] Gagal inisialisasi client: {exc}")

    def stop(self):
        if self.client is not None:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        ok = (reason_code == 0) or (getattr(reason_code, "is_failure", False) is False)
        self.connected = ok
        if ok:
            print(f"[MQTT] Terhubung ke tcp://{self.host}:{self.port} -> subscribe {self.TOPIC_CONTROL}")
            client.subscribe(self.TOPIC_CONTROL, qos=1)
        else:
            print(f"[MQTT] Gagal konek: {reason_code}")

    def _on_disconnect(self, client, userdata, *args):
        self.connected = False

    def _on_message(self, client, userdata, msg):
        try:
            cmd = json.loads(msg.payload.decode("utf-8", errors="ignore"))
        except Exception:
            return
        if not isinstance(cmd, dict):
            return
        print(f"[MQTT -> ESP] Kontrol dashboard: {cmd}")
        mode_req = str(cmd.get("mode", "")).upper() if "mode" in cmd else ""
        has_actuator = any(k in cmd for k in ("fan", "vent", "conveyor"))

        if mode_req == "AUTO":
            with self._lock:
                self._manual_mode = False
                self._last_telem_pub = 0.0
            self.state.send_uart_json({"cmd": "ALL_AUTO"})
            return

        if mode_req == "MANUAL" or has_actuator:
            with self._lock:
                self._manual_mode = True
                self._last_telem_pub = 0.0

        if mode_req == "MANUAL" and not has_actuator:
            # Kunci kondisi aktuator saat ini ke mode manual (kompatibel tanpa reflash ESP32 + kirim ALL_MANUAL)
            with self.state.lock:
                t_cur = dict(self.state.telemetry)
            self.state.send_uart_json({"cmd": "ALL_MANUAL"})
            self.state.send_uart_json({"cmd": "FAN_ON" if int(t_cur.get("fan", 0)) else "FAN_OFF"})
            self.state.send_uart_json({"cmd": "VENT_OPEN" if int(t_cur.get("vent", 0)) > 10 else "VENT_CLOSE"})
            self.state.send_uart_json({"cmd": "RUN_CONVEYOR" if int(t_cur.get("pwm", 0)) > 0 else "STOP_CONVEYOR"})
            self.publish_telemetry(t_cur, force=True)
            return

        if "fan" in cmd:
            v = str(cmd["fan"]).upper()
            if v in ("ON", "OFF"):
                self.state.send_uart_json({"cmd": f"FAN_{v}"})
        if "vent" in cmd:
            v = str(cmd["vent"]).upper()
            if v in ("OPEN", "CLOSE"):
                self.state.send_uart_json({"cmd": f"VENT_{v}"})
        if "conveyor" in cmd:
            v = str(cmd["conveyor"]).upper()
            if v == "START":
                self.state.send_uart_json({"cmd": "RUN_CONVEYOR"})
            elif v == "STOP":
                self.state.send_uart_json({"cmd": "STOP_CONVEYOR"})

    def publish_telemetry(self, telem, force=False):
        if not self.enabled or self.client is None or not self.connected:
            return
        now = time.time()
        with self._lock:
            if not force and (now - self._last_telem_pub < 2.0):
                return
            self._last_telem_pub = now
            manual_mode = self._manual_mode
        try:
            temp = float(telem.get("temp", -1.0))
            hum = float(telem.get("hum", -1.0))
            gas = float(telem.get("gas", 0.0))
            if temp < -20.0 or hum < 0.0:
                return
            st_raw = str(telem.get("state", "IDLE")).upper()
            pwm_now = int(telem.get("pwm", 0))
            conv_map = {
                "IDLE": "STOPPED",
                "APPROACH": "RUNNING",
                "WAIT_SCAN": "SCANNING",
                "VALID_HOLD": "STOPPED",
                "INVALID_HOLD": "ERROR",
                "RELEASE": "RUNNING",
            }
            conv_status = "RUNNING" if pwm_now > 0 else conv_map.get(st_raw, "STOPPED")
            esp_mode = str(telem.get("mode", "")).upper()
            ctrl_mode = esp_mode if esp_mode in ("AUTO", "MANUAL") else ("MANUAL" if manual_mode else "AUTO")
            payload = {
                "device_id": self.device_id,
                "temperature": round(temp, 2),
                "humidity": round(hum, 2),
                "gas_level": round(max(0.0, gas), 1),
                "exhaust_fan": bool(int(telem.get("fan", 0))),
                "vent_open": int(telem.get("vent", 0)) > 10,
                "buzzer_alarm": False,
                "control_mode": ctrl_mode,
                "conveyor_status": conv_status,
                "distance_cm": round(max(0.0, float(telem.get("dist", 0.0))), 1),
            }
            self.client.publish(self.TOPIC_TELEM, json.dumps(payload), qos=1)
        except Exception as exc:
            print(f"[MQTT] Gagal kirim telemetri: {exc}")

    def publish_valid_scan(self, info, frame_bgr=None, qr_pts=None):
        """Kirim JSON QR barang VALID + screenshot view kamera (base64 JPEG) ke MQTT -> SQLite."""
        if not info.get("valid"):
            return
        snap_b64 = ""
        if frame_bgr is not None:
            try:
                rh, rw = frame_bgr.shape[:2]
                dw, dh = (640, 360) if rw > 640 else (rw, rh)
                shot = cv2.resize(frame_bgr, (dw, dh), interpolation=cv2.INTER_AREA)
                sx, sy = dw / float(rw), dh / float(rh)
                if qr_pts is not None and len(qr_pts) >= 3:
                    spts = np.column_stack((qr_pts[:, 0] * sx, qr_pts[:, 1] * sy)).astype(np.int32)
                    cv2.polylines(shot, [spts.reshape((-1, 1, 2))], True, (0, 255, 100), 2)
                lbl = f"{info.get('box_id', 'VALID')} | MC:{info.get('mc', 0)}% -> {info.get('target_silo', '-')}"
                cv2.rectangle(shot, (0, 0), (dw, 28), (16, 42, 24), -1)
                cv2.putText(shot, f"{lbl}  [{datetime.now().strftime('%H:%M:%S')}]",
                            (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 255, 150), 1, cv2.LINE_AA)
                ok, buf = cv2.imencode(".jpg", shot, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
                if ok:
                    jpg_bytes = buf.tobytes()
                    snap_b64 = base64.b64encode(jpg_bytes).decode("ascii")
                    snap_dir = os.path.join(SCRIPT_DIR, "snapshots")
                    os.makedirs(snap_dir, exist_ok=True)
                    safe_id = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(info.get("box_id", "box")))
                    with open(os.path.join(snap_dir, f"{safe_id}_{int(time.time())}.jpg"), "wb") as fs:
                        fs.write(jpg_bytes)
            except Exception as exc:
                print(f"[MQTT] Gagal buat screenshot kamera: {exc}")

        if not self.enabled or self.client is None or not self.connected:
            return
        try:
            body = {}
            raw = info.get("raw", "")
            if raw:
                try:
                    parsed = json.loads(raw)
                    if isinstance(parsed, dict):
                        body.update(parsed)
                except Exception:
                    pass
            body.setdefault("id", info.get("box_id", ""))
            body["device_id"] = self.device_id
            body["batch_id"] = info.get("box_id", "")
            body["material"] = info.get("variety", "-")
            body["ok"] = True
            if snap_b64:
                body["snapshot_b64"] = snap_b64
            self.client.publish(self.TOPIC_SCAN, json.dumps(body, ensure_ascii=False), qos=1)
            print(f"[MQTT -> WEB] Terkirim VALID {body['batch_id']} (+screenshot {len(snap_b64)//1024} KB)")
        except Exception as exc:
            print(f"[MQTT] Gagal kirim scan valid: {exc}")


class BridgeState:
    def __init__(self, args, roi_norm):
        self.args = args
        self.roi_norm = roi_norm
        self.running = True
        self.lock = threading.Lock()
        self.frame_cond = threading.Condition(self.lock)

        self.latest_jpeg = None
        self.frame_seq = 0
        self.stream_clients = 0
        self.auto_boost = (args.boost < 0)
        self.soft_boost = 0 if args.boost < 0 else int(args.boost)
        self.calibrate_requested = False

        # Kontrol fokus manual/auto & panel driver kamera
        self.focus_cmd = None
        self.focus_val = int(args.focus)  # -1 = auto focus, 0..255 = manual focus locked
        self.sharpness = 0.0

        # Komunikasi thread kamera (30 FPS) <-> thread QR decoder (asinkron)
        self.qr_req_lock = threading.Lock()
        self.qr_req_frame = None
        self.qr_req_seq = 0
        self.qr_req_active = False
        self.qr_req_deep = False
        self.qr_res_lock = threading.Lock()
        self.qr_latest_dets = []
        self.qr_res_seq = 0

        self.telemetry = dict(DEFAULT_TELEM)
        self.uart_connected = False
        self.ser = None
        self.ser_lock = threading.Lock()

        self.active_scan_id = 0
        self.scan_phase = "IDLE"
        self.scan_started_at = 0.0
        self.cached_reply = None
        self.last_result = None
        self.alert_msg = ""
        self.belt_status = "KOSONG"
        self.valid_count = 0
        self.invalid_count = 0
        self.history = []
        self.fps = 0.0

    def send_uart_json(self, payload_dict):
        line = json.dumps(payload_dict, separators=(",", ":")) + "\n"
        with self.ser_lock:
            if self.ser is not None and self.ser.is_open:
                self.ser.write(line.encode("utf-8"))
                print(f"[RASPI -> ESP32] {line.strip()}")
                return True
        print(f"[SIMULASI -> ESP32] {line.strip()}")
        return False

    def trigger_scan_request(self, scan_id):
        with self.lock:
            if scan_id == self.active_scan_id and self.cached_reply is not None:
                reply = dict(self.cached_reply)
            elif scan_id == self.active_scan_id and self.scan_phase == "SCANNING":
                return
            else:
                self.active_scan_id = scan_id
                self.scan_phase = "SCANNING"
                self.scan_started_at = time.time()
                self.cached_reply = None
                self.alert_msg = ""
                print(f"[ESP32 -> RASPI] SCAN_REQ #{scan_id} diterima -> memindai ROI konveyor...")
                return
        self.send_uart_json(reply)

    def cancel_scan(self, scan_id):
        with self.lock:
            if scan_id == self.active_scan_id or self.scan_phase == "SCANNING":
                self.scan_phase = "IDLE"
                self.cached_reply = None
                self.alert_msg = ""
                print(f"[ESP32 -> RASPI] SCAN_CANCEL #{scan_id}")


def make_handler(state: BridgeState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = HTML_DASHBOARD.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/api/status":
                with state.lock:
                    payload = {
                        "uart_connected": state.uart_connected,
                        "telemetry": state.telemetry,
                        "active_scan_id": state.active_scan_id,
                        "scan_phase": state.scan_phase,
                        "belt_status": state.belt_status,
                        "last_result": state.last_result,
                        "alert_msg": state.alert_msg,
                        "valid_count": state.valid_count,
                        "invalid_count": state.invalid_count,
                        "history": state.history[:10],
                        "fps": state.fps,
                    }
                body = json.dumps(payload).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/video_feed":
                try:
                    import socket as _sock
                    self.connection.setsockopt(_sock.IPPROTO_TCP, _sock.TCP_NODELAY, 1)
                except Exception:
                    pass
                self.send_response(HTTPStatus.OK)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                with state.lock:
                    state.stream_clients += 1
                last_seq = -1
                try:
                    while state.running:
                        with state.frame_cond:
                            state.frame_cond.wait_for(
                                lambda: (not state.running) or (state.frame_seq != last_seq and state.latest_jpeg is not None),
                                timeout=1.0,
                            )
                            if not state.running:
                                break
                            if state.latest_jpeg is None or state.frame_seq == last_seq:
                                continue
                            jpg = state.latest_jpeg
                            last_seq = state.frame_seq
                        hdr = b"--FRAME\r\nContent-Type: image/jpeg\r\n" + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                        self.wfile.write(hdr + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    pass
                finally:
                    with state.lock:
                        state.stream_clients = max(0, state.stream_clients - 1)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length).decode("utf-8", errors="ignore")
            try:
                req = json.loads(raw)
                if self.path == "/api/cam":
                    act = req.get("action", "")
                    with state.lock:
                        if act == "bright_up":
                            state.auto_boost = False
                            state.soft_boost = min(120, state.soft_boost + 10)
                        elif act == "bright_down":
                            state.auto_boost = False
                            state.soft_boost = max(0, state.soft_boost - 10)
                        elif act == "calibrate_bg":
                            state.auto_boost = True
                            state.calibrate_requested = True
                        elif act in ("focus_up", "focus_down", "focus_auto", "driver_panel"):
                            state.focus_cmd = act
                        b = state.soft_boost
                        fv = state.focus_val
                    resp = {"ok": True, "boost": b, "focus": fv}
                elif self.path == "/api/cmd":
                    cmd = str(req.get("cmd", "")).strip().upper()
                    state.send_uart_json({"cmd": cmd})
                    resp = {"ok": True, "cmd": cmd}
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
            except Exception as exc:
                resp = {"ok": False, "err": str(exc)}
            body = json.dumps(resp).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def uart_worker(state: BridgeState, mqtt_bridge=None):
    if state.args.no_uart or serial is None:
        if serial is None and not state.args.no_uart:
            print("[WARN] pyserial belum terpasang -> jalan mode kamera saja.")
        return

    while state.running:
        try:
            with serial.Serial(state.args.port, state.args.baud, timeout=1.0) as ser:
                with state.ser_lock:
                    state.ser = ser
                with state.lock:
                    state.uart_connected = True
                print(f"[UART] Terhubung ke ESP32 di {state.args.port} ({state.args.baud} bps)")

                while state.running:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    evt = str(msg.get("evt", "")).upper()
                    if evt == "SCAN_REQ":
                        state.trigger_scan_request(int(msg.get("scan_id", 0)))
                    elif evt == "SCAN_CANCEL":
                        state.cancel_scan(int(msg.get("scan_id", 0)))
                    elif evt == "TELEM":
                        with state.lock:
                            state.telemetry.update(msg)
                            telem_snap = dict(state.telemetry)
                            if msg.get("state") in ("IDLE", "APPROACH") and state.scan_phase in ("DECIDED_VALID", "WAIT_REMOVAL"):
                                state.scan_phase = "IDLE"
                                state.alert_msg = ""
                        if mqtt_bridge is not None:
                            mqtt_bridge.publish_telemetry(telem_snap)
        except Exception as exc:
            with state.ser_lock:
                state.ser = None
            with state.lock:
                state.uart_connected = False
            print(f"[UART] Menunggu port {state.args.port} ({exc})...")
            time.sleep(2.0)


def qr_worker(state: BridgeState, scanner: QRScanner):
    """Thread khusus decode QR di latar belakang agar loop kamera & stream tetap tinggi (~25-30 FPS)."""
    last_processed_seq = -1
    while state.running:
        frame_to_scan = None
        seq = -1
        deep = False
        with state.qr_req_lock:
            if state.qr_req_active and state.qr_req_frame is not None and state.qr_req_seq != last_processed_seq:
                frame_to_scan = state.qr_req_frame
                seq = state.qr_req_seq
                deep = state.qr_req_deep

        if frame_to_scan is None:
            time.sleep(0.015)
            continue

        last_processed_seq = seq
        try:
            dets = scanner.detect_in_roi(frame_to_scan, state.roi_norm, fast_only=(not deep))
        except Exception as exc:
            print(f"[WARN] QR decode error: {exc}")
            dets = []

        with state.qr_res_lock:
            state.qr_latest_dets = dets
            state.qr_res_seq = seq


def apply_camera_focus(cap, cam_idx, focus_val):
    """Terapkan kunci fokus manual / auto di Windows (DirectShow) & Linux/Raspi (V4L2 + v4l2-ctl)."""
    if focus_val >= 0:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if sys.platform.startswith("win"):
            cap.set(cv2.CAP_PROP_FOCUS, focus_val)
        else:
            # Di Linux V4L2 (XC-TECH USB Camera), rentang focus_absolute = 0..1023 (4x skala 0..255 Windows)
            # Harus 2 panggilan terpisah karena focus_absolute berstatus inactive saat autofocus masih ON.
            v4l2_val = int(round(max(0, min(255, int(focus_val))) * 1023 / 255.0))
            cap.set(cv2.CAP_PROP_FOCUS, v4l2_val)
            try:
                dev = f"/dev/video{cam_idx}"
                subprocess.run(
                    ["v4l2-ctl", "-d", dev, "--set-ctrl=focus_automatic_continuous=0"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5, check=False,
                )
                subprocess.run(
                    ["v4l2-ctl", "-d", dev, f"--set-ctrl=focus_absolute={v4l2_val}"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5, check=False,
                )
                subprocess.run(
                    ["v4l2-ctl", "-d", dev, "--set-ctrl=brightness=4,contrast=13,sharpness=11"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5, check=False,
                )
            except Exception:
                pass
    else:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        if not sys.platform.startswith("win"):
            try:
                dev = f"/dev/video{cam_idx}"
                subprocess.run(
                    ["v4l2-ctl", "-d", dev, "--set-ctrl=focus_automatic_continuous=1"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5, check=False,
                )
                subprocess.run(
                    ["v4l2-ctl", "-d", dev, "--set-ctrl=brightness=4,contrast=13,sharpness=11"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5, check=False,
                )
            except Exception:
                pass


def vision_worker(state: BridgeState, scanner: QRScanner, roi_det: ConveyorROIDetector, mqtt_bridge=None):
    args = state.args
    if not sys.platform.startswith("win"):
        try:
            cv2.setNumThreads(2)
        except Exception:
            pass
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_V4L2
    cap = cv2.VideoCapture(args.cam, backend)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"[ERROR] Kamera index {args.cam} gagal dibuka.")
        state.running = False
        return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if sys.platform.startswith("win"):
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    else:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    # Set fokus awal (manual lock jika --focus >= 0, atau auto focus jika -1)
    apply_camera_focus(cap, args.cam, state.focus_val)
    if state.focus_val >= 0:
        print(f"[CAMERA] Fokus dikunci MANUAL di nilai {state.focus_val}")

    stable_valid_cnt = 0
    stable_invalid_cnt = 0
    clear_since = None
    no_uart_obj_since = None
    latched_det = None          # Kunci hasil QR sesaat agar tidak kedip saat blur 1 frame
    last_qr_seen_at = 0.0
    empty_since = None
    calib_frames = []
    # Jika conveyor_bg.npz versi terbaru sudah tersimpan dari kalibrasi belt kosong, langsung pakai!
    # Jika belum ada, tunggu Auto-Exposure stabil lalu kalibrasi belt kosong otomatis.
    startup_waiting = not roi_det.has_bg()
    startup_t0 = time.time()
    stable_ae_cnt = 0
    prev_ae_luma = -1.0
    if startup_waiting:
        with state.lock:
            state.calibrate_requested = True
        print("[ROI] Menunggu auto-exposure & white-balance kamera stabil untuk kalibrasi belt kosong...")
    else:
        print("[ROI] Menggunakan kalibrasi belt kosong tersimpan (klik 'Kalibrasi Belt Kosong' jika posisi/lampu berubah).")

    prev_t = time.time()
    fps_smooth = 0.0
    last_seen_qr_seq = -1
    frame_counter = 0
    last_no_uart_pub_id = ""
    last_roi_info = None
    lut_cache_boost = -1
    lut_cache = None

    print(f"[CAMERA] Aktif {args.width}x{args.height} @ {args.fps} FPS | ROI={state.roi_norm}")

    def commit_decision(info, scan_id, snap_frame=None, snap_pts=None):
        nonlocal clear_since
        with state.lock:
            telem = dict(state.telemetry)

        record = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "scan_id": scan_id,
            "status": info["status"],
            "box_id": info["box_id"],
            "lot": info["lot"],
            "variety": info["variety"],
            "process": info["process"],
            "mc": info["mc"],
            "grade": info["grade"],
            "weight": info["weight"],
            "target_silo": info["target_silo"],
            "reason": info["reason"],
            "temp": telem.get("temp", -1),
            "hum": telem.get("hum", -1),
            "gas": telem.get("gas", 0),
        }
        save_scan_record(record)

        if info["valid"]:
            reply = {"cmd": "SCAN_VALID", "scan_id": scan_id, "id": info["box_id"], "silo": info["target_silo"]}
            with state.lock:
                state.scan_phase = "DECIDED_VALID"
                state.cached_reply = reply
                state.last_result = record
                state.alert_msg = ""
                state.valid_count += 1
                state.history.insert(0, record)
                state.history = state.history[:20]
            state.send_uart_json(reply)
            if mqtt_bridge is not None:
                mqtt_bridge.publish_valid_scan(info, frame_bgr=snap_frame, qr_pts=snap_pts)
        else:
            reply = {"cmd": "SCAN_INVALID", "scan_id": scan_id, "id": info["box_id"], "reason": info["reason"]}
            clear_since = None
            with state.lock:
                state.scan_phase = "WAIT_REMOVAL"
                state.cached_reply = reply
                state.last_result = record
                state.alert_msg = f"BARANG GAK VALID ({info['reason']}) - Angkat barang dari konveyor!"
                state.invalid_count += 1
                state.history.insert(0, record)
                state.history = state.history[:20]
            state.send_uart_json(reply)

    while state.running:
        ret, raw_frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue
        frame_counter += 1

        # Paksa motor fokus UVC (VCM) bergerak setelah stream aktif (frame ke-5):
        # Banyak webcam UVC mengabaikan focus_absolute sebelum cap.read() pertama, atau men-cache nilai lama.
        if frame_counter == 5 and state.focus_val >= 0:
            nudge = state.focus_val - 5 if state.focus_val >= 5 else state.focus_val + 5
            apply_camera_focus(cap, args.cam, nudge)
            time.sleep(0.08)
            apply_camera_focus(cap, args.cam, state.focus_val)
            print(f"[CAMERA] Auto-nudge lensa VCM setelah stream aktif -> kunci tajam di {state.focus_val}")

        with state.lock:
            auto_boost = state.auto_boost
            boost = state.soft_boost
            do_calib = state.calibrate_requested
            fcmd = state.focus_cmd
            state.focus_cmd = None
            scan_phase = state.scan_phase
            active_id = state.active_scan_id
            started_at = state.scan_started_at
            esp_present = int(state.telemetry.get("present", 0))
            esp_state = str(state.telemetry.get("state", "OFFLINE"))
            need_jpeg = (state.stream_clients > 0)

        # Terapkan perintah fokus / panel driver kamera secara real-time di thread kamera
        if fcmd:
            if fcmd == "focus_auto":
                apply_camera_focus(cap, args.cam, -1)
                with state.lock:
                    state.focus_val = -1
                print("[CAMERA] Mode Fokus: AUTO")
            elif fcmd in ("focus_up", "focus_down"):
                cur_f = state.focus_val
                if cur_f < 0:
                    hw_f = int(cap.get(cv2.CAP_PROP_FOCUS))
                    cur_f = hw_f if 0 <= hw_f <= 255 else 40
                cur_f = max(0, min(255, cur_f + (5 if fcmd == "focus_up" else -5)))
                apply_camera_focus(cap, args.cam, cur_f)
                with state.lock:
                    state.focus_val = cur_f
                print(f"[CAMERA] Fokus DIKUNCI MANUAL = {cur_f} (gunakan --focus {cur_f} agar permanen)")
            elif fcmd == "driver_panel" and sys.platform.startswith("win"):
                print("[CAMERA] Membuka panel kontrol driver Windows (matikan Auto Focus di tab Camera Control)...")
                cap.set(cv2.CAP_PROP_SETTINGS, 1)

        # Kalibrasi latar belakang belt kosong (saat startup tunggu sampai auto-exposure benar-benar stabil)
        if do_calib:
            if startup_waiting:
                cur_luma = float(np.mean(cv2.resize(raw_frame, (80, 45))))
                if prev_ae_luma >= 0.0 and abs(cur_luma - prev_ae_luma) < 0.6 and cur_luma > 15.0:
                    stable_ae_cnt += 1
                else:
                    stable_ae_cnt = 0
                prev_ae_luma = cur_luma
                if stable_ae_cnt >= 15 or (time.time() - startup_t0) >= 4.5:
                    startup_waiting = False
                else:
                    with state.lock:
                        state.belt_status = "STABILISASI CAHAYA..."
                    continue
            calib_frames.append(raw_frame.copy())
            if len(calib_frames) >= 10:
                # Cegah kalibrasi startup menimpa belt kosong jika saat mulai sudah ada box QR di atas konveyor!
                has_startup_qr = (not roi_det.has_bg()) or False
                startup_qr_hits = scanner.detect_in_roi(calib_frames[-1], state.roi_norm)
                if startup_qr_hits and roi_det.has_bg():
                    print("[ROI] Terdeteksi QR di atas sabuk saat startup -> pakai kalibrasi belt kosong tersimpan!")
                else:
                    roi_det.calibrate_from_frames(calib_frames, save=(not bool(startup_qr_hits)))
                    if startup_qr_hits:
                        print("[ROI] PERINGATAN: Ada QR di konveyor saat startup! Angkat barang lalu klik 'Kalibrasi Belt Kosong'.")
                    else:
                        print("[ROI] Kalibrasi belt kosong live selesai & disimpan ke conveyor_bg.npz")
                calib_frames.clear()
                with state.lock:
                    state.calibrate_requested = False
                    state.alert_msg = ""
                    state.last_result = None

        # 1. Ambil hasil terbaru dari thread QR terlebih dahulu (hanya live deteksi, BUKAN latch!)
        fresh_qr_update = False
        with state.qr_res_lock:
            live_qr_dets = list(state.qr_latest_dets)
            if state.qr_res_seq != last_seen_qr_seq:
                last_seen_qr_seq = state.qr_res_seq
                fresh_qr_update = True

        # 2. Cek keberadaan benda di ROI (tiap 3 frame saat IDLE agar FPS Raspi tinggi, tiap 2 frame saat SCANNING)
        roi_stride = 2 if (scan_phase == "SCANNING") else 3
        if last_roi_info is None or (frame_counter % roi_stride == 0):
            roi_info = roi_det.analyze(raw_frame, has_qr_in_roi=(len(live_qr_dets) > 0))
            last_roi_info = roi_info
        else:
            roi_info = last_roi_info
        belt_st = roi_info["status"]

        # Hitung ketajaman fokus (Laplacian variance) tiap 8 frame (dan langsung di frame ke-6 setelah nudge)
        if frame_counter == 6 or (frame_counter % 8 == 0):
            rx1_s, ry1_s, rx2_s, ry2_s = roi_to_pixels(raw_frame.shape, state.roi_norm)
            roi_g = cv2.cvtColor(cv2.resize(raw_frame[ry1_s:ry2_s, rx1_s:rx2_s], (120, 160)), cv2.COLOR_BGR2GRAY)
            sharp_val = float(cv2.Laplacian(roi_g, cv2.CV_64F).var())
            with state.lock:
                state.sharpness = sharp_val

        # 3. Kirim frame ke thread QR background:
        #    - Mode DEEP multi-engine hanya saat SCANNING (atau --no-uart) agar CPU Raspi 3 tidak tercekik di IDLE!
        #    - Mode FAST (pyzbar 1-pass ~3ms) tiap 6-15 frame saat IDLE sehingga preview tetap mulus 20+ FPS!
        deep_qr = (scan_phase == "SCANNING") or args.no_uart
        if deep_qr:
            if (frame_counter % 2 == 0) or (scan_phase == "SCANNING"):
                with state.qr_req_lock:
                    state.qr_req_frame = raw_frame
                    state.qr_req_seq += 1
                    state.qr_req_active = True
                    state.qr_req_deep = True
        elif belt_st in ("HAND", "MOVING"):
            with state.qr_req_lock:
                state.qr_req_active = False
        elif (belt_st == "OBJECT" and (frame_counter % 6 == 0)) or (belt_st == "EMPTY" and (frame_counter % 20 == 0)):
            with state.qr_req_lock:
                state.qr_req_frame = raw_frame
                state.qr_req_seq += 1
                state.qr_req_active = True
                state.qr_req_deep = False

        detections = live_qr_dets if belt_st not in ("HAND", "MOVING") else []

        # 4. Kunci QR dengan batas waktu (TTL 1.2 dtk) & lepas segera saat barang diambil (EMPTY/HAND/MOVING)!
        now_latch = time.time()
        if detections:
            latched_det = detections[0]
            last_qr_seen_at = now_latch
            empty_since = None
        elif belt_st == "EMPTY":
            if empty_since is None:
                empty_since = now_latch
            if (now_latch - empty_since) >= 0.20:
                latched_det = None
        elif belt_st in ("HAND", "MOVING"):
            empty_since = None
            latched_det = None
        else:
            empty_since = None
            if latched_det is not None and (now_latch - last_qr_seen_at) >= 1.2:
                latched_det = None

        effective_dets = detections if detections else ([latched_det] if latched_det is not None else [])

        belt_label_map = {
            "EMPTY": "KOSONG",
            "HAND": "TANGAN (DIABAIKAN)",
            "MOVING": "GERAK (TUNGGU DIAM)",
            "OBJECT": f"BARANG ({roi_info['score_pct']:.1f}%)",
        }

        # Mode uji kamera tanpa ESP32 (--no-uart)
        if args.no_uart:
            if effective_dets:
                no_uart_obj_since = None
                inf = effective_dets[0]["info"]
                with state.lock:
                    state.last_result = {
                        "status": inf["status"], "box_id": inf["box_id"], "lot": inf["lot"],
                        "variety": inf["variety"], "process": inf["process"], "mc": inf["mc"],
                        "grade": inf["grade"], "target_silo": inf["target_silo"], "reason": inf["reason"],
                    }
                    state.alert_msg = "" if inf["valid"] else f"BARANG GAK VALID: {inf['reason']}"
                if inf["valid"] and inf["box_id"] and inf["box_id"] != last_no_uart_pub_id:
                    last_no_uart_pub_id = inf["box_id"]
                    if mqtt_bridge is not None:
                        mqtt_bridge.publish_valid_scan(inf, frame_bgr=raw_frame, qr_pts=effective_dets[0].get("pts"))
            elif belt_st == "OBJECT":
                if no_uart_obj_since is None:
                    no_uart_obj_since = time.time()
                elif (time.time() - no_uart_obj_since) >= 1.8:
                    with state.lock:
                        state.last_result = {"status": "INVALID", "box_id": "", "reason": "Barang di conveyor tidak memiliki QR valid!"}
                        state.alert_msg = "BARANG GAK VALID: Tidak ada QR valid pada barang di konveyor!"
            else:
                no_uart_obj_since = None
                if belt_st == "EMPTY":
                    last_no_uart_pub_id = ""
                with state.lock:
                    state.last_result = None
                    state.alert_msg = ""

        # Alur penuh terhubung ESP32 (dipicu SCAN_REQ)
        if scan_phase == "SCANNING":
            if fresh_qr_update and detections:
                inf = detections[0]["info"]
                if inf["valid"]:
                    stable_valid_cnt += 1
                    stable_invalid_cnt = 0
                    if stable_valid_cnt >= args.confirm_frames:
                        commit_decision(inf, active_id, snap_frame=raw_frame, snap_pts=detections[0].get("pts"))
                        stable_valid_cnt = 0
                else:
                    stable_invalid_cnt += 1
                    stable_valid_cnt = 0
                    if stable_invalid_cnt >= args.confirm_frames:
                        commit_decision(inf, active_id)
                        stable_invalid_cnt = 0
            elif fresh_qr_update and not detections:
                stable_valid_cnt = 0
                stable_invalid_cnt = 0

            if not detections and belt_st != "HAND" and (time.time() - started_at) >= args.qr_timeout:
                if belt_st == "EMPTY":
                    # Kamera memastikan konveyor benar-benar kosong (false trigger pantulan HC-SR04)
                    print(f"[SCAN] SCAN_REQ #{active_id} diabaikan: konveyor KOSONG (tidak ada barang) -> reset ESP32 ke IDLE")
                    state.send_uart_json({"cmd": "CONV_AUTO"})
                    with state.lock:
                        state.scan_phase = "IDLE"
                        state.cached_reply = None
                        state.alert_msg = ""
                    stable_valid_cnt = 0
                    stable_invalid_cnt = 0
                else:
                    no_qr_info = {
                        "box_id": "", "lot": "-", "variety": "-", "process": "-",
                        "mc": 0.0, "grade": "-", "weight": "-", "target_silo": "-",
                        "status": "INVALID", "valid": False,
                        "reason": f"Tidak ada QR valid setelah {args.qr_timeout:.1f}s",
                    }
                    commit_decision(no_qr_info, active_id)

        elif scan_phase == "WAIT_REMOVAL" or esp_state == "INVALID_HOLD":
            roi_empty = (belt_st == "EMPTY") and (len(detections) == 0)
            if roi_empty:
                if clear_since is None:
                    clear_since = time.time()
                need_sec = 1.0 if (esp_present == 0 or args.no_uart) else 2.0
                if (time.time() - clear_since) >= need_sec:
                    state.send_uart_json({"cmd": "ITEM_CLEARED", "scan_id": active_id})
                    if esp_present != 0 and not args.no_uart:
                        # Pengaman jika HC-SR04 membaca pantulan dinding belakang saat belt sudah kosong
                        state.send_uart_json({"cmd": "CONV_AUTO"})
                    with state.lock:
                        state.scan_phase = "IDLE"
                        state.cached_reply = None
                        state.alert_msg = ""
                        state.last_result = None
                    clear_since = None
            else:
                clear_since = None

        now_t = time.time()
        dt = now_t - prev_t
        prev_t = now_t
        if dt > 0:
            inst = 1.0 / dt
            fps_smooth = inst if fps_smooth == 0.0 else (0.85 * fps_smooth + 0.15 * inst)

        with state.lock:
            state.fps = fps_smooth
            state.belt_status = belt_label_map.get(belt_st, belt_st)
            alert_now = state.alert_msg
            fval_now = state.focus_val
            sharp_now = state.sharpness

        # Auto-boost pencahayaan tampilan berdasarkan ambien (tiap 5 frame agar hemat CPU)
        if auto_boost and (frame_counter % 5 == 0):
            amb = roi_info.get("ambient_luma", 80.0)
            target_b = int(np.clip((115.0 - amb) * 0.65, 0, 45))
            boost = int(round(0.85 * boost + 0.15 * target_b))
            with state.lock:
                state.soft_boost = boost

        if need_jpeg:
            rh, rw = raw_frame.shape[:2]
            dw, dh = (854, 480) if rw > 854 else (rw, rh)
            vis = cv2.resize(raw_frame, (dw, dh), interpolation=cv2.INTER_LINEAR) if (dw != rw or dh != rh) else raw_frame.copy()
            # Pada ARM/Raspi lewati LUT & 4x bitshift per-piksel agar FPS stream tetap lancar (>15 FPS)
            is_win = sys.platform.startswith("win")
            if boost > (2 if is_win else 12):
                if boost != lut_cache_boost or lut_cache is None:
                    lut_cache = np.clip(np.arange(256, dtype=np.float32) * 1.08 + boost, 0, 255).astype(np.uint8)
                    lut_cache_boost = boost
                cv2.LUT(vis, lut_cache, dst=vis)

            sx, sy = dw / float(rw), dh / float(rh)
            rx1, ry1, rx2, ry2 = roi_to_pixels(vis.shape, state.roi_norm)

            if is_win:
                # Redupkan area luar ROI hanya di desktop/PC (hemat ~30ms/frame di CPU ARM Raspi 3)
                vis[:, :rx1] >>= 1
                vis[:, rx2:] >>= 1
                vis[:ry1, rx1:rx2] >>= 1
                vis[ry2:, rx1:rx2] >>= 1

            if effective_dets and effective_dets[0]["info"]["valid"]:
                roi_color = (0, 255, 100)
            elif alert_now or (effective_dets and not effective_dets[0]["info"]["valid"]):
                roi_color = (0, 0, 255)
            elif belt_st in ("HAND", "MOVING"):
                roi_color = (0, 200, 255)
            else:
                roi_color = (180, 180, 180)

            cv2.rectangle(vis, (rx1, ry1), (rx2, ry2), roi_color, 2)
            roi_lbl = f"CONVEYOR ROI: {belt_label_map.get(belt_st, belt_st)}"
            (tw, th), _ = cv2.getTextSize(roi_lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.46, 1)
            ly = max(th + 30, ry1 - 4)
            cv2.rectangle(vis, (rx1, ly - th - 6), (min(dw - 2, rx1 + tw + 10), ly + 2), (18, 20, 26), -1)
            cv2.putText(vis, roi_lbl, (rx1 + 5, ly - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, roi_color, 1, cv2.LINE_AA)

            # Gambar kotak benda hanya saat belum ada poligon QR (agar tidak dobel kotak di atas QR)
            if not effective_dets and roi_info["bbox_frame"] is not None:
                bx1, by1, bx2, by2 = roi_info["bbox_frame"]
                cv2.rectangle(vis, (int(bx1 * sx), int(by1 * sy)), (int(bx2 * sx), int(by2 * sy)), roi_color, 2)

            for det in effective_dets:
                inf = det["info"]
                color = (0, 255, 100) if inf["valid"] else (0, 0, 255)
                pts = det["pts"]
                if pts is not None and len(pts) >= 3:
                    spts = np.column_stack((pts[:, 0] * sx, pts[:, 1] * sy)).astype(np.int32)
                    cv2.polylines(vis, [spts.reshape((-1, 1, 2))], True, color, 2)
                    tx, ty = int(spts[0][0]), max(46, int(spts[0][1]) - 6)
                else:
                    tx, ty = rx1 + 8, ry1 + 24
                (qtw, qth), _ = cv2.getTextSize(det["label"], cv2.FONT_HERSHEY_SIMPLEX, 0.46, 1)
                tx = max(4, min(dw - qtw - 10, tx))
                cv2.rectangle(vis, (tx - 4, ty - qth - 5), (min(dw - 2, tx + qtw + 6), ty + 3), (18, 20, 26), -1)
                cv2.putText(vis, det["label"], (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)

            f_tag = "AUTO" if fval_now < 0 else f"MAN({fval_now})"
            s_tag = "TAJAM" if sharp_now >= 90.0 else "BUREM"
            hud_txt = f"FPS:{fps_smooth:.1f} | Focus:{f_tag} [{s_tag}:{sharp_now:.0f}] | ESP:{esp_state}"
            cv2.rectangle(vis, (0, 0), (dw, 26), (14, 17, 24), -1)
            cv2.putText(vis, hud_txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 230, 255), 1, cv2.LINE_AA)

            if alert_now:
                cv2.rectangle(vis, (0, dh - 34), (dw, dh), (0, 0, 180), -1)
                cv2.putText(vis, alert_now[:74], (10, dh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA)

            ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 58])
            if ok:
                with state.frame_cond:
                    state.latest_jpeg = buf.tobytes()
                    state.frame_seq += 1
                    state.frame_cond.notify_all()

    cap.release()


def run_dataset_benchmark(dataset_dir, roi_norm, max_mc):
    """Uji akurasi ROI detector + QR scanner pada seluruh foto di folder DATASET."""
    roi_det = ConveyorROIDetector(roi_norm=roi_norm, auto_load_saved=False)
    ds_empty = sorted(glob.glob(os.path.join(dataset_dir, "*14_14_*.jpg")))
    if ds_empty:
        roi_det.calibrate_from_frames([cv2.imread(p) for p in ds_empty if cv2.imread(p) is not None], save=False)
    scanner = QRScanner(max_mc=max_mc)
    imgs = sorted(glob.glob(os.path.join(dataset_dir, "*.jpg")) + glob.glob(os.path.join(dataset_dir, "*.png")))
    for p in imgs:
        im = cv2.imread(p)
        if im is None:
            continue
        roi_det.prev_L = None  # uji frame statis terpisah
        qrs = scanner.detect_in_roi(im, roi_norm)
        info = roi_det.analyze(im, has_qr_in_roi=(len(qrs) > 0), allow_adapt=False)
        verdict = "KOSONG"
        if qrs:
            verdict = f"QR:{qrs[0]['info']['box_id']} ({qrs[0]['info']['status']})"
        elif info["status"] == "OBJECT":
            verdict = "BARANG TANPA QR -> INVALID"
        elif info["status"] == "HAND":
            verdict = "TANGAN -> DIABAIKAN"
        print(f"{os.path.basename(p):36s} | ROI={info['status']:6s} ({info['score_pct']:4.1f}%) | {verdict}")


def main():
    parser = argparse.ArgumentParser(description="Silo ESP32 <-> Raspberry Pi 3 Conveyor ROI + QR Bridge")
    parser.add_argument("--cam", "-c", type=int, default=0, help="Index webcam USB (Raspi: 0, Windows eksternal: 1)")
    parser.add_argument("--width", type=int, default=1280, help="Lebar frame 16:9 (default: 1280, Pi 3 bisa 640)")
    parser.add_argument("--height", type=int, default=720, help="Tinggi frame 16:9 (default: 720, Pi 3 bisa 360)")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS kamera (default: 30)")
    parser.add_argument("--boost", type=int, default=-1, help="Brightness boost (-1 = otomatis ikut cahaya ruangan, atau 0..100 manual)")
    parser.add_argument("--focus", type=int, default=255, help="Kunci fokus manual kamera (default: 255, atau -1 = auto focus)")
    parser.add_argument("--roi", default="0.33,0.20,0.67,0.90", help="Area sabuk konveyor x1,y1,x2,y2 (rasio 0..1)")
    parser.add_argument("--max-mc", type=float, default=12.5, help="Batas kadar air %% lolos scan (default: 12.5)")
    parser.add_argument("--confirm-frames", type=int, default=2, help="Frame stabil sebelum kirim hasil (default: 2)")
    parser.add_argument("--qr-timeout", type=float, default=4.0, help="Timeout tunggu QR saat SCAN_REQ (detik)")
    parser.add_argument("--port", default="/dev/serial0" if not sys.platform.startswith("win") else "COM3", help="Port UART ESP32")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate UART (default: 115200)")
    parser.add_argument("--no-uart", action="store_true", help="Tes kamera + ROI + QR saja tanpa UART ESP32")
    parser.add_argument("--http-port", type=int, default=5000, help="Port web stream (default: 5000)")
    parser.add_argument("--mqtt-host", default="127.0.0.1", help="Host broker MQTT (default: 127.0.0.1)")
    parser.add_argument("--mqtt-port", type=int, default=1883, help="Port broker MQTT (default: 1883)")
    parser.add_argument("--device-id", default="silo-01", help="ID perangkat untuk payload MQTT (default: silo-01)")
    parser.add_argument("--no-mqtt", action="store_true", help="Matikan publish/subscribe MQTT")
    parser.add_argument("--test-dataset", default="", help="Uji akurasi ROI & QR pada folder DATASET lalu keluar")
    args = parser.parse_args()

    roi_norm = parse_roi(args.roi)

    if args.test_dataset:
        run_dataset_benchmark(args.test_dataset, roi_norm, args.max_mc)
        return

    scanner = QRScanner(max_mc=args.max_mc)
    roi_det = ConveyorROIDetector(roi_norm=roi_norm, auto_load_saved=True)
    state = BridgeState(args, roi_norm)
    mqtt_bridge = MqttBridge(state, host=args.mqtt_host, port=args.mqtt_port, device_id=args.device_id, enabled=(not args.no_mqtt))
    mqtt_bridge.start()

    def stop_handler(signum, frame):
        print("\n[SISTEM] Menutup bridge...")
        state.running = False
        with state.frame_cond:
            state.frame_cond.notify_all()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    t_uart = threading.Thread(target=uart_worker, args=(state, mqtt_bridge), daemon=True)
    t_qr = threading.Thread(target=qr_worker, args=(state, scanner), daemon=True)
    t_vis = threading.Thread(target=vision_worker, args=(state, scanner, roi_det, mqtt_bridge), daemon=True)
    t_uart.start()
    t_qr.start()
    t_vis.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.http_port), make_handler(state))
    server.timeout = 0.5
    print(f"[WEB] Dashboard aktif di http://127.0.0.1:{args.http_port} (atau http://<IP_RASPI>:{args.http_port})")

    try:
        while state.running:
            server.handle_request()
    finally:
        state.running = False
        with state.frame_cond:
            state.frame_cond.notify_all()
        mqtt_bridge.stop()
        server.server_close()
        t_vis.join(timeout=2.0)
        t_qr.join(timeout=1.0)
        t_uart.join(timeout=1.0)
        print("[SISTEM] Selesai.")


if __name__ == "__main__":
    main()
