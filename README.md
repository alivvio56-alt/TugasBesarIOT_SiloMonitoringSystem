# Sistem Monitoring & Pengendalian Silo Raw Material dengan Smart Conveyor PID Berbasis IoT

## Identitas Kelompok

| Keterangan | Informasi |
|---|---|
| Kelompok | 2 |
| Kelas | A |
| Mata Kuliah | Internet of Things (IoT) |

### Anggota Kelompok & Peran

| No | Nama | NIM | Peran Utama |
|---|---|---|---|
| 1 | **Afif Nur Rahman** | H1H024016 | Pemrograman Raspberry Pi 3, *vision* QR (Python), UART-MQTT, & PID *conveyor* |
| 2 | **Ardina Jihan Mariska** | H1H024018 | Pengembangan web *dashboard* Laravel, integrasi REST API Go, & fitur kendali |
| 3 | **Nesa Dwi Cahyani** | H1H024024 | Perancangan awal & pengembangan antarmuka web Laravel, *tracking*, & kendali |
| 4 | **Javier Antonio Justiansah** | H1H024026 | Mekanikal *conveyor*, diagram blok sistem, skenario pengujian, & dokumentasi |
| 5 | **Ardhis Alivio Rajendra** | H1H024031 | *Wiring hardware*, desain & cetak *enclosure*, akuisisi sensor/aktuator, & mekanik |

---

## Deskripsi Proyek

Sistem berbasis **Internet of Things (IoT)** dan **Sistem Kendali** untuk memantau kondisi lingkungan silo bahan baku sekaligus mengotomatisasi pelacakan barang masuk (*traceability*) melalui *smart conveyor*.

- **Lingkungan Silo (ESP32):** Memantau suhu, kelembapan (DHT22), dan gas (MQ-2 dengan filter EMA) untuk mengatur 2 kipas *exhaust* (relay) serta katup ventilasi (servo SG90) dengan logika histeresis.
- **Smart Conveyor & Vision QR (ESP32 + Raspi 3):** Menghentikan kotak tepat di bawah *webcam* menggunakan PID diskrit (HC-SR04 & L298N), memindai QR Code dengan OpenCV + pyzbar, menyalakan LED indikator (`VALID`/`INVALID`), lalu memasukkan barang valid ke silo.
- **Web Dashboard & Server Lokal:** Menyajikan monitoring *real-time*, riwayat *tracking* beserta foto bukti, *live* kamera, dan kendali perangkat jarak jauh menggunakan Go API, SQLite, Laravel, Mosquitto MQTT, Nginx, dan Cloudflare Tunnel.

## Tujuan Proyek

1. Memantau suhu, kelembapan, dan indikator gas silo secara *real-time*.
2. Mengendalikan kipas dan katup ventilasi secara otomatis (histeresis) maupun manual.
3. Mengatur posisi henti *conveyor* secara presisi menggunakan kendali PID diskrit.
4. Melacak identitas bahan baku melalui pemindaian QR Code beserta tangkapan foto bukti.
5. Menyediakan tampilan lokal (LCD & LED) serta *dashboard* web terintegrasi.

## Komponen dan Teknologi

| Komponen / Teknologi | Spesifikasi / Pin | Fungsi |
|---|---|---|
| **ESP32 DevKit** | Dual-core, UART2 (`RX=32`, `TX=33`) | Pengendali sensor, histeresis, PID, LCD, & LED |
| **Raspberry Pi 3 Model B** | UART (`Pin 8/10`), USB, Wi-Fi | *Gateway* UART-MQTT, *vision* QR, & *host* server |
| **DHT22** | GPIO 4 | Sensor suhu (Celcius) dan kelembapan (%RH) |
| **MQ-2** | GPIO 34 (ADC) | Sensor indikator gas (dengan filter EMA) |
| **HC-SR04** | TRIG 16, ECHO 17 | Sensor jarak umpan balik PID *conveyor* |
| **Webcam USB** | 640x360 MJPG, fokus manual | Deteksi ROI *conveyor* & pemindai QR Code |
| **L298N + Motor DC** | ENA 25, IN1 26, IN2 27 | Penggerak PWM (62-125) & rem aktif *conveyor* |
| **Relay + 2x Fan DC 12V** | GPIO 19 (logika NC) | Sakelar ON/OFF kipas *exhaust* silo |
| **Servo SG90** | GPIO 18 | Penggerak katup ventilasi (0 / 90 derajat) |
| **LCD 16x2 I2C** | SDA 21, SCL 22 (`0x27`) | Tampilan status sensor, aktuator, & hasil *scan* |
| **LED Hijau & Merah** | GPIO 13 & GPIO 14 | Indikator hasil *scan* QR (`VALID`/`INVALID`) |
| **Adaptor + 2x Step-Down** | 12V & 2x 5V LM2596 | Rel daya terpisah untuk motor/kipas & logika |
| **Python (OpenCV + pyzbar)** | *Service* `silo-bridge` | *Bridge* UART2, deteksi ROI/QR, *live cam*, MQTT |
| **Mosquitto MQTT** | Port 1883 (3 topik) | *Broker* pesan lokal (telemetri, *scan*, kontrol) |
| **Go API + SQLite** | Port 3000 | REST API & penyimpanan riwayat data serta foto |
| **Laravel + Nginx** | Port 8000 / Proxy 8080 | *Dashboard* web, *tracking*, & kendali perangkat |

## Mekanisme Kerja Sistem

1. **Kendali Lingkungan Silo (Histeresis):** ESP32 membaca DHT22 dan MQ-2 (filter EMA). Kipas menyala otomatis jika suhu > 28 C (mati < 27 C) atau gas > 2000 ADC (pulih < 1800 ADC). Servo membuka ventilasi (90 derajat) jika alarm gas aktif atau kelembapan < 50% (tutup kembali saat >= 70% atau > 80%).
2. **Kendali PID Conveyor:** HC-SR04 mendeteksi kotak di zona awal (<= 17,5 cm), menunggu jeda 2 detik, lalu PID (`Kp=6,5`, `Ki=0,45`, `Kd=4,0`, interval 60 ms) memodulasi PWM L298N (62-125) agar *conveyor* melambat halus dan berhenti di *setpoint* 20,5 cm (toleransi +/- 0,4 cm) dengan rem aktif.
3. **Validasi QR Code (Computer Vision):** Saat berhenti, ESP32 mengirim permintaan *scan* via UART2 ke Raspberry Pi 3. Kamera memeriksa ROI (terlatih untuk kondisi *belt* kosong, QR valid, dan benda acak/invalid). Jika **VALID**, LED Hijau menyala 2 detik, LCD menampilkan status valid, data + foto tersimpan, lalu *conveyor* berjalan 2 detik memasukkan barang ke silo. Jika **INVALID**, LED Merah menyala dan *conveyor* berhenti sampai barang diangkat.
4. **Komunikasi MQTT & Web:** Raspberry Pi mem-publish 2 topik (telemetri data & hasil *scan*) dan men-subscribe 1 topik (kontrol) melalui Mosquitto MQTT; sebaliknya *backend* Go men-subscribe telemetri/*scan* untuk disimpan ke SQLite dan mem-publish perintah kontrol dari *dashboard* Laravel.

## Arsitektur Sistem

- **Field Layer (ESP32):** Akuisisi sensor (DHT22, MQ-2, HC-SR04), kendali histeresis & PID, LCD 16x2, dan LED indikator.
- **Edge & Gateway Layer (Raspi 3):** Komunikasi UART2 (`115200 bps`), *computer vision* QR + *live streaming*, dan *broker* Mosquitto MQTT.
- **Application Layer:** Go API + SQLite, *Web Dashboard* Laravel (`Silo SCM`), dan Nginx (*auto-start* `systemd`).

## Status Pengembangan

| Bagian | Status |
|---|---|
| Desain skematik, *enclosure control box*, & perakitan fisik silo + *conveyor* | Selesai |
| Akuisisi sensor DHT22, MQ-2 (EMA), dan HC-SR04 (median filter) | Selesai |
| Kendali histeresis kipas & servo serta PID *conveyor* (*anti-stall* + rem aktif) | Selesai |
| Tampilan lokal LCD 16x2 I2C dan indikator LED Valid/Invalid | Selesai |
| Integrasi UART2 ESP32 - Raspi 3 dan *computer vision* QR + foto bukti | Selesai |
| Layanan lokal Mosquitto MQTT, Go API + SQLite, dan Dashboard Laravel | Selesai |
| Konfigurasi *auto-start* (`systemd`) dan pengujian sistem terintegrasi | Selesai |

## Dokumentasi

### Video Presentasi & Demonstrasi Lengkap

[**Tonton Video Presentasi Kelompok (YouTube)**](https://youtu.be/JVmAXdI7Zao)

### Simulasi Rangkaian Awal

[**Lihat Simulasi Rangkaian Cirkit Designer**](https://app.cirkitdesigner.com/project/09ebd522-7023-452e-b6f0-0ca62a58bb7d)

### Fitur Dashboard Web (`Silo SCM`)

- **Ringkasan:** Menampilkan status perangkat (*online*/*offline*), pembacaan *real-time* suhu, kelembapan, gas ADC, dan jarak beserta indikator kondisi (`NORMAL`, `WARNING`, `CRITICAL`), grafik tren suhu & kelembapan (filter `1 jam`, `6 jam`, `24 jam`, `3 hari`), grafik batang indikator gas dengan garis batas kritis (2500), persentase level gas, daftar *batch* terbaru, serta log kejadian (*conveyor*, barang masuk, dan perubahan mode).
- **Riwayat Tracking:** Tabel *batch* barang valid yang masuk otomatis (foto, ID box, varietas, proses, *grade*, berat, silo tujuan, dan waktu) dilengkapi halaman **Detail Lengkap** berisi nomor lot, kode FRM, kadar air, koordinat & ketinggian (tautan Google Maps), tangkapan foto bukti saat valid, serta data mentah JSON perangkat.
- **Kendali Perangkat:** Pengaturan mode `Otomatis` (berbasis sensor ESP32) dan `Manual` (sakelar langsung untuk kipas, ventilasi, dan *conveyor*) lengkap dengan panel pembacaan sensor langsung secara *real-time*.
- **Live Kamera:** *Streaming* area *conveyor* dan ROI deteksi QR, status perangkat keras (state ESP32, jarak, PWM, status LED, jumlah valid/invalid), serta tombol kalibrasi *belt* kosong dan pengaturan fokus/pencahayaan lensa.

---

**Kelompok 2 – Kelas A**  
**Proyek Tugas Besar Internet of Things (IoT)**
