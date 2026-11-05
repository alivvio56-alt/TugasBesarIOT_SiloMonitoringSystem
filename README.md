# Sistem Monitoring dan Pengendalian Kondisi Lingkungan Silo Penyimpanan Gandum

## Identitas Kelompok

| Keterangan | Informasi |
|---|---|
| Kelompok | 2 |
| Kelas | A |
| Mata Kuliah | Internet of Things (IoT) |

### Anggota Kelompok

1. **Afif Nur Rahman** — H1H024016
2. **Ardina Jihan Mariska** — H1H024018
3. **Ardhis Alivio Rajendra** — H1H024031
4. **Nesa Dwi Cahyani** — H1H024024
5. **Javier Anthonio Justiansah** — H1H024026

---

## Deskripsi Proyek

Proyek ini merupakan pengembangan sistem berbasis **Internet of Things (IoT)** untuk melakukan monitoring dan pengendalian kondisi lingkungan pada silo penyimpanan bahan baku.

Sistem dirancang menggunakan ESP32 sebagai mikrokontroler utama yang terhubung dengan sensor DHT22 untuk membaca suhu dan kelembapan udara, serta sensor MQ135 untuk mendeteksi perubahan kualitas udara.

Data hasil pembacaan sensor diproses untuk menentukan kondisi lingkungan ke dalam tiga kategori, yaitu **NORMAL, WARNING, dan CRITICAL**. Berdasarkan kondisi tersebut, sistem dapat mengendalikan kipas ventilasi melalui relay serta memberikan indikator peringatan menggunakan LED dan buzzer.

Selain pengendalian otomatis, sistem dirancang memiliki dashboard berbasis web yang memungkinkan pengguna memantau kondisi lingkungan dan mengendalikan kipas secara manual maupun otomatis melalui komunikasi MQTT.

## Tujuan Proyek

1. Memantau suhu dan kelembapan lingkungan di dalam silo.
2. Mendeteksi perubahan kualitas udara menggunakan sensor MQ135.
3. Mengidentifikasi kondisi lingkungan berdasarkan tingkat risiko.
4. Mengendalikan sistem ventilasi secara otomatis maupun manual.
5. Menyediakan dashboard untuk monitoring dan pengendalian jarak jauh.
6. Mengembangkan penyimpanan data historis dan sistem kendali PID.

## Komponen dan Teknologi

| Komponen / Teknologi | Fungsi |
|---|---|
| ESP32 | Mikrokontroler dan pemrosesan data sensor |
| DHT22 | Mengukur suhu dan kelembapan udara |
| MQ135 | Mendeteksi perubahan kualitas udara |
| Relay | Mengendalikan kipas secara ON/OFF |
| Motor DC | Simulasi kipas atau ventilasi |
| LED | Indikator kondisi sistem |
| Buzzer | Alarm peringatan |
| MQTT / HiveMQ | Komunikasi data dan perintah kontrol |
| HTML, CSS, JavaScript | Pengembangan dashboard web |
| Laravel | Rencana pengembangan backend |
| SQLite | Rencana penyimpanan data historis |

## Mekanisme Kerja Sistem

1. ESP32 membaca data suhu, kelembapan, dan kualitas udara dari sensor.
2. Data sensor diproses oleh ESP32 untuk menentukan kondisi lingkungan.
3. Sistem mengklasifikasikan kondisi menjadi NORMAL, WARNING, atau CRITICAL.
4. Berdasarkan kondisi tersebut, ESP32 mengendalikan kipas ventilasi, LED, dan buzzer.
5. Data monitoring dikirimkan melalui MQTT menuju dashboard.
6. Pengguna dapat memantau kondisi silo dan memberikan perintah FAN ON, FAN OFF, atau AUTO MODE melalui dashboard.

## Arsitektur Sistem

Sistem terdiri dari tiga bagian utama:

- **Perangkat Keras:** ESP32, sensor DHT22, sensor MQ135, relay, motor DC, LED, dan buzzer.
- **Komunikasi:** MQTT menggunakan broker HiveMQ untuk pertukaran data monitoring dan perintah kontrol.
- **Aplikasi:** Dashboard web untuk menampilkan informasi sensor dan menyediakan kontrol kipas.

## Status Pengembangan

| Bagian | Status |
|---|---|
| Rancangan rangkaian ESP32 dan sensor | Selesai pada simulasi |
| Pembacaan dan pemrosesan sensor | Dirancang |
| Klasifikasi kondisi lingkungan | Dirancang |
| Kendali kipas ON/OFF | Dirancang pada simulasi |
| Indikator LED dan buzzer | Dirancang pada simulasi |
| Komunikasi MQTT | Jalur komunikasi dirancang |
| Dashboard monitoring | Prototipe telah dibuat |
| Integrasi perangkat fisik | Dalam pengembangan |
| Kendali PID menggunakan PWM | Rencana pengembangan |
| Backend Laravel dan database SQLite | Rencana pengembangan |

**Catatan:** Status di atas menggambarkan progress pengembangan berdasarkan laporan proyek. Implementasi fisik, pengujian komunikasi secara menyeluruh, dan validasi parameter sensor masih menjadi bagian dari pekerjaan berikutnya.

## Dokumentasi

### Simulasi Rangkaian

Simulasi rangkaian perangkat keras menggunakan Cirkit Designer.

[**Lihat Simulasi Rangkaian Cirkit Designer**](https://app.cirkitdesigner.com/project/09ebd522-7023-452e-b6f0-0ca62a58bb7d)

### Dashboard

Dashboard dirancang menggunakan HTML, CSS, dan JavaScript dengan komunikasi MQTT/WebSocket. Dashboard menampilkan:

- Suhu lingkungan.
- Kelembapan udara.
- Nilai pembacaan sensor gas.
- Status kondisi lingkungan.
- Status kipas ventilasi.
- Kontrol FAN ON, FAN OFF, dan AUTO MODE.

## Rencana Pengembangan Selanjutnya

1. Melakukan perakitan dan pengujian perangkat keras secara fisik.
2. Memvalidasi parameter sensor dan batas kondisi lingkungan.
3. Mengintegrasikan ESP32 dengan dashboard melalui MQTT.
4. Mengembangkan backend menggunakan Laravel.
5. Mengintegrasikan database SQLite untuk penyimpanan data historis.
6. Mengembangkan kendali PID untuk pengaturan ventilasi berdasarkan kondisi lingkungan.
7. Melakukan pengujian sistem secara menyeluruh.

---

**Kelompok 2 – Kelas A**  
**Proyek Tugas Besar Internet of Things (IoT)**
