# COMPONENTS VERSION1
# Dokumentasi Komponen dan Wiring Hardware

## 1. Deskripsi Proyek

Proyek ini menggunakan ESP32 sebagai mikrokontroler utama untuk membaca data sensor dan mengendalikan aktuator. Raspberry Pi 3 digunakan sebagai perangkat pemrosesan dan komunikasi data melalui UART.

ESP32 menangani pembacaan sensor suhu, kelembapan, gas, dan jarak, serta mengontrol motor DC, servo, dan relay.

## 2. Daftar Komponen

| No. | Komponen | Jumlah | Fungsi |
|---|---|---:|---|
| 1 | ESP32 DevKit V1 | 1 | Mikrokontroler utama |
| 2 | Raspberry Pi 3 | 1 | Pemrosesan dan komunikasi data |
| 3 | L298N Motor Driver | 1 | Pengendali motor DC |
| 4 | DC Motor | 1 | Aktuator penggerak |
| 5 | Relay Module | 1 | Sakelar elektronik |
| 6 | Servo Motor | 1 | Aktuator posisi |
| 7 | HC-SR04 | 1 | Sensor jarak ultrasonik |
| 8 | DHT22 | 1 | Sensor suhu dan kelembapan |
| 9 | MQ Sensor | 1 | Sensor gas |
| 10 | Power Supply Eksternal | Secukupnya | Sumber daya aktuator dan sensor |
| 11 | Kabel Jumper | Secukupnya | Penghubung antarperangkat |

## 3. Pin Configuration ESP32

### 3.1 Mapping GPIO

| Komponen | Pin Komponen | GPIO ESP32 | Fungsi |
|---|---|---:|---|
| L298N | ENA | GPIO25 | PWM motor |
| L298N | IN1 | GPIO26 | Kontrol arah motor |
| L298N | IN2 | GPIO27 | Kontrol arah motor |
| Relay | IN | GPIO19 | Kontrol relay |
| Servo | Signal | GPIO18 | Sinyal PWM servo |
| HC-SR04 | TRIG | GPIO16 | Trigger ultrasonik |
| HC-SR04 | ECHO | GPIO17 | Echo ultrasonik |
| DHT22 | DATA | GPIO4 | Data suhu dan kelembapan |
| MQ Sensor | AO | GPIO34 | Input analog |
| Raspberry Pi 3 | UART TX | GPIO21 | Pengiriman data |
| Raspberry Pi 3 | UART RX | GPIO22 | Penerimaan data |

## 4. Wiring Setiap Komponen

### 4.1 ESP32 dan L298N

| Pin L298N | Pin ESP32 |
|---|---|
| ENA | GPIO25 |
| IN1 | GPIO26 |
| IN2 | GPIO27 |
| GND | Common GND |

Koneksi motor:
- Motor positif dan negatif dihubungkan ke OUT1 dan OUT2.
- Supply motor dihubungkan ke terminal power L298N sesuai spesifikasi motor.
- Jumper ENA dilepas agar kecepatan motor dapat dikontrol melalui PWM GPIO25.

### 4.2 ESP32 dan Relay

| Pin Relay | Koneksi |
|---|---|
| IN | GPIO19 |
| VCC | Supply sesuai spesifikasi modul |
| GND | GND ESP32 / common ground |

Relay digunakan untuk mengendalikan beban eksternal, seperti kipas DC 12V.

### 4.3 ESP32 dan Servo

| Kabel Servo | Koneksi |
|---|---|
| Signal (Oranye/Kuning) | GPIO18 |
| VCC (Merah) | PSU eksternal 5V |
| GND (Cokelat/Hitam) | GND PSU |

GND PSU servo harus disatukan dengan GND ESP32 agar sinyal kontrol memiliki referensi yang sama.

Servo dikendalikan menggunakan sinyal PWM 50 Hz dengan rentang posisi 0–90 derajat pada program pengujian.

### 4.4 ESP32 dan HC-SR04

| Pin HC-SR04 | Koneksi |
|---|---|
| VCC | 5V |
| TRIG | GPIO16 |
| ECHO | GPIO17 melalui voltage divider |
| GND | Common GND |

**Perhatian:** Pin ECHO HC-SR04 menghasilkan tegangan 5V. Gunakan voltage divider untuk menurunkan tegangan menjadi sekitar 3.3V sebelum masuk ke GPIO17 ESP32.

### 4.5 ESP32 dan DHT22

| Pin DHT22 | Koneksi |
|---|---|
| VCC | 3.3V |
| DATA | GPIO4 |
| GND | Common GND |

DHT22 digunakan untuk mengukur suhu dan kelembapan lingkungan.

Jika menggunakan sensor DHT22 tanpa modul, diperlukan resistor pull-up sekitar 10 kΩ antara DATA dan VCC.

### 4.6 ESP32 dan MQ Sensor

| Pin MQ | Koneksi |
|---|---|
| VCC | Supply sesuai modul |
| GND | Common GND |
| AO | GPIO34 melalui voltage divider jika diperlukan |
| DO | Tidak digunakan |

GPIO34 digunakan untuk membaca keluaran analog sensor MQ.

**Perhatian:** Pastikan tegangan output analog AO tidak melebihi 3.3V karena GPIO ESP32 tidak toleran terhadap input 5V.

Nilai pembacaan awal berupa ADC dan tegangan, bukan konsentrasi gas dalam satuan PPM.

### 4.7 ESP32 dan Raspberry Pi 3

Komunikasi antara ESP32 dan Raspberry Pi 3 menggunakan UART dengan level logika 3.3V.

| ESP32 | Raspberry Pi 3 | Pin Fisik RPi |
|---|---|---:|
| GPIO21 (TX) | GPIO15 (RXD) | Pin 10 |
| GPIO22 (RX) | GPIO14 (TXD) | Pin 8 |
| GND | GND | Pin 6 |

Koneksi TX dan RX harus disilangkan:
- TX ESP32 menuju RX Raspberry Pi.
- RX ESP32 menuju TX Raspberry Pi.
- GND kedua perangkat disatukan.

UART Raspberry Pi harus diaktifkan dan serial console dinonaktifkan sebelum digunakan untuk komunikasi.

## 5. Power Distribution

| Perangkat | Sumber Tegangan |
|---|---|
| ESP32 | USB / 5V sesuai board |
| Raspberry Pi 3 | Supply 5V tersendiri sesuai kebutuhan |
| L298N | Supply motor sesuai spesifikasi motor |
| Motor DC | Melalui output L298N |
| Relay | Sesuai spesifikasi modul |
| Servo | PSU eksternal 5V |
| HC-SR04 | 5V |
| DHT22 | 3.3V |
| MQ Sensor | Sesuai spesifikasi modul |

### Ketentuan Ground

- GND ESP32 dan PSU eksternal disatukan untuk perangkat yang menggunakan referensi sinyal bersama.
- GND Raspberry Pi dan ESP32 disatukan untuk komunikasi UART.
- Jangan menggabungkan output positif dari beberapa power supply secara sembarangan.
- Pastikan kapasitas arus PSU mencukupi kebutuhan servo, motor, dan sensor.
- Untuk modul relay dengan optocoupler, ikuti konfigurasi isolasi sesuai skema modul.

## 6. Software dan Library

Pengembangan program ESP32 menggunakan Arduino IDE.

Library yang digunakan:

| Library | Fungsi |
|---|---|
| ESP32Servo | Kontrol posisi servo |
| DHT sensor library | Pembacaan DHT22 |
| Adafruit Unified Sensor | Dependensi sensor Adafruit |
| ESP32 Arduino Core | Dukungan pemrograman ESP32 |

## 7. Serial Monitor Commands

Pengujian peripheral dilakukan melalui Serial Monitor dengan baud rate 115200 dan line ending Newline.

| Perintah | Fungsi |
|---|---|
| `motor on` | Mengaktifkan motor |
| `motor off` | Mematikan motor |
| `motor speed 150` | Mengatur PWM motor |
| `relay on` | Mengaktifkan relay |
| `relay off` | Menonaktifkan relay |
| `servo on` | Menggerakkan servo ke 90° |
| `servo off` | Mengembalikan servo ke 0° |
| `distance` | Membaca jarak HC-SR04 |
| `temperature` | Membaca suhu dan kelembapan |
| `gas` | Membaca nilai analog MQ |
| `read` | Membaca seluruh sensor |
| `help` | Menampilkan daftar perintah |

## 8. Catatan Keselamatan Hardware

1. Jangan memberikan tegangan 5V langsung ke GPIO ESP32.
2. Gunakan voltage divider pada ECHO HC-SR04.
3. Pastikan output analog MQ aman untuk ADC ESP32.
4. Gunakan power supply eksternal untuk servo dan motor.
5. Perhatikan polaritas VCC dan GND setiap komponen.
6. Pastikan jumper ENA L298N dilepas ketika menggunakan kontrol PWM eksternal.
7. Lakukan pengujian setiap peripheral secara terpisah sebelum menjalankan keseluruhan sistem.

---

**Dokumentasi ini menggunakan konfigurasi GPIO ESP32 DevKit V1 / ESP32-WROOM-32 sesuai program Peripheral Test.**
