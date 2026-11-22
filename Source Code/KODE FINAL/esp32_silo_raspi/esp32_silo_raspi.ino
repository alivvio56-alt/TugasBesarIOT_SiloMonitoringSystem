// ============================================================================
// SILO + CONVEYOR PID + INTEGRASI UART RASPBERRY PI 3 (QR SCANNER + LED)
// ============================================================================
// Basis kode : seilo_serial_i2c.ino (Ardhis) — seluruh pin eksisting TETAP SAMA!
// Tambahan   :
//   1. Komunikasi UART2 ke Raspberry Pi 3:
//      - ESP32 GPIO 32 (RX2) <- Raspi Pin 8  (GPIO 14 TXD)
//      - ESP32 GPIO 33 (TX2) -> Raspi Pin 10 (GPIO 15 RXD)
//      - ESP32 GND           <-> Raspi Pin 6 (GND)
//   2. Indikator LED Valid / Invalid (via resistor 220-330 Ohm ke GND):
//      - LED HIJAU = GPIO 13 (Nyala 2 detik saat VALID, lalu conveyor maju)
//      - LED MERAH = GPIO 14 (Nyala terus saat INVALID sampai barang diangkat)
//
// Alur Kerja Conveyor + Raspi:
//   1. Benda diletakkan dekat sensor -> jeda 2 dtk -> PID maju ke SETPOINT (21.5 cm)
//   2. Sampai di titik kamera -> Rem aktif 150ms -> State WAIT_SCAN
//   3. ESP32 kirim {"evt":"SCAN_REQ","scan_id":N,"dist":21.5} ke Raspi via UART2
//   4. Raspi scan QR & kirim feedback:
//      - SCAN_VALID   -> State VALID_HOLD -> LED HIJAU nyala 2 detik -> RELEASE masuk silo -> IDLE
//      - SCAN_INVALID -> State INVALID_HOLD -> LED MERAH nyala terus & conveyor DIAM
//                        sampai barang diangkat (!present) & kamera clear (ITEM_CLEARED) -> IDLE
//
// Library Arduino IDE:
//   ESP32Servo, DHT sensor library (+ Adafruit Unified Sensor), LiquidCrystal_I2C
// Serial Monitor USB: 115200 baud, Newline (ketik "help" untuk daftar perintah)
// ============================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <math.h>

// ==========================================
// 1. PIN CONFIGURATION (SAMA DENGAN RIG ARDHIS + UART2 + LED)
// ==========================================

// MOTOR DRIVER L298N
#define ENA           25
#define IN1           26
#define IN2           27

// RELAY 2 KIPAS (BALIK: relay ON = kipas MATI, relay OFF = kipas NYALA)
#define RELAY_PIN     19

// SERVO VENTILASI
const int servoPin  = 18;

// HC-SR04 (JANGAN UBAH: tetap di 16 & 17 sesuai wiring aktif)
#define TRIG_PIN      16
#define ECHO_PIN      17

// DHT22 & MQ GAS
#define DHT_PIN        4
#define DHTTYPE    DHT22
#define MQ_PIN        34

// LCD I2C 16x2
#define I2C_SDA       21
#define I2C_SCL       22

// UART2 KE RASPBERRY PI 3 (GPIO Matrix ESP32: bebas pakai pin 32 & 33)
#define RASPI_RX_PIN  32   // Sambung ke Pin 8 Raspi (TXD)
#define RASPI_TX_PIN  33   // Sambung ke Pin 10 Raspi (RXD)
#define RASPI_BAUD 115200

// INDIKATOR LED VALID & INVALID (Active-HIGH via resistor 220-330 Ohm)
#define LED_GREEN_PIN 13   // LED Hijau: Barang VALID
#define LED_RED_PIN   14   // LED Merah: Barang TIDAK VALID / KARANTINA

// ==========================================
// 2. OBJEK HARDWARE
// ==========================================

HardwareSerial RaspiSerial(2);
Servo myServo;
DHT dht(DHT_PIN, DHTTYPE);
LiquidCrystal_I2C* lcd = nullptr;

// ==========================================
// 3. PARAMETER MONITORING SILO
// ==========================================

const bool RELAY_ACTIVE_LOW = true;

float T_ON     = 28.0f;    // kipas ON bila suhu > ini
float T_OFF    = 27.0f;    // kipas OFF lagi bila suhu < ini
float H_OPEN   = 50.0f;    // ventilasi buka bila kelembapan < ini
float H_TARGET = 70.0f;    // tutup lagi setelah kelembapan >= ini
float H_MAX    = 80.0f;    // kelembapan > ini: ventilasi pasti tutup
float G_ON     = 2000;     // gas > ini: kipas ON + ventilasi buka
float G_OFF    = 1800;     // gas < ini: alarm gas berakhir

const int   SERVO_OPEN    = 90;
const int   SERVO_CLOSE   = 0;
const float GAS_EMA_ALPHA = 0.15f;

// ==========================================
// 4. PARAMETER PID CONVEYOR & SCAN RASPI
// ==========================================

float SETPOINT_CM     = 20.5f; // posisi berhenti tepat di tengah ROI kamera (cm, aman di bawah dinding 23 cm)
float TOL_CM          = 0.4f;
float START_DETECT_CM = 17.5f; // zona awal masuk: benda di IDLE harus <= ini agar conveyor mulai jalan
float DETECT_CM       = 22.0f; // batas maksimum area conveyor+kamera (>22.0 cm dianggap dinding/belt kosong)
float DMIN_CM         = 1.0f;  // ...dan > ini
float KP = 6.5f, KI = 0.45f, KD = 4.0f;
float PWM_MIN         = 62;    // PWM merayap halus saat mendekati kamera (turun bertahap dari PWM_MAX)
float PWM_MAX         = 125;   // PWM awal saat benda masih jauh dari kamera
float REL_PWM         = 110;   // PWM saat maju membuang barang valid ke silo
int   PERIOD_MS       = 60;    // interval pembacaan jarak (median 5 ping ~45ms)
float LEAD_CM         = 0.35f; // berhenti sedikit lebih awal (kecil karena belt sudah pelan mendekati target)
int   BRAKE_MS        = 180;   // durasi rem dinamis aktif L298N
int   PRESENT_N       = 20;    // bacaan valid berturut-turut -> hadir
int   ABSENT_N        = 10;    // bacaan kosong berturut-turut -> hilang
int   STOP_N          = 2;     // konfirmasi di titik kamera sebelum rem
float MAX_JUMP_CM     = 6.0f;  // penolak lonjakan noise
int   JUMP_ACCEPT_N   = 8;
float REL_MAX_MS      = 2000;  // durasi maju membuang barang valid ke silo
float REARM_CM        = 5.0f;
float DETECT_DELAY_MS = 2000;  // jeda setelah benda terdeteksi sebelum PID maju

// Anti-stall (otomatis dorong jika sabuk seret/nyangkut, lalu langsung melambat halus lagi begitu bergerak)
uint32_t STALL_CHECK_MS = 400;  // jika jarak bergerak < 0.35 cm selama 400 ms -> tambah torsi sementara
float    STALL_STEP_PWM = 12.0f;
float    STALL_MAX_PWM  = 45.0f;

// Parameter Handshake Raspi & LED
uint32_t VALID_LED_MS        = 2000; // LED hijau nyala 2 detik sebelum conveyor lanjut jalan
uint32_t SCAN_REQ_RETRY_MS   = 1500; // kirim ulang SCAN_REQ tiap 1.5 dtk bila belum dijawab Raspi
uint32_t CLEAR_FALLBACK_MS   = 2500; // pengaman: jika !present > 2.5 dtk & Raspi offline, tetap clear

// ==========================================
// 5. STATE MONITORING & LED
// ==========================================

bool fanAuto = true;
bool ventAuto = true;
bool fanManualOn = false;
int  ventManualAngle = SERVO_CLOSE;

bool fanOn = false;
int  servoAngle = SERVO_CLOSE;

float lastTemp = NAN, lastHum = NAN;
uint32_t lastDhtOkMs = 0;
bool dhtEverOk = false;
bool dhtOk = false;

float gasRaw = 0, gasF = 0;
bool gasInit = false;

bool tempHot = false, gasAlarm = false, humOpen = false;

bool logOn    = true;
bool pidLogOn = false; // default off agar log handshake Raspi terbaca bersih (ketik 'pidlog on' jika perlu)

void setLeds(bool greenOn, bool redOn) {
  digitalWrite(LED_GREEN_PIN, greenOn ? HIGH : LOW);
  digitalWrite(LED_RED_PIN,   redOn   ? HIGH : LOW);
}

// ==========================================
// 6. RELAY & SERVO VENTILASI
// ==========================================

void setRelay(bool relayEnergized) {
  digitalWrite(RELAY_PIN, relayEnergized ? (RELAY_ACTIVE_LOW ? LOW : HIGH)
                                         : (RELAY_ACTIVE_LOW ? HIGH : LOW));
}

void setFan(bool on) {
  if (on == fanOn) return;
  fanOn = on;
  setRelay(!on); // kipas ON = relay OFF (NC)
  Serial.printf("[FAN] %s (relay %s)\n", on ? "NYALA" : "MATI", on ? "OFF" : "ON");
}

void setServo(int angle) {
  if (angle == servoAngle) return;
  servoAngle = angle;
  myServo.write(angle);
  Serial.printf("[VENT] %s (%d derajat)\n", angle > 0 ? "TERBUKA" : "TERTUTUP", angle);
}

// ==========================================
// 7. SENSOR LINGKUNGAN (DHT22 + MQ-2)
// ==========================================

void sampleGas() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(MQ_PIN);
  gasRaw = sum / 8.0f;
  if (!gasInit) { gasF = gasRaw; gasInit = true; }
  else gasF = GAS_EMA_ALPHA * gasRaw + (1.0f - GAS_EMA_ALPHA) * gasF;
}

void sampleDht() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  bool valid = !isnan(t) && !isnan(h) && t > -20 && t < 80 && h >= 0 && h <= 100;
  if (valid) {
    lastTemp = t;
    lastHum = h;
    lastDhtOkMs = millis();
    dhtEverOk = true;
  } else {
    Serial.println("[DHT] ERROR: bacaan tidak valid, pakai data terakhir");
  }
  dhtOk = dhtEverOk && (millis() - lastDhtOkMs < 10000);
}

void evaluateEnv() {
  if (!gasAlarm && gasF > G_ON) gasAlarm = true;
  else if (gasAlarm && gasF < G_OFF) gasAlarm = false;

  if (dhtOk) {
    if (!tempHot && lastTemp > T_ON) tempHot = true;
    else if (tempHot && lastTemp < T_OFF) tempHot = false;

    if (!humOpen && lastHum < H_OPEN) humOpen = true;
    else if (humOpen && lastHum >= H_TARGET) humOpen = false;
    if (lastHum > H_MAX) humOpen = false;
  }

  if (fanAuto) setFan(tempHot || gasAlarm);
  else         setFan(fanManualOn);

  if (ventAuto) setServo((gasAlarm || humOpen) ? SERVO_OPEN : SERVO_CLOSE);
  else          setServo(ventManualAngle);
}

// ==========================================
// 8. MOTOR CONVEYOR (L298N)
// ==========================================

int  pwmNow     = 0;
bool motorState = false;
int  motorSpeed = 100;
bool convAuto   = true;

void motorRun(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwm);
  pwmNow = pwm;
  motorState = (pwm > 0);
}

void motorBrake() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  pwmNow = 0;
  motorState = false;
}

void motorOff() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  pwmNow = 0;
  motorState = false;
}

// ==========================================
// 9. HC-SR04 (MEDIAN 5 PING + FILTER)
// ==========================================

float pingOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 12000);
  if (d == 0) return -1;
  return d * 0.0343f / 2.0f;
}

float readDistance() {
  float v[5];
  int n = 0;
  for (int i = 0; i < 5; i++) {
    float d = pingOnce();
    if (d > DMIN_CM) v[n++] = d;
    delay(6);
  }
  if (n < 3) return -1;
  for (int a = 1; a < n; a++) {
    float x = v[a]; int b = a - 1;
    while (b >= 0 && v[b] > x) { v[b + 1] = v[b]; b--; }
    v[b + 1] = x;
  }
  return v[n / 2];
}

float dist = -1;
float rawDist = -1;
bool  present = false;
int   presentCnt = 0, absentCnt = 0, jumpCnt = 0;

void updateDistance() {
  float d = readDistance();
  rawDist = d;
  bool valid = (d > DMIN_CM && d <= DETECT_CM);

  if (valid) {
    if (present && dist >= 0 && fabsf(d - dist) > MAX_JUMP_CM && jumpCnt < JUMP_ACCEPT_N) {
      jumpCnt++;
      return;
    }
    jumpCnt = 0;
    dist = (dist < 0) ? d : (0.3f * dist + 0.7f * d);
    presentCnt++; absentCnt = 0;
    if (presentCnt >= PRESENT_N) present = true;
  } else {
    absentCnt++; presentCnt = 0;
    if (absentCnt >= ABSENT_N) { present = false; dist = -1; jumpCnt = 0; }
  }
}

// ==========================================
// 10. PID CONVEYOR + PERLAMBATAN HALUS + ANTI-STALL
// ==========================================

float    integ        = 0;
float    prevMeas     = 0;
bool     firstPid     = true;
float    stallRefDist = -1.0f;
uint32_t stallRefMs   = 0;
float    stallBoost   = 0.0f;

void pidReset() {
  integ = 0;
  firstPid = true;
  stallRefDist = -1.0f;
  stallRefMs = 0;
  stallBoost = 0.0f;
}

int pidStep(float meas, float dt, uint32_t now) {
  if (dt <= 0.001f) dt = PERIOD_MS / 1000.0f;
  float e = SETPOINT_CM - meas; // + = belum sampai kamera -> maju; makin dekat makin kecil
  if (e < 0.0f) e = 0.0f;

  float dMeas = firstPid ? 0.0f : (meas - prevMeas) / dt;
  if (firstPid) {
    stallRefDist = meas;
    stallRefMs = now;
  }
  firstPid = false;
  prevMeas = meas;

  // Deteksi belt nyangkut / seret: jika tidak bergeser >= 0.35 cm dalam STALL_CHECK_MS,
  // naikkan boost bertahap; begitu bergerak lagi, langsung redam kembali agar tetap pelan halus!
  if (fabsf(meas - stallRefDist) >= 0.35f) {
    stallRefDist = meas;
    stallRefMs = now;
    stallBoost *= 0.65f;
    if (stallBoost < 1.0f) stallBoost = 0.0f;
  } else if (e > TOL_CM && (now - stallRefMs >= STALL_CHECK_MS)) {
    stallBoost = min(stallBoost + STALL_STEP_PWM, STALL_MAX_PWM);
    stallRefMs = now;
  }

  // Integral dengan anti-windup ketat (hanya bantu torsi gesekan, tidak bikin overshoot/bablas)
  integ += e * dt;
  if (KI > 0.0001f) {
    float maxI = 35.0f / KI;
    integ = constrain(integ, 0.0f, maxI);
  }

  // Feed-forward PWM_MIN + PID + Anti-Stall:
  // Jauh (e ~10cm) -> ~PWM_MAX (125), makin dekat (e -> 0.5cm) -> turun halus ke PWM_MIN (62)
  float u = PWM_MIN + (KP * e) + (KI * integ) - (KD * dMeas) + stallBoost;
  return constrain((int)roundf(u), (int)PWM_MIN, (int)PWM_MAX);
}

// ==========================================
// 11. STATE MACHINE CONVEYOR + HANDSHAKE RASPI
// ==========================================

enum State { IDLE, APPROACH, WAIT_SCAN, VALID_HOLD, INVALID_HOLD, RELEASE };
State    state           = IDLE;
uint32_t stateSince      = 0;
uint32_t detectedAt      = 0;
uint32_t lastScanReqMs   = 0;
uint32_t absentSinceMs   = 0;
uint32_t currentScanId   = 0;
int      stopCnt         = 0;
bool     needClear       = false;
bool     raspiClearedEvt = false;
String   lastBoxId       = "";

const char* stateName() {
  switch (state) {
    case IDLE:         return "IDLE";
    case APPROACH:     return "APPROACH";
    case WAIT_SCAN:    return "WAIT_SCAN";
    case VALID_HOLD:   return "VALID_HOLD";
    case INVALID_HOLD: return "INVALID_HOLD";
    case RELEASE:      return "RELEASE";
    default:           return "?";
  }
}

void sendScanRequest() {
  lastScanReqMs = millis();
  RaspiSerial.printf("{\"evt\":\"SCAN_REQ\",\"scan_id\":%lu,\"dist\":%.1f}\n",
                     (unsigned long)currentScanId, dist);
  Serial.printf("[ESP -> RASPI] SCAN_REQ #%lu (dist=%.1f cm)\n",
                (unsigned long)currentScanId, dist);
}

void sendTelemetryToRaspi() {
  float tOut = dhtOk ? lastTemp : -1.0f;
  float hOut = dhtOk ? lastHum  : -1.0f;
  const char* ctrlMode = (fanAuto && ventAuto && convAuto) ? "AUTO" : "MANUAL";
  RaspiSerial.printf(
    "{\"evt\":\"TELEM\",\"state\":\"%s\",\"mode\":\"%s\",\"scan_id\":%lu,\"present\":%d,\"dist\":%.1f,\"pwm\":%d,\"temp\":%.1f,\"hum\":%.1f,\"gas\":%.0f,\"fan\":%d,\"vent\":%d,\"led_g\":%d,\"led_r\":%d}\n",
    stateName(), ctrlMode, (unsigned long)currentScanId, present ? 1 : 0, dist, pwmNow,
    tOut, hOut, gasF, fanOn ? 1 : 0, servoAngle,
    digitalRead(LED_GREEN_PIN), digitalRead(LED_RED_PIN)
  );
}

void setState(int s) {
  state = (State)s;
  stateSince = millis();
  Serial.printf(">> STATE: %s\n", stateName());

  if (state == IDLE || state == APPROACH) {
    setLeds(false, false);
  } else if (state == WAIT_SCAN) {
    setLeds(false, false);
    currentScanId++;
    lastBoxId = "";
    raspiClearedEvt = false;
    sendScanRequest();
  } else if (state == VALID_HOLD) {
    setLeds(true, false);   // LED HIJAU ON selama 2 detik
    motorOff();
  } else if (state == INVALID_HOLD) {
    setLeds(false, true);   // LED MERAH ON terus sampai barang diangkat
    motorOff();
    raspiClearedEvt = false;
    absentSinceMs = 0;
  } else if (state == RELEASE) {
    setLeds(false, false);
  }

  sendTelemetryToRaspi();
}

void convReset() {
  motorOff();
  pidReset();
  stopCnt = 0;
  detectedAt = 0;
  needClear = false;
  raspiClearedEvt = false;
  absentSinceMs = 0;
  setState(IDLE);
}

// Dipanggil saat Raspi mengirim hasil scan VALID
void onScanValid(const String& boxId) {
  if (state != WAIT_SCAN && state != INVALID_HOLD) return;
  lastBoxId = boxId;
  Serial.printf("[RASPI -> ESP] SCAN_VALID (%s) -> LED HIJAU 2 detik lalu MAJU\n",
                lastBoxId.length() ? lastBoxId.c_str() : "OK");
  setState(VALID_HOLD);
}

// Dipanggil saat Raspi mengirim hasil scan INVALID
void onScanInvalid(const String& reason) {
  if (state != WAIT_SCAN) return;
  Serial.printf("[RASPI -> ESP] SCAN_INVALID (%s) -> LED MERAH ON & CONVEYOR DIAM!\n",
                reason.length() ? reason.c_str() : "Tidak valid");
  setState(INVALID_HOLD);
}

// Dipanggil saat Raspi mengonfirmasi barang invalid sudah hilang dari kamera
void onItemClearedFromRaspi() {
  if (state != INVALID_HOLD) return;
  raspiClearedEvt = true;
  Serial.println("[RASPI -> ESP] ITEM_CLEARED (kamera tidak lagi melihat barang invalid)");
}

void convTick(uint32_t now, float dt) {
  switch (state) {

    case IDLE:
      motorOff();
      if (needClear) {
        if (!present || (dist >= 0 && dist < SETPOINT_CM - REARM_CM)) needClear = false;
        detectedAt = 0;
      } else if (present && dist > DMIN_CM && dist <= START_DETECT_CM) {
        if (detectedAt == 0) {
          detectedAt = now;
          Serial.printf(">> DETECTED di %.1f cm (<= %.1f cm), tunggu jeda sebelum jalan\n", dist, START_DETECT_CM);
        } else if (now - detectedAt >= (uint32_t)DETECT_DELAY_MS) {
          detectedAt = 0;
          pidReset(); stopCnt = 0;
          setState(APPROACH);
        }
      } else {
        detectedAt = 0;
      }
      break;

    case APPROACH:
      if (!present) { motorOff(); setState(IDLE); break; }
      if (dist >= 0 && dist >= SETPOINT_CM - TOL_CM - LEAD_CM) {
        if (++stopCnt >= STOP_N) {
          motorBrake();
          setState(WAIT_SCAN);
        } else {
          motorRun(max((int)PWM_MIN - 8, 50));
        }
      } else {
        stopCnt = 0;
        if (dist >= 0) motorRun(pidStep(dist, dt, now));
      }
      break;

    case WAIT_SCAN:
      // Rem aktif selama BRAKE_MS (150 ms), lalu tahan diam menunggu feedback Raspi
      if (now - stateSince < (uint32_t)BRAKE_MS) motorBrake();
      else motorOff();

      // Jika barang diangkat manual saat sedang menunggu scan, batalkan & kembali IDLE
      if (!present) {
        RaspiSerial.printf("{\"evt\":\"SCAN_CANCEL\",\"scan_id\":%lu}\n", (unsigned long)currentScanId);
        Serial.println(">> Barang diangkat saat WAIT_SCAN -> kembali IDLE");
        setState(IDLE);
        break;
      }

      // Kirim ulang SCAN_REQ secara berkala agar kebal terhadap paket hilang
      if (now - lastScanReqMs >= SCAN_REQ_RETRY_MS) {
        sendScanRequest();
      }
      break;

    case VALID_HOLD:
      // LED Hijau menyala selama 2 detik (VALID_LED_MS), setelah itu conveyor lanjut maju
      motorOff();
      if (now - stateSince >= VALID_LED_MS) {
        motorRun((int)REL_PWM);
        setState(RELEASE);
      }
      break;

    case INVALID_HOLD:
      // LED Merah terus menyala, conveyor diam sampai barang dikeluarkan & kamera clear
      motorOff();
      if (!present) {
        if (absentSinceMs == 0) absentSinceMs = now;
        bool fallbackReady = (now - absentSinceMs >= CLEAR_FALLBACK_MS);
        if (raspiClearedEvt || fallbackReady) {
          Serial.println(">> Barang invalid sudah diangkat & clear -> LED MERAH OFF -> siap barang baru!");
          needClear = false;
          setState(IDLE);
        }
      } else {
        absentSinceMs = 0;
        // Jika kamera Raspi sudah mengonfirmasi belt kosong (ITEM_CLEARED), keluar dari INVALID_HOLD
        if (raspiClearedEvt) {
          Serial.println(">> Kamera Raspi konfirmasi belt KOSONG (ITEM_CLEARED) -> LED MERAH OFF -> IDLE");
          needClear = false;
          setState(IDLE);
        }
      }
      break;

    case RELEASE:
      if (now - stateSince >= (uint32_t)REL_MAX_MS) {
        motorOff();
        needClear = true;
        setState(IDLE);
      }
      break;
  }
}

// ==========================================
// 12. LCD I2C 16x2 (4 LAYAR + PRIORITAS STATUS SCAN)
// ==========================================

char    lcdLast[2][17] = {"", ""};
bool    lcdOk   = false;
uint8_t lcdAddr = 0;

void lcdWriteRow(int row, const char* text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16.16s", text);
  if (strcmp(buf, lcdLast[row]) == 0) return;
  strcpy(lcdLast[row], buf);
  lcd->setCursor(0, row);
  lcd->print(buf);
}

void updateLcd() {
  if (!lcdOk) return;
  char l1[24], l2[24];

  // Prioritaskan tampilan scan saat sedang memproses barang di depan kamera
  if (state == WAIT_SCAN) {
    snprintf(l1, sizeof(l1), "SCANNING QR...");
    snprintf(l2, sizeof(l2), "D:%.1fcm #%lu", dist, (unsigned long)currentScanId);
    lcdWriteRow(0, l1); lcdWriteRow(1, l2);
    return;
  }
  if (state == VALID_HOLD) {
    snprintf(l1, sizeof(l1), "VALID! [HIJAU]");
    snprintf(l2, sizeof(l2), "%s", lastBoxId.length() ? lastBoxId.c_str() : "Lanjut ke Silo");
    lcdWriteRow(0, l1); lcdWriteRow(1, l2);
    return;
  }
  if (state == INVALID_HOLD) {
    snprintf(l1, sizeof(l1), "INVALID! [MERAH]");
    snprintf(l2, sizeof(l2), "Angkat Barang!");
    lcdWriteRow(0, l1); lcdWriteRow(1, l2);
    return;
  }

  int screen = (millis() / 2000) % 4;
  switch (screen) {
    case 0:
      if (dhtOk) snprintf(l1, sizeof(l1), "T:%.1fC H:%.0f%%", lastTemp, lastHum);
      else       snprintf(l1, sizeof(l1), "T:ERR H:ERR");
      snprintf(l2, sizeof(l2), "DHT:%s", dhtOk ? "OK" : "ERROR");
      break;
    case 1:
      snprintf(l1, sizeof(l1), "Gas:%.0f", gasF);
      snprintf(l2, sizeof(l2), "Status:%s", gasAlarm ? "ALARM" : "OK");
      break;
    case 2:
      snprintf(l1, sizeof(l1), "Fan:%s/%c", fanOn ? "ON" : "OFF", fanAuto ? 'A' : 'M');
      snprintf(l2, sizeof(l2), "Vent:%d/%c", servoAngle, ventAuto ? 'A' : 'M');
      break;
    default:
      snprintf(l1, sizeof(l1), "Conv:%s", convAuto ? stateName() : "MANUAL");
      if (dist >= 0) snprintf(l2, sizeof(l2), "D:%.1fcm", dist);
      else           snprintf(l2, sizeof(l2), "D:--");
      break;
  }
  lcdWriteRow(0, l1);
  lcdWriteRow(1, l2);
}

bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void i2cScan() {
  Serial.println("Scan I2C (SDA=21, SCL=22)...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPing(a)) { Serial.printf("  ditemukan: 0x%02X\n", a); found++; }
  }
  if (!found) Serial.println("  tidak ada perangkat I2C");
}

void lcdInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (i2cPing(0x27)) lcdAddr = 0x27;
  else if (i2cPing(0x3F)) lcdAddr = 0x3F;
  if (lcdAddr == 0) {
    Serial.println("LCD I2C tidak ditemukan di 0x27/0x3F.");
    return;
  }
  lcd = new LiquidCrystal_I2C(lcdAddr, 16, 2);
  lcd->init();
  lcd->backlight();
  lcd->clear();
  lcdOk = true;
  Serial.printf("LCD I2C aktif di alamat 0x%02X\n", lcdAddr);
  lcdWriteRow(0, "Silo ESP-RASPI");
  lcdWriteRow(1, "UART Ready...");
}

// ==========================================
// 13. PARSER PERINTAH DARI RASPBERRY PI (UART2)
// ==========================================

String extractJsonField(const String& src, const char* key) {
  String pat = String("\"") + key + "\":\"";
  int idx = src.indexOf(pat);
  if (idx < 0) return "";
  int start = idx + pat.length();
  int end = src.indexOf('"', start);
  if (end < 0) return "";
  return src.substring(start, end);
}

void processRaspiMessage(String msg) {
  msg.trim();
  if (msg.length() == 0) return;

  String upper = msg;
  upper.toUpperCase();

  if (upper.indexOf("SCAN_VALID") >= 0 || upper.indexOf("SCAN_PASS") >= 0) {
    onScanValid(extractJsonField(msg, "id"));
  } else if (upper.indexOf("SCAN_INVALID") >= 0 || upper.indexOf("SCAN_HOLD") >= 0) {
    onScanInvalid(extractJsonField(msg, "reason"));
  } else if (upper.indexOf("ITEM_CLEARED") >= 0 || upper.indexOf("SCAN_CLEAR") >= 0) {
    onItemClearedFromRaspi();
  } else if (upper.indexOf("STOP_CONVEYOR") >= 0) {
    convAuto = false;
    motorOff();
    setLeds(false, false);
    Serial.println("[RASPI -> ESP] STOP_CONVEYOR (manual)");
  } else if (upper.indexOf("RUN_CONVEYOR") >= 0) {
    convAuto = false;
    motorRun(motorSpeed);
    Serial.println("[RASPI -> ESP] RUN_CONVEYOR (manual)");
  } else if (upper.indexOf("CONV_AUTO") >= 0 || upper.indexOf("AUTO_MODE") >= 0) {
    convAuto = true;
    convReset();
    Serial.println("[RASPI -> ESP] CONV_AUTO");
  } else if (upper.indexOf("FAN_ON") >= 0) {
    fanAuto = false; fanManualOn = true; evaluateEnv();
    Serial.println("[RASPI -> ESP] FAN_ON");
  } else if (upper.indexOf("FAN_OFF") >= 0) {
    fanAuto = false; fanManualOn = false; evaluateEnv();
    Serial.println("[RASPI -> ESP] FAN_OFF");
  } else if (upper.indexOf("VENT_OPEN") >= 0) {
    ventAuto = false; ventManualAngle = SERVO_OPEN; evaluateEnv();
    Serial.println("[RASPI -> ESP] VENT_OPEN");
  } else if (upper.indexOf("VENT_CLOSE") >= 0) {
    ventAuto = false; ventManualAngle = SERVO_CLOSE; evaluateEnv();
    Serial.println("[RASPI -> ESP] VENT_CLOSE");
  } else if (upper.indexOf("ALL_MANUAL") >= 0) {
    fanAuto = false; fanManualOn = fanOn;
    ventAuto = false; ventManualAngle = servoAngle;
    convAuto = false;
    sendTelemetryToRaspi();
    Serial.println("[RASPI -> ESP] ALL_MANUAL");
  } else if (upper.indexOf("ALL_AUTO") >= 0) {
    fanAuto = true; ventAuto = true; convAuto = true; evaluateEnv(); convReset();
    Serial.println("[RASPI -> ESP] ALL_AUTO");
  } else if (upper.indexOf("PING") >= 0) {
    sendTelemetryToRaspi();
  }
}

// ==========================================
// 14. TAMPILAN & PERINTAH SERIAL MONITOR USB
// ==========================================

void printStatus() {
  Serial.println("------------------------------------------------------------");
  if (dhtOk) Serial.printf("Suhu/Lembab: %.1f C / %.1f %%\n", lastTemp, lastHum);
  else       Serial.println("Suhu/Lembab: DHT ERROR");
  Serial.printf("Gas        : %.0f (alarm=%s)\n", gasF, gasAlarm ? "YA" : "tidak");
  Serial.printf("Kipas/Vent : Fan=%s[%s] | Vent=%d deg[%s]\n",
                fanOn ? "ON" : "OFF", fanAuto ? "A" : "M",
                servoAngle, ventAuto ? "A" : "M");
  Serial.printf("Conveyor   : %s [%s] pwm=%d D=%.1f cm (SP=%.1f)\n",
                stateName(), convAuto ? "AUTO" : "MANUAL", pwmNow, dist, SETPOINT_CM);
  Serial.printf("LED        : Hijau=%d | Merah=%d | ScanID=#%lu\n",
                digitalRead(LED_GREEN_PIN), digitalRead(LED_RED_PIN), (unsigned long)currentScanId);
  Serial.println("------------------------------------------------------------");
}

void printHelp() {
  Serial.println("\n======= SILO ESP32 <-> RASPBERRY PI 3 =======");
  Serial.println("Simulasi Raspi (tanpa kabel): valid | invalid | clear");
  Serial.println("Conveyor : conv auto | motor on | motor off | motor speed 100");
  Serial.println("Fan/Vent : fan auto|on|off | servo auto|on|off");
  Serial.println("Info     : status | distance | i2c | log on|off | pidlog on|off");
  Serial.println("Tuning   : sp 20.5 | startcm 17.5 | detcm 22 | pwmmin 62 | pwmmax 125 | kp 6.5 | ki 0.45 | kd 4 | lead 0.35 | brake 180");
  Serial.println("=============================================\n");
}

bool setParam(const String& k, float v) {
  if      (k == "ton") T_ON = v;
  else if (k == "toff") T_OFF = v;
  else if (k == "hopen") H_OPEN = v;
  else if (k == "htarget") H_TARGET = v;
  else if (k == "hmax") H_MAX = v;
  else if (k == "gon") G_ON = v;
  else if (k == "goff") G_OFF = v;
  else if (k == "kp") KP = v;
  else if (k == "ki") KI = v;
  else if (k == "kd") KD = v;
  else if (k == "sp") SETPOINT_CM = v;
  else if (k == "startcm") START_DETECT_CM = v;
  else if (k == "detcm") DETECT_CM = v;
  else if (k == "tol") TOL_CM = v;
  else if (k == "detect") DETECT_CM = v;
  else if (k == "pwmmin") PWM_MIN = v;
  else if (k == "pwmmax") PWM_MAX = v;
  else if (k == "relpwm") REL_PWM = v;
  else if (k == "relms") REL_MAX_MS = v;
  else if (k == "dmin") DMIN_CM = v;
  else if (k == "delay") DETECT_DELAY_MS = v;
  else if (k == "period") PERIOD_MS = (int)v;
  else if (k == "lead") LEAD_CM = v;
  else if (k == "brake") BRAKE_MS = (int)v;
  else if (k == "validms") VALID_LED_MS = (uint32_t)v;
  else if (k == "n") PRESENT_N = (int)v;
  else if (k == "m") ABSENT_N = (int)v;
  else if (k == "stopn") STOP_N = (int)v;
  else if (k == "jump") MAX_JUMP_CM = v;
  else return false;
  return true;
}

void processUsbCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  // Simulasi manual feedback Raspi lewat USB Serial Monitor
  if      (cmd == "valid")   { onScanValid("TEST-USB"); }
  else if (cmd == "invalid") { onScanInvalid("TEST-USB"); }
  else if (cmd == "clear")   { onItemClearedFromRaspi(); }

  else if (cmd == "fan auto")   { fanAuto = true; evaluateEnv(); }
  else if (cmd == "fan on")     { fanAuto = false; fanManualOn = true; evaluateEnv(); }
  else if (cmd == "fan off")    { fanAuto = false; fanManualOn = false; evaluateEnv(); }
  else if (cmd == "servo auto") { ventAuto = true; evaluateEnv(); }
  else if (cmd == "servo on")   { ventAuto = false; ventManualAngle = SERVO_OPEN; evaluateEnv(); }
  else if (cmd == "servo off")  { ventAuto = false; ventManualAngle = SERVO_CLOSE; evaluateEnv(); }
  else if (cmd == "conv auto")  { convAuto = true; convReset(); Serial.println("Conveyor AUTO"); }
  else if (cmd == "motor on")   { convAuto = false; motorRun(motorSpeed); }
  else if (cmd == "motor off")  { convAuto = false; motorOff(); }
  else if (cmd.startsWith("motor speed ")) {
    motorSpeed = constrain(cmd.substring(12).toInt(), 0, 255);
    if (!convAuto && motorState) motorRun(motorSpeed);
  }
  else if (cmd == "status")     { printStatus(); }
  else if (cmd == "distance")   { Serial.printf("Distance: %.2f cm\n", readDistance()); }
  else if (cmd == "i2c")        { i2cScan(); }
  else if (cmd == "log on")     { logOn = true; }
  else if (cmd == "log off")    { logOn = false; }
  else if (cmd == "pidlog on")  { pidLogOn = true; }
  else if (cmd == "pidlog off") { pidLogOn = false; }
  else if (cmd == "help")       { printHelp(); }
  else {
    int sp = cmd.indexOf(' ');
    if (sp > 0 && setParam(cmd.substring(0, sp), cmd.substring(sp + 1).toFloat())) {
      Serial.println("[OK] Parameter diperbarui.");
      printStatus();
    } else {
      Serial.println("Command tidak dikenal. Ketik 'help'.");
    }
  }
}

// ==========================================
// 15. SETUP & LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  RaspiSerial.begin(RASPI_BAUD, SERIAL_8N1, RASPI_RX_PIN, RASPI_TX_PIN);

  // LED Indikator (tes nyala singkat 300ms saat boot)
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  setLeds(true, true);
  delay(300);
  setLeds(false, false);

  // SERVO (dialokasikan sebelum analogWrite motor)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
  myServo.write(SERVO_CLOSE);

  // MOTOR L298N
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorOff();

  // RELAY (awal kipas mati -> relay ON)
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(true);

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // DHT22 & MQ-2
  dht.begin();
  pinMode(MQ_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ_PIN, ADC_11db);

  lcdInit();
  Serial.println("\n[READY] SILO ESP32 <-> RASPI 3 (UART2: RX=32, TX=33 | LED_G=13, LED_R=14)");
  printHelp();
}

String usbBuf, raspiBuf;
uint32_t tGas = 0, tDht = 0, tEval = 0, tLog = 0, tPid = 0, tPidLog = 0, tLcd = 0, tTelem = 0;

void loop() {
  // 1. Baca perintah dari USB Serial Monitor
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbBuf.length() > 0) {
        String cmd = usbBuf; usbBuf = "";
        processUsbCommand(cmd);
      }
    } else if (usbBuf.length() < 80) {
      usbBuf += c;
    }
  }

  // 2. Baca feedback / perintah dari Raspberry Pi 3 via UART2
  while (RaspiSerial.available()) {
    char c = (char)RaspiSerial.read();
    if (c == '\n' || c == '\r') {
      if (raspiBuf.length() > 0) {
        String msg = raspiBuf; raspiBuf = "";
        processRaspiMessage(msg);
      }
    } else if (raspiBuf.length() < 180) {
      raspiBuf += c;
    }
  }

  uint32_t now = millis();

  // 3. Sensor & Kendali Lingkungan Silo
  if (now - tGas >= 100)               { tGas = now; sampleGas(); }
  if (now - tDht >= 2000 || tDht == 0) { tDht = now ? now : 1; sampleDht(); }
  if (now - tEval >= 500)              { tEval = now; evaluateEnv(); }

  // 4. Loop PID Conveyor + Handshake Scan
  if (now - tPid >= (uint32_t)PERIOD_MS) {
    float dt = (now - tPid) / 1000.0f;
    tPid = now;
    updateDistance();
    if (convAuto) convTick(now, dt);
  }

  // 5. Kirim telemetri berkala ke Raspi (500 ms)
  if (now - tTelem >= 500) {
    tTelem = now;
    sendTelemetryToRaspi();
  }

  // 6. LCD & Log USB
  if (now - tLcd >= 500) { tLcd = now; updateLcd(); }
  if (logOn && now - tLog >= 2000) {
    tLog = now;
    Serial.printf("State:%s | D=%.1f | P=%d | T=%.1f H=%.0f G=%.0f | LED(G:%d R:%d)\n",
                  stateName(), dist, present ? 1 : 0,
                  dhtOk ? lastTemp : -1.0f, dhtOk ? lastHum : -1.0f, gasF,
                  digitalRead(LED_GREEN_PIN), digitalRead(LED_RED_PIN));
  }
  if (pidLogOn && now - tPidLog >= 200) {
    tPidLog = now;
    Serial.printf("Distance: %.2f cm | filter=%.2f | err=%.2f | %s | pwm=%d | stall=+%.0f\n",
                  rawDist, dist, max(0.0f, SETPOINT_CM - dist), stateName(), pwmNow, stallBoost);
  }
}
