#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>

// Define Pin Sensor
#define DHTPIN 15
#define DHTTYPE DHT22
#define PIN_MQ135 34

// Define Pin Aktuator & Indikator
#define PIN_RELAY 18      // Relay penggerak Exhaust Fan
#define PIN_BUZZER 19     // Buzzer alarm bahaya
#define PIN_LED_RED 4     // LED Merah: Fan ON (Aktif)
#define PIN_LED_GREEN 2   // LED Hijau: Fan OFF (Standby / Aman)

// Konfigurasi Histeresis Ambang Batas
// Level WARNING: Nyala jika tembus batas atas, mati/turun ke normal jika turun di bawah batas bawah
const float TEMP_WARN_ON  = 35.0;
const float TEMP_WARN_OFF = 33.0;   // Histeresis 2 C
const float HUM_WARN_ON   = 65.0;
const float HUM_WARN_OFF  = 62.0;   // Margin histeresis 3%
const int   GAS_WARN_ON   = 2175;
const int   GAS_WARN_OFF  = 2050;   // Margin histeresis ~125 ADC

// Level CRITICAL: Picu bahaya jika tembus batas atas, turun ke warning jika di bawah batas bawah
const float TEMP_CRIT_ON  = 40.0;
const float TEMP_CRIT_OFF = 38.0;
const float HUM_CRIT_ON   = 75.0;
const float HUM_CRIT_OFF  = 72.0;
const int   GAS_CRIT_ON   = 2500;
const int   GAS_CRIT_OFF  = 2375;

// Konfigurasi Jaringan & MQTT Broker
const char* ssid = "CirkitWifi";
const char* password = "";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

// Topik Publish (Data Telemetri) & Subscribe (Perintah Manual)
const char* mqtt_topic_pub = "silo/scm/telemetry/data";
const char* mqtt_topic_sub = "silo/scm/telemetry/control";

WiFiClient espClient; // buat koneksi tcp dari esp
PubSubClient client(espClient);// buat objek mqtt
DHT dht(DHTPIN, DHTTYPE); // buat objek sensor 

unsigned long lastPublish = 0; //simpan waktu data terakhir dikirim
const long interval = 3000; // sensor dikirim tiap 3detik

// Variabel status sistem
bool isManualMode = false;   // false = AUTO, true = MANUAL
bool fanActive = false;
bool buzzerActive = false;
String currentRiskLevel = "NORMAL"; // Menyimpan state status risiko terakhir

// Fungsi pengatur status Fan (Relay dan kedua LED Indikator)
void updateFanOutputs(bool state) {
  fanActive = state;
  digitalWrite(PIN_RELAY, state ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, state ? HIGH : LOW);     // Merah ON saat kipas aktif
  digitalWrite(PIN_LED_GREEN, state ? LOW : HIGH);   // Hijau ON saat kipas mati
}

// Handler Callback saat menerima pesan MQTT perintah kontrol manual
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {    // ubah payload jadi string
    message += (char)payload[i];
  }
  Serial.print("\n[MQTT SUB] Perintah diterima: ");
  Serial.println(message);

  StaticJsonDocument<200> doc;                   // baca json
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.println("Format JSON perintah tidak valid!");
    return;  //kalo json salah
  }

  // 1. Kontrol Mode Operasi (AUTO / MANUAL)
  if (doc.containsKey("mode")) {
    String modeVal = doc["mode"].as<String>();
    if (modeVal.equalsIgnoreCase("MANUAL")) { // manual / MANUAL / mAnUAl tetep sama dibaca true
      isManualMode = true;
      Serial.println("[MODE] Beralih ke kontrol MANUAL oleh operator.");
    } else if (modeVal.equalsIgnoreCase("AUTO")) {
      isManualMode = false;
      Serial.println("[MODE] Beralih ke kontrol OTOMATIS sensor.");
    }
  }

  // 2. Kontrol Manual Kipas Exhaust & LED
  if (doc.containsKey("fan")) {
    String fanCmd = doc["fan"].as<String>();
    isManualMode = true;
    
    if (fanCmd.equalsIgnoreCase("ON")) {   //equalsignore : abaikan huruf besar/kecil
      updateFanOutputs(true);
      Serial.println("[MANUAL] Kipas Exhaust & LED Merah AKTIF.");
    } else if (fanCmd.equalsIgnoreCase("OFF")) {
      updateFanOutputs(false);
      Serial.println("[MANUAL] Kipas Exhaust MATI & LED Hijau AKTIF.");
    }
  }
}

void setupWifi() {
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung! IP: " + WiFi.localIP().toString());
}

void checkWifiAndMqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi terputus, mencoba menghubungkan ulang...");
    WiFi.begin(ssid, password);
    unsigned long startAttempt = millis(); // mencatat waktu internal esp pas baru putus
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
      delay(300);
      Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) return;
  }

  if (!client.connected()) {
    String clientId = "ESP32-SiloSCM-" + String(random(0xffff), HEX);
    Serial.print("Menghubungkan ke HiveMQ...");
    if (client.connect(clientId.c_str())) {
      Serial.println(" Berhasil!");
      client.subscribe(mqtt_topic_sub);
      Serial.print("Subscribed ke topik kontrol: ");
      Serial.println(mqtt_topic_sub);
    } else {
      Serial.print(" Gagal, rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  // Status awal (Fan mati, LED hijau menyala, buzzer mati)
  updateFanOutputs(false);
  digitalWrite(PIN_BUZZER, LOW);

  setupWifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

void loop() {
  checkWifiAndMqtt();
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastPublish >= interval) {
    lastPublish = currentMillis;

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int gasRaw = analogRead(PIN_MQ135);

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Gagal membaca sensor DHT22!");
      return;
    }

    // LOGIKA EVALUASI DENGAN HISTERESIS 
    // Suhu, kelembapan, atau gas dapat menaikkan status risiko.
    if (currentRiskLevel == "CRITICAL") {
      // Turun dari CRITICAL hanya setelah semua parameter kritis pulih.
      if (temperature < TEMP_CRIT_OFF && humidity < HUM_CRIT_OFF && gasRaw < GAS_CRIT_OFF) {
        if (temperature >= TEMP_WARN_ON || humidity >= HUM_WARN_ON || gasRaw >= GAS_WARN_ON) {
          currentRiskLevel = "WARNING";
        } else if (temperature < TEMP_WARN_OFF && humidity < HUM_WARN_OFF && gasRaw < GAS_WARN_OFF) {
          currentRiskLevel = "NORMAL";
        }
      }
    } else if (currentRiskLevel == "WARNING") {
      if (temperature > TEMP_CRIT_ON || humidity > HUM_CRIT_ON || gasRaw > GAS_CRIT_ON) {
        currentRiskLevel = "CRITICAL";
      } else if (temperature < TEMP_WARN_OFF && humidity < HUM_WARN_OFF && gasRaw < GAS_WARN_OFF) {
        currentRiskLevel = "NORMAL";
      }
    } else { // NORMAL
      if (temperature > TEMP_CRIT_ON || humidity > HUM_CRIT_ON || gasRaw > GAS_CRIT_ON) {
        currentRiskLevel = "CRITICAL";
      } else if (temperature > TEMP_WARN_ON || humidity > HUM_WARN_ON || gasRaw > GAS_WARN_ON) {
        currentRiskLevel = "WARNING";
      }
    }

    //  LOGIKA KONTROL SISTEM 
    if (!isManualMode) {
      // MODE AUTO (Sensor-driven)
      if (currentRiskLevel == "CRITICAL") {
        updateFanOutputs(true);
        buzzerActive = true;
      } else if (currentRiskLevel == "WARNING") {
        updateFanOutputs(true);
        buzzerActive = false;
      } else {
        updateFanOutputs(false);
        buzzerActive = false;
      }

      } else {
        // MODE MANUAL (Operator-driven, alarm tetap aktif saat WARNING/CRITICAL)
        buzzerActive = (currentRiskLevel == "WARNING" || currentRiskLevel == "CRITICAL");
      }
      
      // Piezo diberi frekuensi agar berbunyi; warning dan critical beda nada.
      if (currentRiskLevel == "CRITICAL") {
        buzzerActive = true;
        tone(PIN_BUZZER, 2500);
      } else if (currentRiskLevel == "WARNING") {
        buzzerActive = true;
        tone(PIN_BUZZER, 1200);
      } else {
        buzzerActive = false;
        noTone(PIN_BUZZER);
      }

    // Kirim payload data telemetri ke HiveMQ
    StaticJsonDocument<256> doc;
    doc["device_id"] = "SILO-ZONE-01";
    doc["control_mode"] = isManualMode ? "MANUAL" : "AUTO";
    doc["temperature"] = serialized(String(temperature, 1));
    doc["humidity"] = serialized(String(humidity, 1));
    doc["gas_level"] = gasRaw;
    doc["exhaust_fan"] = fanActive;
    doc["buzzer_alarm"] = buzzerActive;
    doc["scm_risk_level"] = currentRiskLevel;

    char buffer[256];
    serializeJson(doc, buffer);
    client.publish(mqtt_topic_pub, buffer);
    Serial.println(buffer);
  }
}