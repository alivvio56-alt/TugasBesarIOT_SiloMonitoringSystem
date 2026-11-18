
#include <ESP32Servo.h>
#include <DHT.h>

// ==========================================
// PIN CONFIGURATION
// ==========================================

// MOTOR DRIVER L298N
#define ENA 25
#define IN1 26
#define IN2 27

// RELAY
#define RELAY_PIN 19

// SERVO
const int servoPin = 18;

// HC-SR04
#define TRIG_PIN 16
#define ECHO_PIN 17

// DHT22
#define DHT_PIN 4
#define DHTTYPE DHT22

// MQ SENSOR
#define MQ_PIN 34

// ==========================================
// OBJECT
// ==========================================

Servo myServo;
DHT dht(DHT_PIN, DHTTYPE);

// ==========================================
// CONFIGURATION
// ==========================================

const bool RELAY_ACTIVE_LOW = true;

int motorSpeed = 180;
bool motorState = false;

// ==========================================
// MOTOR CONTROL
// ==========================================

void motorOn() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, motorSpeed);

  motorState = true;

  Serial.println("Motor ON");
}

void motorOff() {
  analogWrite(ENA, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  motorState = false;

  Serial.println("Motor OFF");
}

void setMotorSpeed(int speedValue) {
  motorSpeed = constrain(speedValue, 0, 255);

  Serial.print("Motor PWM: ");
  Serial.println(motorSpeed);

  if (motorState) {
    analogWrite(ENA, motorSpeed);
  }
}

// ==========================================
// RELAY CONTROL
// ==========================================

void relayOn() {
  digitalWrite(
    RELAY_PIN,
    RELAY_ACTIVE_LOW ? LOW : HIGH
  );

  Serial.println("Relay ON");
}

void relayOff() {
  digitalWrite(
    RELAY_PIN,
    RELAY_ACTIVE_LOW ? HIGH : LOW
  );

  Serial.println("Relay OFF");
}

// ==========================================
// SERVO CONTROL
// Menggunakan kode standalone yang berhasil
// ==========================================

void servoOn() {
  myServo.write(90);

  Serial.println("Servo ON - Posisi 90 derajat");
}

void servoOff() {
  myServo.write(0);

  Serial.println("Servo OFF - Posisi 0 derajat");
}

// ==========================================
// HC-SR04 DISTANCE SENSOR
// ==========================================

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2.0;
}

void testDistance() {
  float distance = readDistance();

  if (distance < 0) {
    Serial.println("HC-SR04: No Echo");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}

// ==========================================
// DHT22 TEMPERATURE & HUMIDITY
// ==========================================

void testTemperature() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT ERROR: Sensor tidak terbaca");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
}

// ==========================================
// MQ GAS SENSOR
// ==========================================

void testGas() {
  int gasValue = analogRead(MQ_PIN);

  float voltage = gasValue * (3.3 / 4095.0);

  Serial.print("MQ Analog: ");
  Serial.println(gasValue);

  Serial.print("MQ Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
}

// ==========================================
// READ ALL SENSORS
// ==========================================

void readAllSensors() {
  Serial.println();
  Serial.println("========== SENSOR DATA ==========");

  testTemperature();
  testDistance();
  testGas();

  Serial.println("=================================");
  Serial.println();
}

// ==========================================
// SERIAL COMMAND MENU
// ==========================================

void printHelp() {
  Serial.println();
  Serial.println("======= ESP32 PERIPHERAL TEST =======");

  Serial.println("--- MOTOR ---");
  Serial.println("motor on");
  Serial.println("motor off");
  Serial.println("motor speed 150");

  Serial.println("--- RELAY ---");
  Serial.println("relay on");
  Serial.println("relay off");

  Serial.println("--- SERVO ---");
  Serial.println("servo on");
  Serial.println("servo off");

  Serial.println("--- SENSOR ---");
  Serial.println("distance");
  Serial.println("temperature");
  Serial.println("gas");
  Serial.println("read");

  Serial.println("--- OTHER ---");
  Serial.println("help");

  Serial.println("====================================");
  Serial.println();
}

// ==========================================
// PROCESS SERIAL COMMAND
// ==========================================

void processCommand(String command) {

  command.trim();
  command.toLowerCase();

  // MOTOR
  if (command == "motor on") {
    motorOn();
  }

  else if (command == "motor off") {
    motorOff();
  }

  else if (command.startsWith("motor speed ")) {
    int speedValue = command.substring(12).toInt();
    setMotorSpeed(speedValue);
  }

  // RELAY
  else if (command == "relay on") {
    relayOn();
  }

  else if (command == "relay off") {
    relayOff();
  }

  // SERVO
  else if (command == "servo on") {
    servoOn();
  }

  else if (command == "servo off") {
    servoOff();
  }

  // SENSOR
  else if (command == "distance") {
    testDistance();
  }

  else if (command == "temperature") {
    testTemperature();
  }

  else if (command == "gas") {
    testGas();
  }

  else if (command == "read") {
    readAllSensors();
  }

  // HELP
  else if (command == "help") {
    printHelp();
  }

  else {
    Serial.println("Command tidak dikenal. Ketik help.");
  }
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  // SERVO SETUP (di-attach pertama agar timer LEDC 50Hz tidak bentrok
  // dengan analogWrite() motor)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
  myServo.write(0);

  // MOTOR SETUP
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  motorOff();

  // RELAY SETUP
  pinMode(RELAY_PIN, OUTPUT);
  relayOff();


  // HC-SR04 SETUP
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  // DHT SETUP
  dht.begin();

  // MQ SETUP
  pinMode(MQ_PIN, INPUT);
  analogReadResolution(12);

  Serial.println();
  Serial.println("ESP32 PERIPHERAL TEST READY");
  Serial.println("Baudrate: 115200");

  printHelp();
}

// ==========================================
// LOOP
// ==========================================

void loop() {

  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');

    processCommand(command);

  }

}
