// spinnything.ino — MotorESP
// Receives ESP-NOW packets from ButtonESP and runs the corresponding motor
// for 1 second per trigger.
//
// MotorESP  MAC : c0:cd:d6:81:ff:80
// ButtonESP MAC : 68:fe:71:2b:75:d8
//
// Command byte -> Motor
//   0x01 -> Motor 1 (M1_PWM / M1_DIR)
//   0x02 -> Motor 2 (M2_PWM / M2_DIR)
//   0x03 -> Motor 3 (M3_PWM / M3_DIR)
//   0x04 -> Motor 4 (M4_PWM / M4_DIR)

#include <esp_now.h>
#include <WiFi.h>

// --- Motor Driver 1 Pins ---
const int M1_PWM = 25;
const int M1_DIR = 26;

const int M2_PWM = 17;
const int M2_DIR = 27;

// --- Motor Driver 2 Pins ---
const int M3_PWM = 5;
const int M3_DIR = 13;

const int M4_PWM = 23;
const int M4_DIR = 19;

// Timing
unsigned long m1_startTime = 0;
unsigned long m2_startTime = 0;
unsigned long m3_startTime = 0;
unsigned long m4_startTime = 0;
const long runDuration = 1000;  // 1 second

bool m1_Running = false;
bool m2_Running = false;
bool m3_Running = false;
bool m4_Running = false;

// Helper: start a motor
void startMotor(int pwmPin, int dirPin, unsigned long *startTime, bool *running) {
  if (*running) return;  // already running, ignore
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, 150);
  *startTime = millis();
  *running = true;
}

// Helper: stop a motor
void stopMotor(int pwmPin, bool *running) {
  analogWrite(pwmPin, 0);
  *running = false;
}

// ---- ESP-NOW receive callback ----
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  switch (data[0]) {
    case 0x01:
      Serial.println("-> Motor 1 START (ESP-NOW)");
      startMotor(M1_PWM, M1_DIR, &m1_startTime, &m1_Running);
      break;
    case 0x02:
      Serial.println("-> Motor 2 START (ESP-NOW)");
      startMotor(M2_PWM, M2_DIR, &m2_startTime, &m2_Running);
      break;
    case 0x03:
      Serial.println("-> Motor 3 START (ESP-NOW)");
      startMotor(M3_PWM, M3_DIR, &m3_startTime, &m3_Running);
      break;
    case 0x04:
      Serial.println("-> Motor 4 START (ESP-NOW)");
      startMotor(M4_PWM, M4_DIR, &m4_startTime, &m4_Running);
      break;
    default:
      Serial.print("-> Unknown command: 0x");
      Serial.println(data[0], HEX);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor output pins
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);

  Serial.println("MotorESP ready — waiting for ButtonESP triggers.");
}

void loop() {
  unsigned long currentTime = millis();

  // Auto-stop each motor after runDuration
  if (m1_Running && (currentTime - m1_startTime >= runDuration)) {
    stopMotor(M1_PWM, &m1_Running);
    Serial.println("-> Motor 1 STOP");
  }
  if (m2_Running && (currentTime - m2_startTime >= runDuration)) {
    stopMotor(M2_PWM, &m2_Running);
    Serial.println("-> Motor 2 STOP");
  }
  if (m3_Running && (currentTime - m3_startTime >= runDuration)) {
    stopMotor(M3_PWM, &m3_Running);
    Serial.println("-> Motor 3 STOP");
  }
  if (m4_Running && (currentTime - m4_startTime >= runDuration)) {
    stopMotor(M4_PWM, &m4_Running);
    Serial.println("-> Motor 4 STOP");
  }
}