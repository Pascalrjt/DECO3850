// spinnything.ino — MotorESP
// Receives ESP-NOW START/STOP packets from ButtonESP.
// Hold a button to run its motor; release to stop it.
//
// MotorESP  MAC : c0:cd:d6:81:ff:80
// ButtonESP MAC : 68:FE:71:2C:78:BC
//
// Command byte -> action
//   0x01–0x04 -> START Motor 1–4
//   0x11–0x14 -> STOP  Motor 1–4

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

// Helper: start a motor (hold)
void startMotor(int pwmPin, int dirPin) {
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, 150);
}

// Helper: stop a motor
void stopMotor(int pwmPin) {
  analogWrite(pwmPin, 0);
}

// ---- ESP-NOW receive callback ----
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  switch (data[0]) {
    // --- START commands ---
    case 0x01: Serial.println("-> Motor 1 START"); startMotor(M1_PWM, M1_DIR); break;
    case 0x02: Serial.println("-> Motor 2 START"); startMotor(M2_PWM, M2_DIR); break;
    case 0x03: Serial.println("-> Motor 3 START"); startMotor(M3_PWM, M3_DIR); break;
    case 0x04: Serial.println("-> Motor 4 START"); startMotor(M4_PWM, M4_DIR); break;
    // --- STOP commands ---
    case 0x11: Serial.println("-> Motor 1 STOP");  stopMotor(M1_PWM); break;
    case 0x12: Serial.println("-> Motor 2 STOP");  stopMotor(M2_PWM); break;
    case 0x13: Serial.println("-> Motor 3 STOP");  stopMotor(M3_PWM); break;
    case 0x14: Serial.println("-> Motor 4 STOP");  stopMotor(M4_PWM); break;
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
  // Nothing to poll — motors are started/stopped entirely via ESP-NOW callbacks.
}