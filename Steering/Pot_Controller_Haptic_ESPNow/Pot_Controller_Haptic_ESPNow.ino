#include <WiFi.h>
#include <esp_now.h>

const int POT_STEER_PIN = 34;
const int POT_THROTTLE_PIN = 35;
const int POT_BRAKE_PIN = 32;

const int HAPTIC_PWM_PIN = 25;
const int HAPTIC_DIR_PIN = 26;
const int HAPTIC_PWM_LEVEL = 10;

const uint8_t LOCAL_MAC[6] = {0x14, 0x33, 0x5C, 0x25, 0x5B, 0x48};
const uint8_t CAR_MAC[6] = {0x14, 0x33, 0x5C, 0x61, 0x11, 0x40};
const uint8_t ESPNOW_CHANNEL = 6;

const int ADC_MAX = 4095;
const int STEER_DEADZONE = 120;
const int CALIBRATION_SAMPLES = 100;
const int COMMAND_SCALE = 1000;
const int BRAKE_RELEASE_RAW = 4095;
const int BRAKE_FULL_PRESS_RAW = 1600;
const int BRAKE_RELEASE_BAND = 40;
const int BRAKE_FULL_PRESS_BAND = 40;
const float BRAKE_RELEASE_LEVEL = 0.05f;
const float BRAKE_ARM_LEVEL = 0.95f;
const unsigned long REVERSE_HOLD_MS = 2000;
const unsigned long SEND_INTERVAL_MS = 40;
const unsigned long PRINT_INTERVAL_MS = 250;
const unsigned long FEEDBACK_TIMEOUT_MS = 500;

const bool INVERT_STEERING = false;
const bool INVERT_THROTTLE = false;

struct __attribute__((packed)) ControlPacket {
  uint32_t seq;
  int16_t steer;
  uint16_t throttle;
  uint16_t brake;
  uint8_t reverseArmed;
};

struct __attribute__((packed)) FeedbackPacket {
  uint32_t seq;
  uint8_t deflected;
  int16_t joyX;
  int16_t joyY;
};

int steerCenter = 0;
bool reverseArmed = false;
unsigned long brakeHoldStart = 0;
unsigned long lastSendTime = 0;
unsigned long lastPrintTime = 0;
uint32_t packetSequence = 0;

portMUX_TYPE feedbackMux = portMUX_INITIALIZER_UNLOCKED;
FeedbackPacket latestFeedback = {};
bool hasFeedback = false;
unsigned long lastFeedbackMs = 0;
bool motorRunning = false;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

String macToString(const uint8_t *mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

bool macsEqual(const uint8_t *lhs, const uint8_t *rhs) {
  return memcmp(lhs, rhs, 6) == 0;
}

void restartWithMessage(const __FlashStringHelper *message) {
  Serial.println(message);
  delay(3000);
  ESP.restart();
}

float normalizeCentered(int raw, int center, bool invert) {
  int offset = raw - center;
  if (invert) {
    offset = -offset;
  }

  if (abs(offset) <= STEER_DEADZONE) {
    return 0.0f;
  }

  int span = (offset > 0) ? (ADC_MAX - center) : center;
  if (span <= STEER_DEADZONE) {
    return 0.0f;
  }

  float magnitude = (abs(offset) - STEER_DEADZONE) / (float)(span - STEER_DEADZONE);
  return clampFloat((offset > 0 ? 1.0f : -1.0f) * magnitude, -1.0f, 1.0f);
}

float normalizeRange(int raw, bool invert) {
  float normalized = raw / (float)ADC_MAX;
  if (invert) {
    normalized = 1.0f - normalized;
  }

  if (normalized < 0.02f) normalized = 0.0f;
  if (normalized > 0.98f) normalized = 1.0f;
  return clampFloat(normalized, 0.0f, 1.0f);
}

float normalizeBrake(int raw) {
  if (raw >= BRAKE_RELEASE_RAW - BRAKE_RELEASE_BAND) {
    return 0.0f;
  }

  if (raw <= BRAKE_FULL_PRESS_RAW + BRAKE_FULL_PRESS_BAND) {
    return 1.0f;
  }

  float normalized = (BRAKE_RELEASE_RAW - raw) / (float)(BRAKE_RELEASE_RAW - BRAKE_FULL_PRESS_RAW);
  return clampFloat(normalized, 0.0f, 1.0f);
}

int16_t scaledSignedCommand(float normalized) {
  return (int16_t)lroundf(clampFloat(normalized, -1.0f, 1.0f) * COMMAND_SCALE);
}

uint16_t scaledUnsignedCommand(float normalized) {
  return (uint16_t)lroundf(clampFloat(normalized, 0.0f, 1.0f) * COMMAND_SCALE);
}

void onFeedbackRecv(const esp_now_recv_info_t *info, const uint8_t *data, int dataLen) {
  if (dataLen != sizeof(FeedbackPacket)) {
    return;
  }

  if (!macsEqual(info->src_addr, CAR_MAC)) {
    return;
  }

  FeedbackPacket fb;
  memcpy(&fb, data, sizeof(fb));

  portENTER_CRITICAL(&feedbackMux);
  latestFeedback = fb;
  hasFeedback = true;
  lastFeedbackMs = millis();
  portEXIT_CRITICAL(&feedbackMux);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(ESPNOW_CHANNEL);

  while (!WiFi.STA.started()) {
    delay(10);
  }

  uint8_t actualMac[6];
  WiFi.macAddress(actualMac);

  Serial.print("Controller MAC: ");
  Serial.println(macToString(actualMac));
  Serial.print("Expected controller MAC: ");
  Serial.println(macToString(LOCAL_MAC));
  if (!macsEqual(actualMac, LOCAL_MAC)) {
    Serial.println("Warning: this board MAC does not match the planned controller.");
  }
  Serial.print("Target car MAC: ");
  Serial.println(macToString(CAR_MAC));
  Serial.print("ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    restartWithMessage(F("ESP-NOW init failed. Rebooting..."));
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, CAR_MAC, sizeof(CAR_MAC));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
  if (addPeerResult != ESP_OK && addPeerResult != ESP_ERR_ESPNOW_EXIST) {
    restartWithMessage(F("Failed to register car peer. Rebooting..."));
  }

  if (esp_now_register_recv_cb(onFeedbackRecv) != ESP_OK) {
    restartWithMessage(F("Failed to register receive callback. Rebooting..."));
  }
}

void calibrateSteeringCenter() {
  Serial.println("Calibrating steering center - leave the steering pot centered...");

  long total = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    total += analogRead(POT_STEER_PIN);
    delay(10);
  }

  steerCenter = total / CALIBRATION_SAMPLES;
  Serial.print("Steering center: ");
  Serial.println(steerCenter);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(HAPTIC_PWM_PIN, OUTPUT);
  pinMode(HAPTIC_DIR_PIN, OUTPUT);
  digitalWrite(HAPTIC_DIR_PIN, LOW);
  digitalWrite(HAPTIC_PWM_PIN, LOW);

  setupEspNow();
  calibrateSteeringCenter();

  Serial.println("Pot controller transmitter (haptic) ready");
}

void loop() {
  unsigned long now = millis();
  int rawSteer = analogRead(POT_STEER_PIN);
  int rawThrottle = analogRead(POT_THROTTLE_PIN);
  int rawBrake = analogRead(POT_BRAKE_PIN);

  float steer = normalizeCentered(rawSteer, steerCenter, INVERT_STEERING);
  float throttle = normalizeRange(rawThrottle, INVERT_THROTTLE);
  float brake = normalizeBrake(rawBrake);

  if (brake <= BRAKE_RELEASE_LEVEL) {
    reverseArmed = false;
    brakeHoldStart = 0;
  } else if (!reverseArmed) {
    if (brake >= BRAKE_ARM_LEVEL) {
      if (brakeHoldStart == 0) {
        brakeHoldStart = now;
      } else if (now - brakeHoldStart >= REVERSE_HOLD_MS) {
        reverseArmed = true;
      }
    } else {
      brakeHoldStart = 0;
    }
  }

  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;

    ControlPacket packet = {};
    packet.seq = ++packetSequence;
    packet.steer = scaledSignedCommand(steer);
    packet.throttle = scaledUnsignedCommand(throttle);
    packet.brake = scaledUnsignedCommand(brake);
    packet.reverseArmed = reverseArmed ? 1 : 0;

    esp_now_send(CAR_MAC, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  }

  // --- Haptic motor feedback ---
  FeedbackPacket fb = {};
  bool fbValid = false;
  unsigned long fbAge = 0;

  portENTER_CRITICAL(&feedbackMux);
  if (hasFeedback) {
    fb = latestFeedback;
    fbAge = now - lastFeedbackMs;
    fbValid = true;
  }
  portEXIT_CRITICAL(&feedbackMux);

  bool wantMotor = fbValid && fbAge <= FEEDBACK_TIMEOUT_MS && fb.deflected == 1;

  if (wantMotor && !motorRunning) {
    digitalWrite(HAPTIC_DIR_PIN, HIGH);
    analogWrite(HAPTIC_PWM_PIN, HAPTIC_PWM_LEVEL);
    motorRunning = true;
  } else if (!wantMotor && motorRunning) {
    analogWrite(HAPTIC_PWM_PIN, 0);
    digitalWrite(HAPTIC_DIR_PIN, LOW);
    motorRunning = false;
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("SteerRaw: ");
    Serial.print(rawSteer);
    Serial.print("  ThrottleRaw: ");
    Serial.print(rawThrottle);
    Serial.print("  BrakeRaw: ");
    Serial.print(rawBrake);
    Serial.print("  BrakeNorm: ");
    Serial.print(brake, 2);
    Serial.print("  Reverse: ");
    Serial.print(reverseArmed);
    Serial.print("  Haptic: ");
    Serial.print(motorRunning ? "ON" : "OFF");
    Serial.print("  FbSeq: ");
    Serial.print(fb.seq);
    Serial.print("  FbAge: ");
    Serial.println(fbAge);
  }
}
