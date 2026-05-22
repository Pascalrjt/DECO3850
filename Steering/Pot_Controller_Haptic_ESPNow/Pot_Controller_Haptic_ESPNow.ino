#include <WiFi.h>
#include <esp_now.h>

const int POT_STEER_PIN = 34;
const int POT_THROTTLE_PIN = 35;
const int POT_BRAKE_PIN = 32;

// Wall haptic motors (two motors, driven together)
const int HAPTIC_A_DIR_PIN = 19;
const int HAPTIC_A_PWM_PIN = 23;
const int HAPTIC_B_DIR_PIN = 27;
const int HAPTIC_B_PWM_PIN = 13;
const int HAPTIC_PWM_LEVEL = 10;

// Proximity haptic motor
const int PROX_PWM_PIN = 25;
const int PROX_DIR_PIN = 26;

const uint8_t LOCAL_MAC[6] = { 0x68, 0xFE, 0x71, 0x2B, 0x75, 0xD8 };
const uint8_t CAR_MAC[6] = { 0x14, 0x33, 0x5C, 0x61, 0x11, 0x40 };
const uint8_t BRIDGE_MAC[6] = { 0x68, 0xFE, 0x71, 0x2C, 0x0B, 0xC8 };
const uint8_t ESPNOW_CHANNEL = 6;

const int ADC_MAX = 4095;
const int STEER_DEADZONE = 120;
const int THROTTLE_DEADZONE = 400;
const int CALIBRATION_SAMPLES = 100;
const int COMMAND_SCALE = 1000;
const int BRAKE_DEADZONE = 400;
const int BRAKE_FULL_PRESS_RAW = 1600;
const int BRAKE_FULL_PRESS_BAND = 40;
const float BRAKE_RELEASE_LEVEL = 0.05f;
const float BRAKE_ARM_LEVEL = 0.95f;
const unsigned long REVERSE_HOLD_MS = 2000;
const unsigned long SEND_INTERVAL_MS = 40;
const unsigned long PRINT_INTERVAL_MS = 250;
const unsigned long FEEDBACK_TIMEOUT_MS = 500;
const unsigned long PROX_TIMEOUT_MS = 500;

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

struct __attribute__((packed)) ProximityPacket {
  uint32_t seq;
  uint8_t strength;
  uint16_t pulse_on_ms;
  uint16_t pulse_off_ms;
  uint8_t flags;
};

const uint8_t PROX_FLAG_VALID = 0x01;

int steerCenter = 0;
int throttleRest = 0;
int brakeRest = 0;
bool reverseArmed = false;
unsigned long brakeHoldStart = 0;
unsigned long lastSendTime = 0;
unsigned long lastPrintTime = 0;
uint32_t packetSequence = 0;

// Wall haptic state
portMUX_TYPE feedbackMux = portMUX_INITIALIZER_UNLOCKED;
FeedbackPacket latestFeedback = {};
bool hasFeedback = false;
unsigned long lastFeedbackMs = 0;
bool wallMotorOn = false;

// Proximity haptic state
portMUX_TYPE proxMux = portMUX_INITIALIZER_UNLOCKED;
ProximityPacket latestProximity = {};
bool hasProximity = false;
unsigned long lastProxMs = 0;

bool proxMotorOn = false;
unsigned long proxPulseStart = 0;
uint8_t proxStrength = 0;
uint16_t proxPulseOn = 0;
uint16_t proxPulseOff = 0;

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

float normalizeThrottle(int raw, int rest, bool invert) {
  int offset = invert ? (rest - raw) : (raw - rest);
  if (offset <= THROTTLE_DEADZONE) {
    return 0.0f;
  }

  int span = invert ? rest : (ADC_MAX - rest);
  if (span <= THROTTLE_DEADZONE) {
    return 0.0f;
  }

  return clampFloat((offset - THROTTLE_DEADZONE) / (float)(span - THROTTLE_DEADZONE), 0.0f, 1.0f);
}

float normalizeBrake(int raw, int rest) {
  int offset = rest - raw;
  if (offset <= BRAKE_DEADZONE) {
    return 0.0f;
  }

  if (raw <= BRAKE_FULL_PRESS_RAW + BRAKE_FULL_PRESS_BAND) {
    return 1.0f;
  }

  int span = (rest - BRAKE_DEADZONE) - (BRAKE_FULL_PRESS_RAW + BRAKE_FULL_PRESS_BAND);
  if (span <= 0) {
    return 0.0f;
  }

  return clampFloat((offset - BRAKE_DEADZONE) / (float)span, 0.0f, 1.0f);
}

int16_t scaledSignedCommand(float normalized) {
  return (int16_t)lroundf(clampFloat(normalized, -1.0f, 1.0f) * COMMAND_SCALE);
}

uint16_t scaledUnsignedCommand(float normalized) {
  return (uint16_t)lroundf(clampFloat(normalized, 0.0f, 1.0f) * COMMAND_SCALE);
}

void writeWallMotor(int dirPin, int pwmPin, bool on) {
  if (on) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, HAPTIC_PWM_LEVEL);
  } else {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
  }
}

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int dataLen) {
  if (dataLen == sizeof(FeedbackPacket) && macsEqual(info->src_addr, CAR_MAC)) {
    FeedbackPacket fb;
    memcpy(&fb, data, sizeof(fb));

    portENTER_CRITICAL(&feedbackMux);
    latestFeedback = fb;
    hasFeedback = true;
    lastFeedbackMs = millis();
    portEXIT_CRITICAL(&feedbackMux);

  } else if (dataLen == sizeof(ProximityPacket)) {
    ProximityPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    portENTER_CRITICAL(&proxMux);
    latestProximity = pkt;
    hasProximity = true;
    lastProxMs = millis();
    portEXIT_CRITICAL(&proxMux);
  }
}

bool bridgeMacIsSet() {
  for (int i = 0; i < 6; i++) {
    if (BRIDGE_MAC[i] != 0x00) return true;
  }
  return false;
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
  Serial.print("Bridge MAC: ");
  Serial.println(macToString(BRIDGE_MAC));
  if (!bridgeMacIsSet()) {
    Serial.println("Warning: BRIDGE_MAC is all zeros - update it with the real bridge MAC.");
    Serial.println("Proximity packets will still be accepted by size, but add the peer for reliability.");
  }
  Serial.print("ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    restartWithMessage(F("ESP-NOW init failed. Rebooting..."));
  }

  esp_now_peer_info_t carPeer = {};
  memcpy(carPeer.peer_addr, CAR_MAC, sizeof(CAR_MAC));
  carPeer.channel = ESPNOW_CHANNEL;
  carPeer.ifidx = WIFI_IF_STA;
  carPeer.encrypt = false;

  esp_err_t result = esp_now_add_peer(&carPeer);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    restartWithMessage(F("Failed to register car peer. Rebooting..."));
  }

  if (bridgeMacIsSet()) {
    esp_now_peer_info_t bridgePeer = {};
    memcpy(bridgePeer.peer_addr, BRIDGE_MAC, sizeof(BRIDGE_MAC));
    bridgePeer.channel = ESPNOW_CHANNEL;
    bridgePeer.ifidx = WIFI_IF_STA;
    bridgePeer.encrypt = false;

    result = esp_now_add_peer(&bridgePeer);
    if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
      Serial.println("Warning: failed to register bridge peer.");
    }
  }

  if (esp_now_register_recv_cb(onEspNowRecv) != ESP_OK) {
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

void calibrateThrottleRest() {
  Serial.println("Calibrating throttle rest - leave the throttle pot untouched...");

  long total = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    total += analogRead(POT_THROTTLE_PIN);
    delay(10);
  }

  throttleRest = total / CALIBRATION_SAMPLES;
  Serial.print("Throttle rest: ");
  Serial.println(throttleRest);
}

void calibrateBrakeRest() {
  Serial.println("Calibrating brake rest - leave the brake pot released...");

  long total = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    total += analogRead(POT_BRAKE_PIN);
    delay(10);
  }

  brakeRest = total / CALIBRATION_SAMPLES;
  Serial.print("Brake rest: ");
  Serial.println(brakeRest);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(HAPTIC_A_DIR_PIN, OUTPUT);
  pinMode(HAPTIC_A_PWM_PIN, OUTPUT);
  pinMode(HAPTIC_B_DIR_PIN, OUTPUT);
  pinMode(HAPTIC_B_PWM_PIN, OUTPUT);
  digitalWrite(HAPTIC_A_DIR_PIN, LOW);
  digitalWrite(HAPTIC_A_PWM_PIN, LOW);
  digitalWrite(HAPTIC_B_DIR_PIN, LOW);
  digitalWrite(HAPTIC_B_PWM_PIN, LOW);

  pinMode(PROX_PWM_PIN, OUTPUT);
  pinMode(PROX_DIR_PIN, OUTPUT);
  digitalWrite(PROX_DIR_PIN, LOW);
  digitalWrite(PROX_PWM_PIN, LOW);

  setupEspNow();
  calibrateSteeringCenter();
  calibrateThrottleRest();
  calibrateBrakeRest();

  Serial.println("Pot controller transmitter (haptic + proximity) ready");
}

void loop() {
  unsigned long now = millis();
  int rawSteer = analogRead(POT_STEER_PIN);
  int rawThrottle = analogRead(POT_THROTTLE_PIN);
  int rawBrake = analogRead(POT_BRAKE_PIN);

  float steer = normalizeCentered(rawSteer, steerCenter, INVERT_STEERING);
  float throttle = normalizeThrottle(rawThrottle, throttleRest, INVERT_THROTTLE);
  float brake = normalizeBrake(rawBrake, brakeRest);

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

  // --- Wall haptic (M1) ---
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

  bool wantWallMotor = fbValid && fbAge <= FEEDBACK_TIMEOUT_MS && fb.deflected == 1;

  if (wantWallMotor && !wallMotorOn) {
    writeWallMotor(HAPTIC_A_DIR_PIN, HAPTIC_A_PWM_PIN, true);
    writeWallMotor(HAPTIC_B_DIR_PIN, HAPTIC_B_PWM_PIN, true);
    wallMotorOn = true;
  } else if (!wantWallMotor && wallMotorOn) {
    writeWallMotor(HAPTIC_A_DIR_PIN, HAPTIC_A_PWM_PIN, false);
    writeWallMotor(HAPTIC_B_DIR_PIN, HAPTIC_B_PWM_PIN, false);
    wallMotorOn = false;
  }

  // --- Proximity haptic (M2) ---
  ProximityPacket prox = {};
  bool proxValid = false;
  unsigned long proxAge = 0;

  portENTER_CRITICAL(&proxMux);
  if (hasProximity) {
    prox = latestProximity;
    proxAge = now - lastProxMs;
    proxValid = true;
  }
  portEXIT_CRITICAL(&proxMux);

  bool proxActive = proxValid
                    && proxAge <= PROX_TIMEOUT_MS
                    && (prox.flags & PROX_FLAG_VALID)
                    && prox.strength > 0;

  if (proxActive) {
    if (proxStrength != prox.strength || proxPulseOn != prox.pulse_on_ms
        || proxPulseOff != prox.pulse_off_ms) {
      proxStrength = prox.strength;
      proxPulseOn = prox.pulse_on_ms;
      proxPulseOff = prox.pulse_off_ms;
    }

    if (proxPulseOff == 0) {
      if (!proxMotorOn) {
        digitalWrite(PROX_DIR_PIN, HIGH);
        analogWrite(PROX_PWM_PIN, proxStrength);
        proxMotorOn = true;
      }
    } else {
      unsigned long elapsed = now - proxPulseStart;
      if (proxMotorOn && elapsed >= proxPulseOn) {
        analogWrite(PROX_PWM_PIN, 0);
        digitalWrite(PROX_DIR_PIN, LOW);
        proxMotorOn = false;
        proxPulseStart = now;
      } else if (!proxMotorOn && elapsed >= proxPulseOff) {
        digitalWrite(PROX_DIR_PIN, HIGH);
        analogWrite(PROX_PWM_PIN, proxStrength);
        proxMotorOn = true;
        proxPulseStart = now;
      }
    }
  } else {
    if (proxMotorOn) {
      analogWrite(PROX_PWM_PIN, 0);
      digitalWrite(PROX_DIR_PIN, LOW);
      proxMotorOn = false;
    }
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("SteerRaw: ");
    Serial.print(rawSteer);
    Serial.print("  ThrottleRaw: ");
    Serial.print(rawThrottle);
    Serial.print("  ThrottleNorm: ");
    Serial.print(throttle, 2);
    Serial.print("  BrakeRaw: ");
    Serial.print(rawBrake);
    Serial.print("  BrakeNorm: ");
    Serial.print(brake, 2);
    Serial.print("  Reverse: ");
    Serial.print(reverseArmed);
    Serial.print("  Wall: ");
    Serial.print(wallMotorOn ? "ON" : "OFF");
    Serial.print("  Prox: ");
    Serial.print(proxMotorOn ? "ON" : "OFF");
    Serial.print("  ProxStr: ");
    Serial.print(proxStrength);
    Serial.print("  FbSeq: ");
    Serial.print(fb.seq);
    Serial.print("  FbAge: ");
    Serial.print(fbAge);
    Serial.print("  PxSeq: ");
    Serial.print(prox.seq);
    Serial.print("  PxAge: ");
    Serial.println(proxAge);
  }
}
