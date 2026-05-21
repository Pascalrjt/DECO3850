#include <WiFi.h>
#include <esp_now.h>
#include <AccelStepper.h>

// Motor 1 (Left) pins
#define M1_IN1 27
#define M1_IN2 14
#define M1_IN3 12
#define M1_IN4 13

// Motor 2 (Right) pins
#define M2_IN1 26
#define M2_IN2 25
#define M2_IN3 33
#define M2_IN4 32

// Joystick pins (SW unused)
#define JOY_VRX_PIN 35
#define JOY_VRY_PIN 34

const uint8_t LOCAL_MAC[6] = {0x14, 0x33, 0x5C, 0x61, 0x11, 0x40};
const uint8_t CONTROLLER_MAC[6] = {0x68, 0xFE, 0x71, 0x2B, 0x75, 0xD8};
const uint8_t ESPNOW_CHANNEL = 6;

const float MAX_SPEED = 1000.0f;
const float ACCEL_RATE = 1500.0f;
const int COMMAND_SCALE = 1000;
const unsigned long PACKET_TIMEOUT_MS = 250;
const unsigned long PRINT_INTERVAL_MS = 500;

const int JOY_DEADZONE = 300;
const int JOY_ADC_MAX = 4095;
const int JOY_CALIBRATION_SAMPLES = 100;
const unsigned long JOY_SEND_INTERVAL_MS = 40;

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

// Note pin order: IN1, IN3, IN2, IN4 for correct half-step phasing
AccelStepper stepper1(AccelStepper::HALF4WIRE, M1_IN1, M1_IN3, M1_IN2, M1_IN4);
AccelStepper stepper2(AccelStepper::HALF4WIRE, M2_IN1, M2_IN3, M2_IN2, M2_IN4);

portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
ControlPacket latestPacket = {};
bool hasPacket = false;
unsigned long lastPacketMillis = 0;

float currentLeft = 0.0f;
float currentRight = 0.0f;
unsigned long lastPrintTime = 0;
unsigned long lastRampTime = 0;
bool failsafeActive = true;

int joyCenterX = 0;
int joyCenterY = 0;
unsigned long lastJoySendTime = 0;
uint32_t feedbackSeq = 0;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float rampToward(float current, float target, float maxDelta) {
  float diff = target - current;
  if (diff > maxDelta) return current + maxDelta;
  if (diff < -maxDelta) return current - maxDelta;
  return target;
}

float normalizeJoyAxis(int raw, int center) {
  int offset = raw - center;
  if (abs(offset) <= JOY_DEADZONE) return 0.0f;
  int span = (offset > 0) ? (JOY_ADC_MAX - center) : center;
  if (span <= JOY_DEADZONE) return 0.0f;
  float mag = (abs(offset) - JOY_DEADZONE) / (float)(span - JOY_DEADZONE);
  return clampFloat((offset > 0 ? 1.0f : -1.0f) * mag, -1.0f, 1.0f);
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

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int dataLen) {
  if (dataLen != sizeof(ControlPacket)) {
    return;
  }

  if (!macsEqual(info->src_addr, CONTROLLER_MAC)) {
    return;
  }

  ControlPacket packet;
  memcpy(&packet, data, sizeof(packet));

  portENTER_CRITICAL(&packetMux);
  latestPacket = packet;
  hasPacket = true;
  lastPacketMillis = millis();
  portEXIT_CRITICAL(&packetMux);
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

  Serial.print("Receiver MAC: ");
  Serial.println(macToString(actualMac));
  Serial.print("Expected car MAC: ");
  Serial.println(macToString(LOCAL_MAC));
  if (!macsEqual(actualMac, LOCAL_MAC)) {
    Serial.println("Warning: this board MAC does not match the planned RC car receiver.");
  }
  Serial.print("Expected controller MAC: ");
  Serial.println(macToString(CONTROLLER_MAC));
  Serial.print("ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    restartWithMessage(F("ESP-NOW init failed. Rebooting..."));
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, CONTROLLER_MAC, sizeof(CONTROLLER_MAC));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
  if (addPeerResult != ESP_OK && addPeerResult != ESP_ERR_ESPNOW_EXIST) {
    restartWithMessage(F("Failed to register controller peer. Rebooting..."));
  }

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    restartWithMessage(F("Failed to register receive callback. Rebooting..."));
  }
}

void calibrateJoystickCenter() {
  Serial.println("Calibrating joystick center - leave the joystick centered...");
  long totalX = 0;
  long totalY = 0;
  for (int i = 0; i < JOY_CALIBRATION_SAMPLES; i++) {
    totalX += analogRead(JOY_VRX_PIN);
    totalY += analogRead(JOY_VRY_PIN);
    delay(10);
  }
  joyCenterX = totalX / JOY_CALIBRATION_SAMPLES;
  joyCenterY = totalY / JOY_CALIBRATION_SAMPLES;
  Serial.print("Joystick center X: ");
  Serial.print(joyCenterX);
  Serial.print("  Y: ");
  Serial.println(joyCenterY);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  stepper1.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);

  setupEspNow();
  calibrateJoystickCenter();

  Serial.println("RC car receiver ready");
  lastRampTime = millis();
}

void loop() {
  ControlPacket command = {};
  bool packetAvailable = false;
  unsigned long packetAgeMs = 0;
  unsigned long now = millis();

  portENTER_CRITICAL(&packetMux);
  packetAvailable = hasPacket;
  if (hasPacket) {
    command = latestPacket;
    packetAgeMs = now - lastPacketMillis;
  }
  portEXIT_CRITICAL(&packetMux);

  float targetLeft = 0.0f;
  float targetRight = 0.0f;

  if (packetAvailable && packetAgeMs <= PACKET_TIMEOUT_MS) {
    float steer = clampFloat(command.steer / (float)COMMAND_SCALE, -1.0f, 1.0f);
    float throttle = clampFloat(command.throttle / (float)COMMAND_SCALE, 0.0f, 1.0f);
    float brake = clampFloat(command.brake / (float)COMMAND_SCALE, 0.0f, 1.0f);

    float drive = command.reverseArmed ? -brake : (throttle * (1.0f - brake));
    float steerBias = steer * fabsf(drive);

    float leftMix = clampFloat(drive - steerBias, -1.0f, 1.0f);
    float rightMix = clampFloat(drive + steerBias, -1.0f, 1.0f);

    targetLeft = leftMix * MAX_SPEED;
    targetRight = rightMix * MAX_SPEED;
    failsafeActive = false;
  } else {
    failsafeActive = true;
  }

  float dt = (now - lastRampTime) / 1000.0f;
  lastRampTime = now;
  float maxDelta = ACCEL_RATE * dt;

  currentLeft = rampToward(currentLeft, targetLeft, maxDelta);
  currentRight = rampToward(currentRight, targetRight, maxDelta);

  currentLeft = clampFloat(currentLeft, -MAX_SPEED, MAX_SPEED);
  currentRight = clampFloat(currentRight, -MAX_SPEED, MAX_SPEED);

  stepper1.setSpeed(-currentLeft);
  stepper2.setSpeed(currentRight);
  stepper1.runSpeed();
  stepper2.runSpeed();

  if (now - lastJoySendTime >= JOY_SEND_INTERVAL_MS) {
    lastJoySendTime = now;

    float jx = normalizeJoyAxis(analogRead(JOY_VRX_PIN), joyCenterX);
    float jy = normalizeJoyAxis(analogRead(JOY_VRY_PIN), joyCenterY);

    FeedbackPacket fb = {};
    fb.seq = ++feedbackSeq;
    fb.deflected = (jx != 0.0f || jy != 0.0f) ? 1 : 0;
    fb.joyX = (int16_t)lroundf(jx * COMMAND_SCALE);
    fb.joyY = (int16_t)lroundf(jy * COMMAND_SCALE);

    esp_now_send(CONTROLLER_MAC, reinterpret_cast<const uint8_t *>(&fb), sizeof(fb));
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("Seq: ");
    Serial.print(command.seq);
    Serial.print("  AgeMs: ");
    Serial.print(packetAgeMs);
    Serial.print("  Failsafe: ");
    Serial.print(failsafeActive ? "ON" : "OFF");
    Serial.print("  Steer: ");
    Serial.print(command.steer);
    Serial.print("  Throttle: ");
    Serial.print(command.throttle);
    Serial.print("  Brake: ");
    Serial.print(command.brake);
    Serial.print("  Reverse: ");
    Serial.print(command.reverseArmed);
    Serial.print("  CurL: ");
    Serial.print(currentLeft, 0);
    Serial.print("  CurR: ");
    Serial.println(currentRight, 0);
  }
}
