// spinnything.ino — MotorESP
// Two ESP-NOW inputs drive the four vibration motors:
//   1. ButtonESP: hold a button to run its motor solid; release to stop it.
//   2. RC car:    while the car's wall-bump joystick is deflected, all four
//                 motors pulse together (wall haptic feedback).
// A held button takes priority over the wall pulse for its own motor.
//
// MotorESP  MAC : c0:cd:d6:81:ff:80
// ButtonESP MAC : 68:FE:71:2C:78:BC
// RC car    MAC : 14:33:5C:61:11:40
//
// ButtonESP command byte -> action
//   0x01–0x04 -> START Motor 1–4
//   0x11–0x14 -> STOP  Motor 1–4
// RC car FeedbackPacket (9 bytes) -> wall pulse while deflected == 1

#include <esp_now.h>
#include <WiFi.h>

// --- Motor pins (index 0..3 = Motor 1..4) ---
// Driver 1: Motor 1 (PWM 25, DIR 26), Motor 2 (PWM 17, DIR 27)
// Driver 2: Motor 3 (PWM 5,  DIR 13), Motor 4 (PWM 23, DIR 19)
const int NUM_MOTORS = 4;
const int MOTOR_PWM[NUM_MOTORS] = { 25, 17, 5, 23 };
const int MOTOR_DIR[NUM_MOTORS] = { 26, 27, 13, 19 };

const uint8_t CAR_MAC[6] = { 0x14, 0x33, 0x5C, 0x61, 0x11, 0x40 };
const uint8_t ESPNOW_CHANNEL = 6;

const int BUTTON_PWM_LEVEL = 150;
const int WALL_PWM_LEVEL = 150;
const unsigned long WALL_PULSE_ON_MS = 100;
const unsigned long WALL_PULSE_OFF_MS = 100;
const unsigned long WALL_TIMEOUT_MS = 500;

struct __attribute__((packed)) FeedbackPacket {
  uint32_t seq;
  uint8_t deflected;
  int16_t joyX;
  int16_t joyY;
};

// Shared with the ESP-NOW callback
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
bool buttonHeld[NUM_MOTORS] = { false, false, false, false };
bool hasWallFeedback = false;
bool wallDeflected = false;
unsigned long lastWallMs = 0;

// Loop-only state
int currentLevel[NUM_MOTORS] = { -1, -1, -1, -1 };
bool wallWasActive = false;
unsigned long wallPulseStart = 0;

bool macsEqual(const uint8_t *lhs, const uint8_t *rhs) {
  return memcmp(lhs, rhs, 6) == 0;
}

// Only touches the pins when the level changes
void setMotor(int index, int level) {
  if (currentLevel[index] == level) return;
  currentLevel[index] = level;
  if (level > 0) {
    digitalWrite(MOTOR_DIR[index], HIGH);
    analogWrite(MOTOR_PWM[index], level);
  } else {
    analogWrite(MOTOR_PWM[index], 0);
    digitalWrite(MOTOR_DIR[index], LOW);
  }
}

// ---- ESP-NOW receive callback ----
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // Wall feedback from the RC car
  if (len == sizeof(FeedbackPacket)) {
    if (!macsEqual(info->src_addr, CAR_MAC)) return;
    FeedbackPacket fb;
    memcpy(&fb, data, sizeof(fb));

    portENTER_CRITICAL(&stateMux);
    wallDeflected = (fb.deflected == 1);
    lastWallMs = millis();
    hasWallFeedback = true;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  // Button command from ButtonESP
  if (len != 1) return;

  uint8_t cmd = data[0];
  uint8_t motor = cmd & 0x0F;
  uint8_t kind = cmd & 0xF0;
  if (motor < 1 || motor > NUM_MOTORS || (kind != 0x00 && kind != 0x10)) {
    Serial.print("-> Unknown command: 0x");
    Serial.println(cmd, HEX);
    return;
  }

  bool start = (kind == 0x00);
  portENTER_CRITICAL(&stateMux);
  buttonHeld[motor - 1] = start;
  portEXIT_CRITICAL(&stateMux);

  Serial.print("-> Motor ");
  Serial.print(motor);
  Serial.println(start ? " START" : " STOP");
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_PWM[i], OUTPUT);
    pinMode(MOTOR_DIR[i], OUTPUT);
    setMotor(i, 0);
  }

  // ESP-NOW setup (channel 6, shared with the RC car and ButtonESP)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(ESPNOW_CHANNEL);

  while (!WiFi.STA.started()) {
    delay(10);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);

  Serial.println("MotorESP ready — waiting for ButtonESP and RC car wall feedback.");
}

void loop() {
  unsigned long now = millis();

  bool held[NUM_MOTORS];
  bool wallValid;
  bool deflected;
  unsigned long wallAge;

  portENTER_CRITICAL(&stateMux);
  for (int i = 0; i < NUM_MOTORS; i++) held[i] = buttonHeld[i];
  wallValid = hasWallFeedback;
  deflected = wallDeflected;
  wallAge = now - lastWallMs;
  portEXIT_CRITICAL(&stateMux);

  // Wall pulse: active while the car reports a deflection and the link is fresh
  bool wallActive = wallValid && deflected && wallAge <= WALL_TIMEOUT_MS;
  if (wallActive != wallWasActive) {
    Serial.println(wallActive ? "-> Wall contact: pulsing all motors" : "-> Wall contact cleared");
    if (wallActive) wallPulseStart = now;  // each contact starts on an ON phase
    wallWasActive = wallActive;
  }

  bool wallPulseOn = false;
  if (wallActive) {
    unsigned long phase = (now - wallPulseStart) % (WALL_PULSE_ON_MS + WALL_PULSE_OFF_MS);
    wallPulseOn = phase < WALL_PULSE_ON_MS;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    int level = 0;
    if (held[i]) {
      level = BUTTON_PWM_LEVEL;
    } else if (wallPulseOn) {
      level = WALL_PWM_LEVEL;
    }
    setMotor(i, level);
  }
}
