// buttonTrigger.ino — ButtonESP
// Sends ESP-NOW START/STOP commands to MotorESP for each of the 4 big buttons.
// Hold a button to run its motor; release to stop it.
//
// START commands: 0x01–0x04 (Motor 1–4)
// STOP  commands: 0x11–0x14 (Motor 1–4)
//
// A START is sent on the press and repeated every HEARTBEAT_MS while the button
// stays held. MotorESP stops a motor when the repeats stop arriving, so a lost
// packet in either direction corrects itself within a fraction of a second.
//
// ButtonESP MAC : 68:FE:71:2C:78:BC
// MotorESP  MAC : c0:cd:d6:81:ff:80

#include <esp_now.h>
#include <WiFi.h>
#include <esp_idf_version.h>

// --- Button pins (index 0..3 = Button 1..4, each triggers the same-numbered motor) ---
const int NUM_BUTTONS = 4;
const int BTN_PINS[NUM_BUTTONS] = { 27, 14, 12, 13 };

// MotorESP MAC address
uint8_t motorEspMac[] = { 0xC0, 0xCD, 0xD6, 0x81, 0xFF, 0x80 };

// Must match MotorESP, which shares channel 6 with the RC car
const uint8_t ESPNOW_CHANNEL = 6;

const unsigned long DEBOUNCE_MS = 20;    // a reading must hold this long before it counts
const unsigned long HEARTBEAT_MS = 100;  // START repeat interval while held (MotorESP expects one within 400 ms)
const unsigned long POLL_DELAY_MS = 5;

// Packet: single byte command
// START: 0x01 = Motor 1, 0x02 = Motor 2, 0x03 = Motor 3, 0x04 = Motor 4
// STOP:  0x11 = Motor 1, 0x12 = Motor 2, 0x13 = Motor 3, 0x14 = Motor 4
typedef struct {
  uint8_t command;
} TriggerPacket;

// The send callback's first argument changed in ESP-IDF 5.5 (Arduino-ESP32 3.3)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
typedef const wifi_tx_info_t *SendCallbackInfo;
#else
typedef const uint8_t *SendCallbackInfo;
#endif

// Button state tracking
bool pressed[NUM_BUTTONS] = { false, false, false, false };   // debounced state
bool lastRaw[NUM_BUTTONS] = { false, false, false, false };   // last raw reading
unsigned long lastRawChangeMs[NUM_BUTTONS] = { 0, 0, 0, 0 };
unsigned long lastHeartbeatMs = 0;

void onDataSent(SendCallbackInfo info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("-> Send FAILED (is MotorESP powered and on the same channel?)");
  }
}

void sendCommand(uint8_t cmd) {
  TriggerPacket packet;
  packet.command = cmd;
  esp_now_send(motorEspMac, (uint8_t *)&packet, sizeof(packet));
}

void sendStart(uint8_t motorNum) {
  Serial.print("Button ");
  Serial.print(motorNum);
  Serial.println(" held — START");
  sendCommand(motorNum);
}

void sendStop(uint8_t motorNum) {
  Serial.print("Button ");
  Serial.print(motorNum);
  Serial.println(" released — STOP");
  sendCommand(0x10 | motorNum);
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
  }

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

  esp_now_register_send_cb(onDataSent);

  // Register MotorESP as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, motorEspMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add MotorESP peer");
    return;
  }

  Serial.println("ButtonESP ready — 4 buttons active.");
}

void loop() {
  unsigned long now = millis();

  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool raw = (digitalRead(BTN_PINS[i]) == LOW);   // pressed pulls the pin to GND

    if (raw != lastRaw[i]) {
      lastRaw[i] = raw;
      lastRawChangeMs[i] = now;
    } else if (raw != pressed[i] && now - lastRawChangeMs[i] >= DEBOUNCE_MS) {
      pressed[i] = raw;
      if (raw) {
        sendStart(i + 1);   // press   -> START
      } else {
        sendStop(i + 1);    // release -> STOP
      }
    }
  }

  // Repeat START for every held button so MotorESP can tell the link is alive
  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    for (int i = 0; i < NUM_BUTTONS; i++) {
      if (pressed[i]) sendCommand(i + 1);
    }
  }

  delay(POLL_DELAY_MS);
}
