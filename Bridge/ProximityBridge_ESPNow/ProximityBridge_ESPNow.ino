#include <WiFi.h>
#include <esp_now.h>

// --- Target: pot controller ---
const uint8_t CONTROLLER_MAC[6] = {0x14, 0x33, 0x5C, 0x25, 0x5B, 0x48};
const uint8_t ESPNOW_CHANNEL = 6;

// --- Serial framing ---
const uint8_t FRAME_MAGIC_0 = 0xAA;
const uint8_t FRAME_MAGIC_1 = 0x55;
const int PAYLOAD_SIZE = 10;

struct __attribute__((packed)) ProximityPacket {
  uint32_t seq;
  uint8_t strength;
  uint16_t pulse_on_ms;
  uint16_t pulse_off_ms;
  uint8_t flags;
};

static_assert(sizeof(ProximityPacket) == PAYLOAD_SIZE, "ProximityPacket size mismatch");

enum ParseState { WAIT_MAGIC_0, WAIT_MAGIC_1, READ_PAYLOAD, READ_CHECKSUM };

ParseState parseState = WAIT_MAGIC_0;
uint8_t payloadBuf[PAYLOAD_SIZE];
int payloadIdx = 0;

uint32_t framesForwarded = 0;
uint32_t framesDropped = 0;
unsigned long lastStatusMs = 0;
const unsigned long STATUS_INTERVAL_MS = 2000;

String macToString(const uint8_t *mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(ESPNOW_CHANNEL);

  while (!WiFi.STA.started()) {
    delay(10);
  }

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("Bridge MAC: ");
  Serial.println(macToString(mac));
  Serial.print("Target controller MAC: ");
  Serial.println(macToString(CONTROLLER_MAC));
  Serial.print("ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed. Rebooting...");
    delay(3000);
    ESP.restart();
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, CONTROLLER_MAC, sizeof(CONTROLLER_MAC));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    Serial.println("Failed to add controller peer. Rebooting...");
    delay(3000);
    ESP.restart();
  }
}

void processSerialByte(uint8_t b) {
  switch (parseState) {
    case WAIT_MAGIC_0:
      if (b == FRAME_MAGIC_0) parseState = WAIT_MAGIC_1;
      break;

    case WAIT_MAGIC_1:
      if (b == FRAME_MAGIC_1) {
        parseState = READ_PAYLOAD;
        payloadIdx = 0;
      } else {
        parseState = WAIT_MAGIC_0;
      }
      break;

    case READ_PAYLOAD:
      payloadBuf[payloadIdx++] = b;
      if (payloadIdx >= PAYLOAD_SIZE) parseState = READ_CHECKSUM;
      break;

    case READ_CHECKSUM: {
      uint8_t expected = 0;
      for (int i = 0; i < PAYLOAD_SIZE; i++) expected ^= payloadBuf[i];

      if (b == expected) {
        esp_now_send(CONTROLLER_MAC, payloadBuf, PAYLOAD_SIZE);
        framesForwarded++;
      } else {
        framesDropped++;
      }
      parseState = WAIT_MAGIC_0;
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  setupEspNow();
  Serial.println("Proximity bridge ready - waiting for serial frames");
}

void loop() {
  while (Serial.available()) {
    processSerialByte(Serial.read());
  }

  unsigned long now = millis();
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    Serial.print("Forwarded: ");
    Serial.print(framesForwarded);
    Serial.print("  Dropped: ");
    Serial.println(framesDropped);
  }
}
