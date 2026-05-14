// buttonTrigger.ino — ButtonESP
// Sends an ESP-NOW command to MotorESP for each of the 4 big buttons.
// Each button maps to one vibration motor on the MotorESP.
//
// ButtonESP MAC : 68:fe:71:2b:75:d8
// MotorESP  MAC : c0:cd:d6:81:ff:80

#include <esp_now.h>
#include <WiFi.h>

// --- Button pins ---
const int BTN_1 = 27;   // triggers Motor 1
const int BTN_2 = 14;   // triggers Motor 2
const int BTN_3 = 12;   // triggers Motor 3
const int BTN_4 = 13;   // triggers Motor 4

// MotorESP MAC address
uint8_t motorEspMac[] = { 0xC0, 0xCD, 0xD6, 0x81, 0xFF, 0x80 };

// Packet: single byte command
// 0x01 = Motor 1, 0x02 = Motor 2, 0x03 = Motor 3, 0x04 = Motor 4
typedef struct {
  uint8_t command;
} TriggerPacket;

TriggerPacket packet;

// Button state tracking
bool lastBtn1 = HIGH;
bool lastBtn2 = HIGH;
bool lastBtn3 = HIGH;
bool lastBtn4 = HIGH;

void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.print("-> Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void sendTrigger(uint8_t motorCmd) {
  Serial.print("Button ");
  Serial.print(motorCmd);
  Serial.println(" pressed");
  packet.command = motorCmd;
  esp_now_send(motorEspMac, (uint8_t *)&packet, sizeof(packet));
  Serial.print("-> Sent trigger for Motor ");
  Serial.println(motorCmd);
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  // Register MotorESP as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, motorEspMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add MotorESP peer");
    return;
  }

  Serial.println("ButtonESP ready — 4 buttons active.");
}

void loop() {
  bool currentBtn1 = digitalRead(BTN_1);
  bool currentBtn2 = digitalRead(BTN_2);
  bool currentBtn3 = digitalRead(BTN_3);
  bool currentBtn4 = digitalRead(BTN_4);

  // Send on falling edge (press)
  if (currentBtn1 == LOW && lastBtn1 == HIGH) sendTrigger(0x01);
  if (currentBtn2 == LOW && lastBtn2 == HIGH) sendTrigger(0x02);
  if (currentBtn3 == LOW && lastBtn3 == HIGH) sendTrigger(0x03);
  if (currentBtn4 == LOW && lastBtn4 == HIGH) sendTrigger(0x04);

  lastBtn1 = currentBtn1;
  lastBtn2 = currentBtn2;
  lastBtn3 = currentBtn3;
  lastBtn4 = currentBtn4;

  delay(10);
}
