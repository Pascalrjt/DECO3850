#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000); // give Serial Monitor time to reconnect after reset
  Serial.println();
  Serial.println("=== MAC ADDRESS ===");
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  Serial.println("===================");
}

void loop() {
  delay(1000);
  Serial.println(WiFi.macAddress()); // print repeatedly so you can't miss it
}