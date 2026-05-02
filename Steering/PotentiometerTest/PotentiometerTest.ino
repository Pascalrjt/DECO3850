const int POT1_PIN = 34;   // 5k Steering
const int POT2_PIN = 35;   // 10k Throttle
const int POT3_PIN = 32;   // 50k Reverse

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);              // 0–4095
  analogSetAttenuation(ADC_11db);        // full 0–3.3V range
}

void loop() {
  int p1 = analogRead(POT1_PIN);
  int p2 = analogRead(POT2_PIN);
  int p3 = analogRead(POT3_PIN);

  float v1 = p1 * 3.3f / 4095.0f;
  float v2 = p2 * 3.3f / 4095.0f;
  float v3 = p3 * 3.3f / 4095.0f;

  Serial.printf("Pot1 (10k, GPIO34): %4d  %.2fV   |   ", p1, v1);
  Serial.printf("Pot2 (10k, GPIO35): %4d  %.2fV   |   ", p2, v2);
  Serial.printf("Pot3 (5k,  GPIO32): %4d  %.2fV\n",     p3, v3);

  delay(200);
}
