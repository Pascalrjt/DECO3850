  // Joystick pins
  const int VRX_PIN = 39;   // X-axis
  const int VRY_PIN = 36;   // Y-axis
  const int SW_PIN  = 23;   // Button

  void setup() {
    Serial.begin(115200);

    // Button pin
    pinMode(SW_PIN, INPUT_PULLUP);

    // Optional: set ADC resolution for ESP32 (0 - 4095)
    analogReadResolution(12);

    Serial.println("ESP32 Joystick Reader Started");
  }

  void loop() {
    int xValue = analogRead(VRX_PIN);
    int yValue = analogRead(VRY_PIN);
    int swValue = digitalRead(SW_PIN);

    Serial.print("X: ");
    Serial.print(xValue);
    Serial.print(" | Y: ");
    Serial.print(yValue);
    Serial.print(" | SW: ");

    if (swValue == LOW) {
      Serial.println("Pressed");
    } else {
      Serial.println("Released");
    }

    delay(200);
  }