// RC_Car_JoystickTest: bench test for the RC car with no controller and no radio.
// The car's own joystick drives the two steppers, using the same motion as
// RC_Car_ESPNow: same pins, speed limit, acceleration ramp and steering mix.
//
//   Joystick Y -> drive   (forward / reverse)
//   Joystick X -> steering (only has an effect while driving, as on the real car)
//
// Leave the joystick centred while powering on (start-up calibration).
// Flash RC_Car_ESPNow again afterwards for normal use.

#include <AccelStepper.h>

// Motor 1 (Left) pins
#define M1_IN1 27
#define M1_IN2 14
#define M1_IN3 16   // not GPIO 12: that pin sets the flash voltage at boot and blocked start-up
#define M1_IN4 13

// Motor 2 (Right) pins
#define M2_IN1 26
#define M2_IN2 25
#define M2_IN3 33
#define M2_IN4 32

// Joystick pins (SW unused)
#define JOY_VRX_PIN 35
#define JOY_VRY_PIN 34

// Flip these if the car moves the wrong way for a joystick direction
const bool INVERT_DRIVE = false;
const bool INVERT_STEER = false;

// Same values as RC_Car_ESPNow
const float MAX_SPEED = 1000.0f;
const float ACCEL_RATE = 1500.0f;
const int JOY_DEADZONE = 300;
const int JOY_ADC_MAX = 4095;
const int JOY_CALIBRATION_SAMPLES = 100;
const unsigned long JOY_READ_INTERVAL_MS = 40;
const unsigned long PRINT_INTERVAL_MS = 250;

// Note pin order: IN1, IN3, IN2, IN4 for correct half-step phasing
AccelStepper stepper1(AccelStepper::HALF4WIRE, M1_IN1, M1_IN3, M1_IN2, M1_IN4);
AccelStepper stepper2(AccelStepper::HALF4WIRE, M2_IN1, M2_IN3, M2_IN2, M2_IN4);

int joyCenterX = 0;
int joyCenterY = 0;
int rawX = 0;
int rawY = 0;
float targetLeft = 0.0f;
float targetRight = 0.0f;
float currentLeft = 0.0f;
float currentRight = 0.0f;
float drive = 0.0f;
float steer = 0.0f;
unsigned long lastJoyReadTime = 0;
unsigned long lastRampTime = 0;
unsigned long lastPrintTime = 0;

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

  calibrateJoystickCenter();

  Serial.println("RC car joystick test ready");
  lastRampTime = millis();
}

void loop() {
  unsigned long now = millis();

  // Read the joystick at the same rate the real sketch does
  if (now - lastJoyReadTime >= JOY_READ_INTERVAL_MS) {
    lastJoyReadTime = now;
    rawX = analogRead(JOY_VRX_PIN);
    rawY = analogRead(JOY_VRY_PIN);

    steer = normalizeJoyAxis(rawX, joyCenterX);
    drive = normalizeJoyAxis(rawY, joyCenterY);
    if (INVERT_STEER) steer = -steer;
    if (INVERT_DRIVE) drive = -drive;

    // Same mix as RC_Car_ESPNow: steering scales with drive
    float steerBias = steer * fabsf(drive);
    float leftMix = clampFloat(drive - steerBias, -1.0f, 1.0f);
    float rightMix = clampFloat(drive + steerBias, -1.0f, 1.0f);
    targetLeft = leftMix * MAX_SPEED;
    targetRight = rightMix * MAX_SPEED;
  }

  float dt = (now - lastRampTime) / 1000.0f;
  lastRampTime = now;
  float maxDelta = ACCEL_RATE * dt;

  currentLeft = rampToward(currentLeft, targetLeft, maxDelta);
  currentRight = rampToward(currentRight, targetRight, maxDelta);

  currentLeft = clampFloat(currentLeft, -MAX_SPEED, MAX_SPEED);
  currentRight = clampFloat(currentRight, -MAX_SPEED, MAX_SPEED);

  // Left motor is inverted so both sides move forward together
  stepper1.setSpeed(-currentLeft);
  stepper2.setSpeed(currentRight);
  stepper1.runSpeed();
  stepper2.runSpeed();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.printf("JoyX: %d  JoyY: %d  Drive: %.2f  Steer: %.2f  CurL: %.0f  CurR: %.0f\n",
                  rawX, rawY, drive, steer, currentLeft, currentRight);
  }
}
