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

// Joystick pins
#define JOY_X 35  // VRX
#define JOY_Y 34  // VRY

// Note pin order: IN1, IN3, IN2, IN4 for correct half-step phasing
AccelStepper stepper1(AccelStepper::HALF4WIRE, M1_IN1, M1_IN3, M1_IN2, M1_IN4);
AccelStepper stepper2(AccelStepper::HALF4WIRE, M2_IN1, M2_IN3, M2_IN2, M2_IN4);

const float MAX_SPEED = 1000.0;
const float ACCEL_RATE = 1500.0;   // Speed units per second (higher = snappier)
const int DEADZONE = 300;
const unsigned long PRINT_INTERVAL_MS = 500;
const int CALIBRATION_SAMPLES = 100;

// Auto-calibrated center values
int joyCenterX = 0;
int joyCenterY = 0;

// Current ramped speeds
float currentLeft = 0;
float currentRight = 0;

unsigned long lastPrintTime = 0;
unsigned long lastRampTime = 0;

// Map joystick value to speed, applying deadzone around calibrated center
float mapJoystick(int value, int center) {
  int offset = value - center;
  if (abs(offset) < DEADZONE) return 0.0;

  int maxRange = max(center, 4095 - center);

  if (offset > 0) {
    return MAX_SPEED * (float)(offset - DEADZONE) / (float)(maxRange - DEADZONE);
  } else {
    return MAX_SPEED * (float)(offset + DEADZONE) / (float)(maxRange - DEADZONE);
  }
}

// Move current speed toward target by max delta
float rampToward(float current, float target, float maxDelta) {
  float diff = target - current;
  if (diff > maxDelta) return current + maxDelta;
  if (diff < -maxDelta) return current - maxDelta;
  return target;
}

void setup() {
  Serial.begin(115200);
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);

  // Auto-calibrate joystick center position
  // Keep joystick untouched during startup!
  Serial.println("Calibrating joystick - do not touch...");
  long sumX = 0;
  long sumY = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(10);
  }
  joyCenterX = sumX / CALIBRATION_SAMPLES;
  joyCenterY = sumY / CALIBRATION_SAMPLES;

  Serial.print("Calibrated center - X: ");
  Serial.print(joyCenterX);
  Serial.print("  Y: ");
  Serial.println(joyCenterY);
  Serial.println("Joystick tank drive ready");

  lastRampTime = millis();
}

void loop() {
  // Read joystick
  int rawX = analogRead(JOY_X);
  int rawY = analogRead(JOY_Y);

  // Joystick is rotated 90 degrees:
  // X axis -> forward/backward
  // Y axis -> left/right turning
  float drive = -mapJoystick(rawX, joyCenterX);
  float steer = -mapJoystick(rawY, joyCenterY);

  // Dominant-axis control: use whichever input is larger
  float targetLeft;
  float targetRight;
  if (abs(steer) > abs(drive)) {
    // Steering dominates - rotate on the spot
    targetLeft  =  steer;
    targetRight = -steer;
  } else {
    // Drive dominates - move forward/backward
    targetLeft  = drive;
    targetRight = drive;
  }

  // Time-based smooth ramping toward target speeds
  unsigned long now = millis();
  float dt = (now - lastRampTime) / 1000.0;
  lastRampTime = now;
  float maxDelta = ACCEL_RATE * dt;

  currentLeft  = rampToward(currentLeft,  targetLeft,  maxDelta);
  currentRight = rampToward(currentRight, targetRight, maxDelta);

  // Clamp to max speed
  currentLeft  = constrain(currentLeft,  -MAX_SPEED, MAX_SPEED);
  currentRight = constrain(currentRight, -MAX_SPEED, MAX_SPEED);

  // Apply speeds (left motor negated to match forward direction)
  stepper1.setSpeed(-currentLeft);
  stepper2.setSpeed(currentRight);

  // Must be called as often as possible to step the motors
  stepper1.runSpeed();
  stepper2.runSpeed();

  // Debug output
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("X: ");
    Serial.print(rawX);
    Serial.print("  Y: ");
    Serial.print(rawY);
    Serial.print("  TargetL: ");
    Serial.print(targetLeft, 0);
    Serial.print(" TargetR: ");
    Serial.print(targetRight, 0);
    Serial.print("  CurL: ");
    Serial.print(currentLeft, 0);
    Serial.print(" CurR: ");
    Serial.println(currentRight, 0);
  }
}
