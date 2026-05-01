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

// Note pin order: IN1, IN3, IN2, IN4 for correct half-step phasing
AccelStepper stepper1(AccelStepper::HALF4WIRE, M1_IN1, M1_IN3, M1_IN2, M1_IN4);
AccelStepper stepper2(AccelStepper::HALF4WIRE, M2_IN1, M2_IN3, M2_IN2, M2_IN4);

const float MAX_SPEED = 1000.0;
const unsigned long RAMP_UP_TIME_MS = 3000;
const unsigned long HOLD_TIME_MS = 10000;
const unsigned long RAMP_DOWN_TIME_MS = 3000;
const unsigned long PAUSE_TIME_MS = 2000;
const unsigned long PRINT_INTERVAL_MS = 500;

enum State { RAMP_UP, HOLD, RAMP_DOWN, STOPPED };
State currentState = RAMP_UP;

unsigned long phaseStartTime = 0;
unsigned long lastPrintTime = 0;
float currentSpeed = 0;

const char* stateNames[] = { "RAMP_UP", "HOLD", "RAMP_DOWN", "STOPPED" };

void setup() {
  Serial.begin(115200);
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);

  currentState = RAMP_UP;
  phaseStartTime = millis();
  lastPrintTime = millis();

  Serial.println("Stepper ramp test starting (2 motors)...");
  Serial.println("[RAMP_UP] Accelerating to max speed");
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - phaseStartTime;

  switch (currentState) {
    case RAMP_UP:
      if (elapsed >= RAMP_UP_TIME_MS) {
        currentSpeed = MAX_SPEED;
        currentState = HOLD;
        phaseStartTime = now;
        Serial.println("[HOLD] Holding at max speed");
      } else {
        currentSpeed = MAX_SPEED * ((float)elapsed / RAMP_UP_TIME_MS);
      }
      stepper1.setSpeed(-currentSpeed);
      stepper2.setSpeed(currentSpeed);
      stepper1.runSpeed();
      stepper2.runSpeed();
      break;

    case HOLD:
      if (elapsed >= HOLD_TIME_MS) {
        currentState = RAMP_DOWN;
        phaseStartTime = now;
        Serial.println("[RAMP_DOWN] Decelerating to stop");
      }
      stepper1.setSpeed(-MAX_SPEED);
      stepper2.setSpeed(MAX_SPEED);
      stepper1.runSpeed();
      stepper2.runSpeed();
      break;

    case RAMP_DOWN:
      if (elapsed >= RAMP_DOWN_TIME_MS) {
        currentSpeed = 0;
        stepper1.setSpeed(0);
        stepper2.setSpeed(0);
        currentState = STOPPED;
        phaseStartTime = now;
        Serial.println("[STOPPED] Motors stopped, pausing...");
      } else {
        currentSpeed = MAX_SPEED * (1.0 - (float)elapsed / RAMP_DOWN_TIME_MS);
        stepper1.setSpeed(-currentSpeed);
        stepper2.setSpeed(currentSpeed);
        stepper1.runSpeed();
        stepper2.runSpeed();
      }
      break;

    case STOPPED:
      if (elapsed >= PAUSE_TIME_MS) {
        currentState = RAMP_UP;
        phaseStartTime = now;
        Serial.println("[RAMP_UP] Accelerating to max speed");
      }
      break;
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("[");
    Serial.print(stateNames[currentState]);
    Serial.print("] Speed: ");
    Serial.print(currentSpeed, 1);
    Serial.print(" steps/s  M1 pos: ");
    Serial.print(stepper1.currentPosition());
    Serial.print("  M2 pos: ");
    Serial.println(stepper2.currentPosition());
  }
}
