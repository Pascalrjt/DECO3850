// --- Motor Driver 1 Pins ---
const int M1_PWM = 25;
const int M1_DIR = 26;
const int BTN_1 = 14;

const int M2_PWM = 17;
const int M2_DIR = 27;
const int BTN_2 = 12;

// --- Motor Driver 2 Pins ---
// (Assigned to available, non-conflicting ESP32 pins)
const int M3_PWM = 5; 
const int M3_DIR = 13;
const int BTN_3 = 33;

const int M4_PWM = 23;
const int M4_DIR = 19;
const int BTN_4 = 32;

// Timing variables
unsigned long m1_startTime = 0;
unsigned long m2_startTime = 0;
unsigned long m3_startTime = 0;
unsigned long m4_startTime = 0;
const long runDuration = 1000; // 1 seconds

bool m1_Running = false;
bool m2_Running = false;
bool m3_Running = false;
bool m4_Running = false;

// Button state tracking
bool lastBtn1 = HIGH;
bool lastBtn2 = HIGH;
bool lastBtn3 = HIGH;
bool lastBtn4 = HIGH;

void setup() {
  // Driver 1 Setup
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  
  // Driver 2 Setup
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);
  
  // Input pins
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial.println("System Ready: 4 Motors Initialized.");
}

void loop() {
  unsigned long currentTime = millis();

  // --- MOTOR 1 LOGIC ---
  bool currentBtn1 = digitalRead(BTN_1);
  if (currentBtn1 == LOW && lastBtn1 == HIGH && !m1_Running) {
    Serial.println("-> Motor 1 START");
    digitalWrite(M1_DIR, HIGH);
    analogWrite(M1_PWM, 150);
    m1_startTime = currentTime;
    m1_Running = true;
  }
  lastBtn1 = currentBtn1;

  if (m1_Running && (currentTime - m1_startTime >= runDuration)) {
    analogWrite(M1_PWM, 0);
    m1_Running = false;
    Serial.println("-> Motor 1 STOP");
  }

  // --- MOTOR 2 LOGIC ---
  bool currentBtn2 = digitalRead(BTN_2);
  if (currentBtn2 == LOW && lastBtn2 == HIGH && !m2_Running) {
    Serial.println("-> Motor 2 START");
    digitalWrite(M2_DIR, HIGH);
    analogWrite(M2_PWM, 150);
    m2_startTime = currentTime;
    m2_Running = true;
  }
  lastBtn2 = currentBtn2;

  if (m2_Running && (currentTime - m2_startTime >= runDuration)) {
    analogWrite(M2_PWM, 0);
    m2_Running = false;
    Serial.println("-> Motor 2 STOP");
  }

  // --- MOTOR 3 LOGIC ---
  bool currentBtn3 = digitalRead(BTN_3);
  if (currentBtn3 == LOW && lastBtn3 == HIGH && !m3_Running) {
    Serial.println("-> Motor 3 START");
    digitalWrite(M3_DIR, HIGH);
    analogWrite(M3_PWM, 150);
    m3_startTime = currentTime;
    m3_Running = true;
  }
  lastBtn3 = currentBtn3;

  if (m3_Running && (currentTime - m3_startTime >= runDuration)) {
    analogWrite(M3_PWM, 0);
    m3_Running = false;
    Serial.println("-> Motor 3 STOP");
  }

  // --- MOTOR 4 LOGIC ---
  bool currentBtn4 = digitalRead(BTN_4);
  if (currentBtn4 == LOW && lastBtn4 == HIGH && !m4_Running) {
    Serial.println("-> Motor 4 START");
    digitalWrite(M4_DIR, HIGH);
    analogWrite(M4_PWM, 150);
    m4_startTime = currentTime;
    m4_Running = true;
  }
  lastBtn4 = currentBtn4;

  if (m4_Running && (currentTime - m4_startTime >= runDuration)) {
    analogWrite(M4_PWM, 0);
    m4_Running = false;
    Serial.println("-> Motor 4 STOP");
  }
}