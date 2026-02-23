#include <DualMAX14870MotorShield.h>

// =======================
// Hardware objects
// =======================
DualMAX14870MotorShield Motors;

// =======================
// Encoder pins
// =======================
const int Motor1EPinA = 2;   // RIGHT motor encoder A
const int Motor1EPinB = 3;   // RIGHT motor encoder B
const int Motor2EPinA = 18;  // LEFT motor encoder A
const int Motor2EPinB = 19;  // LEFT motor encoder B

volatile long counter1 = 0;  // RIGHT encoder count
volatile long counter2 = 0;  // LEFT encoder count

// =======================
// Motion + geometry constants (from your code)
// =======================
float COUNTS_PER_REV = 12.0 * 4.0 * 10.15 *0.25; // adjust gear ratio if needed
float TURN_CIRC = 59.69;                  // cm (robot turn circumference)
float WHEEL_CIRC_CM = 22.0;               // cm (wheel circumference)

float FULL_ROTATION_COUNTS = (TURN_CIRC / WHEEL_CIRC_CM) * COUNTS_PER_REV;

// Speeds
int TURN_SPEED = 100;

// Safety timeout so it can’t hang forever if encoders fail
const unsigned long SPIN_TIMEOUT_MS = 4000;

// Pause after spin
const unsigned long PAUSE_MS = 2000;

// =======================
// Encoder ISRs
// =======================
void changeUp1A() {
  if (digitalRead(Motor1EPinA) == digitalRead(Motor1EPinB)) counter1++;
  else counter1--;
}
void changeUp1B() {
  if (digitalRead(Motor1EPinA) == digitalRead(Motor1EPinB)) counter1--;
  else counter1++;
}
void changeUp2A() {
  if (digitalRead(Motor2EPinA) == digitalRead(Motor2EPinB)) counter2++;
  else counter2--;
}
void changeUp2B() {
  if (digitalRead(Motor2EPinA) == digitalRead(Motor2EPinB)) counter2--;
  else counter2++;
}

// =======================
// Setup
// =======================
void setup() {
  Serial.begin(9600);

  Motors.enableDrivers();

  pinMode(Motor1EPinA, INPUT_PULLUP);
  pinMode(Motor1EPinB, INPUT_PULLUP);
  pinMode(Motor2EPinA, INPUT_PULLUP);
  pinMode(Motor2EPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Motor1EPinA), changeUp1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor1EPinB), changeUp1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor2EPinA), changeUp2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor2EPinB), changeUp2B, CHANGE);

  Serial.println("Encoder-based spin test starting...");
  Serial.print("FULL_ROTATION_COUNTS target = ");
  Serial.println(FULL_ROTATION_COUNTS);
}

// =======================
// Helper: spin 1 full rotation using encoders
// =======================
void spinOneFullRotation() {
  // Reset counts
  counter1 = 0;
  counter2 = 0;

  Serial.println("Spinning (encoder-based)...");
  unsigned long t0 = millis();
  unsigned long lastPrint = 0;

  // Spin in place: RIGHT forward, LEFT backward
  Motors.setSpeeds(TURN_SPEED, -TURN_SPEED);

  // Use whichever encoder is actually working / moving reliably (here: counter1)
  while (abs(counter1) < (long)FULL_ROTATION_COUNTS) {

    // periodic debug prints
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.print("c1=");
      Serial.print(counter1);
      Serial.print(" c2=");
      Serial.println(counter2);
    }

    // safety timeout
    if (millis() - t0 > SPIN_TIMEOUT_MS) {
      Serial.println("TIMEOUT: encoder not reaching target (check encoders/motors)");
      break;
    }

    delay(1); // tiny yield
  }

  Motors.setSpeeds(0, 0);
  Serial.println("Stopped.");

  Serial.print("Final c1=");
  Serial.print(counter1);
  Serial.print(" c2=");
  Serial.println(counter2);

  delay(PAUSE_MS);
}

// =======================
// Loop
// =======================
void loop() {
  spinOneFullRotation();
}