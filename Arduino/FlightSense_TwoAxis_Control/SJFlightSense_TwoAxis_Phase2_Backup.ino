
#include <Servo.h>

// ==========================================
// FlightSense - Two-Axis Control System
// ==========================================

// Pin assignments
const int BUTTON_PIN = 2;
const int LED_PIN = 7;
const int PITCH_SERVO_PIN = 9;
const int ROLL_SERVO_PIN = 10;

// Servo objects
Servo pitchServo;
Servo rollServo;

// Positions
const int CENTER_ANGLE = 90;
const int LOW_ANGLE = 75;
const int HIGH_ANGLE = 105;

// System state
bool faultActive = false;
bool positionHigh = false;

// Timing
unsigned long lastMove = 0;
const unsigned long MOVE_INTERVAL = 1500;

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);

  // Button: LOW when pressed
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Active-HIGH fault LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Attach servos
  pitchServo.attach(PITCH_SERVO_PIN);
  rollServo.attach(ROLL_SERVO_PIN);

  // Initialize at neutral
  pitchServo.write(CENTER_ANGLE);
  rollServo.write(CENTER_ANGLE);

  lastMove = millis();

  Serial.println("========================");
  Serial.println("FlightSense TWO-AXIS");
  Serial.println("Control System Ready");
  Serial.println("========================");
}

// ==========================================
// Main Loop
// ==========================================
void loop() {

  // Read manual fault button
  bool buttonPressed =
    digitalRead(BUTTON_PIN) == LOW;

  // ========================================
  // FAULT MODE
  // ========================================
  if (buttonPressed) {

    if (!faultActive) {
      faultActive = true;

      Serial.println("FAULT DETECTED");
      Serial.println("Centering pitch & roll");

      // Center both servos
      pitchServo.write(CENTER_ANGLE);
      rollServo.write(CENTER_ANGLE);

      // Turn warning LED ON
      digitalWrite(LED_PIN, HIGH);
    }

  }

  // ========================================
  // NORMAL MODE
  // ========================================
  else {

    if (faultActive) {
      faultActive = false;

      Serial.println("SYSTEM NORMAL");
      Serial.println("Resuming two-axis motion");

      // Reset motion timer
      lastMove = millis();
    }

    // Turn warning LED OFF
    digitalWrite(LED_PIN, LOW);

    // Move every 1.5 seconds
    if (millis() - lastMove >= MOVE_INTERVAL) {

      lastMove = millis();
      positionHigh = !positionHigh;

      if (positionHigh) {
        // Pitch up, roll opposite
        pitchServo.write(HIGH_ANGLE);
        rollServo.write(LOW_ANGLE);

        Serial.println("Pitch: 105 | Roll: 75");
      }
      else {
        // Pitch down, roll opposite
        pitchServo.write(LOW_ANGLE);
        rollServo.write(HIGH_ANGLE);

        Serial.println("Pitch: 75 | Roll: 105");
      }
    }
  }

  delay(20);
}
