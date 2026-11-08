
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// FlightSense Phase 4:
// USB Serial + Two-Axis Servo Control

const byte BUTTON_PIN = 2;
const byte LED_PIN = 7;
const byte PITCH_PIN = 9;
const byte ROLL_PIN = 10;

const int CENTER = 90;
const int MIN_ANGLE = 75;
const int MAX_ANGLE = 105;

const unsigned long TIMEOUT_MS = 10000;

Servo pitchServo;
Servo rollServo;

int pitchTarget = CENTER;
int rollTarget = CENTER;

bool softwareFault = false;
bool remoteActive = false;
bool linkTimeout = false;
bool lastManualState = false;

unsigned long lastContact = 0;

char serialBuffer[40];
byte serialLength = 0;

bool buttonPressed() {
  return digitalRead(BUTTON_PIN) == LOW;
}

bool faultActive() {
  return buttonPressed() ||
         softwareFault ||
         linkTimeout;
}

void applyOutputs() {
  if (faultActive()) {
    digitalWrite(LED_PIN, HIGH);
    pitchServo.write(CENTER);
    rollServo.write(CENTER);
  } else {
    digitalWrite(LED_PIN, LOW);
    pitchServo.write(pitchTarget);
    rollServo.write(rollTarget);
  }
}

void acknowledgeContact() {
  remoteActive = true;
  linkTimeout = false;
  lastContact = millis();
}

bool parseAngle(const char *text, int &result) {
  char *end;
  long value = strtol(text, &end, 10);

  if (end == text || *end != '\0') {
    return false;
  }

  if (value < MIN_ANGLE || value > MAX_ANGLE) {
    return false;
  }

  result = (int)value;
  return true;
}

void printStatus() {
  Serial.print("STATUS PITCH=");
  Serial.print(faultActive() ? CENTER : pitchTarget);

  Serial.print(" ROLL=");
  Serial.print(faultActive() ? CENTER : rollTarget);

  Serial.print(" BUTTON=");
  Serial.print(buttonPressed() ? 1 : 0);

  Serial.print(" FAULT=");
  Serial.print(faultActive() ? 1 : 0);

  Serial.print(" TIMEOUT=");
  Serial.println(linkTimeout ? 1 : 0);
}

void processCommand(char *command) {
  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "PING") == 0) {
    acknowledgeContact();
    Serial.println("PONG");
    return;
  }

  if (strcmp(command, "FAULT:1") == 0) {
    acknowledgeContact();
    softwareFault = true;
    pitchTarget = CENTER;
    rollTarget = CENTER;
    Serial.println("OK FAULT ACTIVE");
    return;
  }

  if (strcmp(command, "FAULT:0") == 0) {
    acknowledgeContact();
    softwareFault = false;
    pitchTarget = CENTER;
    rollTarget = CENTER;
    Serial.println("OK FAULT CLEARED");
    return;
  }

  if (buttonPressed()) {
    Serial.println("ERR MANUAL FAULT");
    return;
  }

  if (softwareFault) {
    Serial.println("ERR SOFTWARE FAULT");
    return;
  }

  if (strcmp(command, "CENTER") == 0) {
    acknowledgeContact();
    pitchTarget = CENTER;
    rollTarget = CENTER;
    Serial.println("OK CENTER");
    return;
  }

  int angle;

  if (strncmp(command, "PITCH:", 6) == 0) {
    if (!parseAngle(command + 6, angle)) {
      Serial.println("ERR PITCH RANGE 75-105");
      return;
    }

    acknowledgeContact();
    pitchTarget = angle;
    Serial.println("OK PITCH");
    return;
  }

  if (strncmp(command, "ROLL:", 5) == 0) {
    if (!parseAngle(command + 5, angle)) {
      Serial.println("ERR ROLL RANGE 75-105");
      return;
    }

    acknowledgeContact();
    rollTarget = angle;
    Serial.println("OK ROLL");
    return;
  }

  Serial.println("ERR UNKNOWN COMMAND");
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (serialLength > 0) {
        serialBuffer[serialLength] = '\0';
        processCommand(serialBuffer);
        serialLength = 0;
      }
    } else if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = c;
    } else {
      serialLength = 0;
      Serial.println("ERR COMMAND TOO LONG");
    }
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  pitchServo.attach(PITCH_PIN);
  rollServo.attach(ROLL_PIN);

  pitchServo.write(CENTER);
  rollServo.write(CENTER);

  lastManualState = buttonPressed();

  Serial.println("FlightSense Phase 4 READY");
  applyOutputs();
}

void loop() {
  bool currentManualState = buttonPressed();

  if (currentManualState && !lastManualState) {
    pitchTarget = CENTER;
    rollTarget = CENTER;
    Serial.println("EVENT MANUAL FAULT");
  }

  if (!currentManualState && lastManualState) {
    Serial.println("EVENT MANUAL CLEARED");
  }

  lastManualState = currentManualState;

  readCommands();

  if (remoteActive && !linkTimeout &&
      millis() - lastContact > TIMEOUT_MS) {
    linkTimeout = true;
    pitchTarget = CENTER;
    rollTarget = CENTER;
    Serial.println("EVENT LINK TIMEOUT");
  }

  applyOutputs();
  delay(10);
}
