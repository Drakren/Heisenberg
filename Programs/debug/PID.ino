#include <Arduino.h>

// --- Motor Pins ---
const int IN1 = 2, IN2 = 3;   // Left motor direction
const int IN3 = 4, IN4 = 7;   // Right motor direction
const int STBY = 8;
const int PWMA = 5, PWMB = 6; // Left/Right PWM

// --- Ultrasonic Pins ---
const int trigF = 10, echoF = 11;   // Forward
const int trigL = 14, echoL = 15;   // Left
const int trigR = 12, echoR = 13;   // Right

// --- Button for STBY ---
const int buttonPin = 9;
bool motorsEnabled = false;
unsigned long lastButtonPress = 0;   // debounce timer

// --- PID parameters (yours) ---
float Kp = 4.5, Ki = 0.00, Kd = 2.15;
float error = 0, prevError = 0, integral = 0;
int baseSpeed = 110;                 // adjust for your motors

// --- Targets / thresholds ---
const int corridorThresh = 20;       // "wall is present" threshold (cm)
//const int targetLeft  = 20;          // desired left-wall distance (cm)
//const int targetRight = 20;          // desired right-wall distance (cm)
const int stopFront   = 10;           // stop if obstacle closer than this (cm)

// -------------------- FUNCTIONS --------------------
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);  // 20ms timeout
  if (duration == 0) return 1000;              // no echo → far away
  return duration * 0.034 / 2;                 // cm
}

void driveMotors(int left, int right) {
  // Left Motor
  if (left >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(PWMA, abs(left));

  // Right Motor
  if (right >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else            { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  analogWrite(PWMB, abs(right));
}

void toggleMotors() {
  // Using INPUT_PULLUP: pressed = LOW
  static bool lastState = HIGH;
  bool currentState = digitalRead(buttonPin);

  if (lastState == HIGH && currentState == LOW) {   // edge: press
    if (millis() - lastButtonPress > 250) {         // debounce 250ms
      motorsEnabled = !motorsEnabled;
      digitalWrite(STBY, motorsEnabled ? HIGH : LOW);
      lastButtonPress = millis();
      // optional: Serial.println(motorsEnabled ? "ON" : "OFF");
    }
  }
  lastState = currentState;
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(trigF, OUTPUT); pinMode(echoF, INPUT);
  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigR, OUTPUT); pinMode(echoR, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(STBY, LOW);  // Motors initially off
}

// -------------------- MAIN LOOP --------------------
void loop() {
  toggleMotors();
  if (!motorsEnabled) {
    driveMotors(0, 0);
    return;
  }

  float distF = getDistance(trigF, echoF);
  float distL = getDistance(trigL, echoL);
  float distR = getDistance(trigR, echoR);

  // ---- Adaptive wall following (no absolutes, relative logic) ----
  if (distL < corridorThresh && distR < corridorThresh) {
    // Corridor → balance between walls
    // Use distR - distL so positive error => steer RIGHT
    error = distR - distL;
  }
  // else if (distL < corridorThresh) {
  //   // Only left wall → keep ~targetLeft
  //   // If too close (distL < target), error positive → steer RIGHT
  //   error = (targetLeft - distL);
  // }
  // else if (distR < corridorThresh) {
  //   // Only right wall → keep ~targetRight
  //   // If too close (distR < target), error negative → steer LEFT
  //   error = (distR - targetRight);
  //}
  else {
    // No walls → go straight
    error = 0;
  }

  // Stop if obstacle too close in front
  if (distF < stopFront) {
    driveMotors(0, 0);
    // Debug
    Serial.print("L:"); Serial.print(distL);
    Serial.print(" R:"); Serial.print(distR);
    Serial.print(" F:"); Serial.print(distF);
    Serial.println("  [STOP]");
    delay(50);
    return;
  }

  // ---- PID Control ----
  integral += error;
  float derivative = error - prevError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  // Apply so: positive correction => RIGHT turn (left faster, right slower)
  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  driveMotors(leftSpeed, rightSpeed);

  // Debug
  Serial.print("L:"); Serial.print(distL);
  Serial.print(" R:"); Serial.print(distR);
  Serial.print(" F:"); Serial.print(distF);
  Serial.print(" Err:"); Serial.print(error);
  Serial.print(" Corr:"); Serial.print(correction);
  Serial.print("  LS:"); Serial.print(leftSpeed);
  Serial.print(" RS:"); Serial.println(rightSpeed);

  delay(50);
}
