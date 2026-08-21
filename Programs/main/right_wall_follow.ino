#include <Wire.h>

#define MPU_ADDR    0x68
#define PWR_MGMT_1  0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

// === Ultrasonic pins ===
int trigf = 10, echof = 11;
int trigr = 12, echor = 13;
int trigl = 14, echol = 15;  // A0/A1 as digital OK on Uno

// === Motor pins (L293D / TB6612 style) ===
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 7;
int PWMA = 5;   // PWM - left
int PWMB = 6;   // PWM - right
int stdby = 8;  // for TB6612

const int baseSpeed   = 150; // fwd speed
const int maxSpeed    = 150; // PID clamp
const int turnPWM     = 38; // turning speed (both wheels opposite)

float err = 0 ;

// === Button ===
const int buttonPin = 9;
unsigned long lastButtonPress = 0;
bool motorsEnabled = false;

// === PID gains (wall following) ===
float Kp = 1.824, Ki = 0.001, Kd = 0.44;
float integral = 0;
float lastError = 0;
const int integralLimit = 200; // no dt -> keep small

// === Control loop timing (not strictly used now) ===
unsigned long lastTime = 0;

// === Maze params ===
const int sideOpenThresh = 24;  // cm to consider cell opening
const int stopFront      = 10;  // cm hard stop to avoid collision
const int corridorThresh = 17;
const unsigned long PID_PERIOD_MS = 75;

// Motor compensation
const int   PWM_DEADBAND = 60;  // min PWM that actually moves your motors
const float LEFT_SCALE   = 1.00; // tweak if one side is stronger (e.g., 0.95)
const float RIGHT_SCALE  = 1.00;

// === Yaw integration ===
float yawZero = 0.0f;           // not used but kept for future
const float ANGLE_TOL = 3.0f;   // deg tolerance
float gzBias = 0.0f;            // gyro bias (dps)

// -------------------- MPU helpers --------------------
static inline int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();
  return gz_raw;
}

float readGyroZ() {
  // Sensitivity: 131 LSB/(°/s) at ±250 dps
  return (readGyroZRaw() / 131.0f) - gzBias; // dps after bias
}

void initMPU() {
  Wire.begin();

  // Wake up device
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro ±250 dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
}

void calibrateGyroZ(uint16_t samples = 500) {
  delay(300); // let it settle
  long sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  float avgRaw = sum / (float)samples;
  gzBias = avgRaw / 131.0f; // dps
}

//-------------------- Ultrasound --------------------
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);  // 20ms timeout
  // if (duration == 0) return 1000;              // no echo → far away
  return duration * 0.034 / 2;                 // cm
}


float getDistanceF(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);  // 20ms timeout
  if (duration == 0) return 1000;              // no echo → far away
  return duration * 0.034 / 2;                 // cm
}
// -------------------- Motion primitives --------------------
void stop() {

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(1000);
}

void setFwdDir() {
  // optional: you can omit this entirely and just call driveMotors in PID
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}


void setInPlaceRight() {
  // left fwd, right back -> sharper, symmetric turn
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void setInPlaceLeft() {
  // left back, right fwd
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void driveMotors(int left, int right) {
  // scale for asymmetry
  float lf = left  * LEFT_SCALE;
  float rf = right * RIGHT_SCALE;

  // deadband (both directions)
  if (lf > 0 && lf < PWM_DEADBAND) lf = PWM_DEADBAND;
  if (lf < 0 && lf > -PWM_DEADBAND) lf = -PWM_DEADBAND;
  if (rf > 0 && rf < PWM_DEADBAND) rf = PWM_DEADBAND;
  if (rf < 0 && rf > -PWM_DEADBAND) rf = -PWM_DEADBAND;

  // clamp
  lf = constrain((int)lf, -255, 255);
  rf = constrain((int)rf, -255, 255);

  // set direction pins once per side
  if (lf >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else         { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }

  if (rf >= 0) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  }
  else         { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }

  analogWrite(PWMA, abs((int)lf));
  analogWrite(PWMB, abs((int)rf));
}


// -------------------- PID (wall following) --------------------
unsigned long lastPID = 0;
float integ = 0, lastErr = 0;

void PID_step() {
  // read in an order that reduces echo overlap
  float leftDist  = getDistance(trigl, echol);
  delay(3);
  float rightDist = getDistance(trigr, echor);
  delay(3);
  float frontDist = getDistanceF(trigf, echof);
  delay(3);
  
  // error: center between walls
  float err = 0;
  static float lastLeft = corridorThresh/2;
  static float lastRight = corridorThresh/2;

// when both visible
 if (leftDist < corridorThresh && rightDist < corridorThresh) {
      err = rightDist - leftDist;
      lastLeft  = leftDist;
      lastRight = rightDist;
  }
  // else if (leftDist > corridorThresh && rightDist < corridorThresh) {
  //     // left wall gone → pretend it’s still at lastLeft
  //     err = rightDist - lastLeft;
  //     lastRight = rightDist;
  // }
  // else if (leftDist < corridorThresh && rightDist > corridorThresh) {
  //     // right wall gone → pretend it’s still at lastRight
  //     err = lastRight - leftDist;
  //     lastLeft = leftDist;
  //}
  else {
      // both gone → straight
      err = 0;
  }


  // timing
  unsigned long now = millis();
  float dt = (lastPID == 0) ? (PID_PERIOD_MS/1000.0f) : (now - lastPID)/1000.0f;
  lastPID = now;
  if (dt <= 0) dt = PID_PERIOD_MS/1000.0f;

  // PID (with anti-windup)
  integ += err * dt;
  integ = constrain(integ, -100.0f, 100.0f);
  float deriv = (err - lastErr) / dt;
  float corr  = Kp*err + Ki*integ + Kd*deriv;
  lastErr = err;

  int leftSpeed  = baseSpeed + (int)corr;
  int rightSpeed = baseSpeed - (int)corr;

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("err:"); Serial.print(err); 
  Serial.print(" corr:"); Serial.print(corr); 
  Serial.print(" LPWM:"); Serial.print(leftSpeed); 
  Serial.print(" RPWM:"); Serial.print(rightSpeed); 
  Serial.print(" F:"); Serial.print(frontDist);
  Serial.print(" L:"); Serial.print(leftDist);
  Serial.print(" R:"); Serial.println(rightDist);
}


// -------------------- Gyro-accurate turn --------------------
void turnByAngle(float targetAngleDeg) {
  // positive = right turn, negative = left turn
  unsigned long last = millis();
  float yaw = 0;

  // set direction based on sign & set turn speed
  if (targetAngleDeg < 0) setInPlaceRight();
  else                    setInPlaceLeft();
  analogWrite(PWMA, turnPWM);
  analogWrite(PWMB, turnPWM);

  const unsigned long failSafeMs = 4000; // 4s guard for 180°
  unsigned long start = millis();

  while (true) {
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;

    float gz_dps = readGyroZ();
    yaw += gz_dps * dt; // integrate

    float remaining = targetAngleDeg - yaw;
    if (fabs(remaining) <= ANGLE_TOL) break;

    if (now - start > failSafeMs) break; // safety exit

    Serial.print("Yaw: ");
    Serial.println(yaw);
  }

  stop(); // full stop at the end of the turn

  Serial.print("Turn complete: ");
  Serial.println(yaw);

  // integ=0;
  // lastErr=0;
  // lastPID=0;
}

// --- Debounce for side distances ---
// Keeps track of the last stable state internally
// bool debounceSide(bool currentState, bool *lastStable, unsigned long *lastChangeTime, unsigned long debounceMs = 80) {
//   unsigned long now = millis();

//   if (currentState != *lastStable) {
//     // State changed, check if stable long enough
//     if (now - *lastChangeTime >= debounceMs) {
//       *lastStable = currentState;  // Accept new state
//       *lastChangeTime = now;
//     }
//   } else {
//     // Reset timer when state matches
//     *lastChangeTime = now;
//   }

//   return *lastStable;
// }

// // -------------------- Button toggle --------------------
void toggleMotors() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(buttonPin);
  if (lastState == HIGH && currentState == LOW) {
    if (millis() - lastButtonPress > 250) { // debounce
      motorsEnabled = !motorsEnabled;
      digitalWrite(stdby, motorsEnabled ? HIGH : LOW);
      lastButtonPress = millis();
    }
  }
  lastState = currentState;
}


void rightWallControlStep() {
  // Single read set for decision (avoid double-pinging)
  float left  = getDistance(trigl, echol);
  delay(3);
  float right = getDistance(trigr, echor);
  delay(3);
  float front = getDistanceF(trigf, echof);
  delay(3);

  // bool leftOpen  = (left  > sideOpenThresh);     // opening to the left
  // bool frontOK   = (front > stopFront);          // safe to go forward
  // bool rightOpen = (right > sideOpenThresh);     // opening to the right

  // Left turn has highest priority whenever there is an opening
  if (right > sideOpenThresh) {
    // move forward slightly into the junction before stopping
    driveMotors(baseSpeed, baseSpeed);
    delay(230);   // tune this (200–300ms works for most bots)
    stop();
    
    turnByAngle(-90.0f);
    PID_step();
    delay(500);   // LEFT
    return;
  }

  // Straight if path ahead is safe
  else if (front > stopFront) {
    PID_step();            // keep centered while moving forward
    return;
  }

  // Right turn if blocked ahead but right is open
  else if (left > sideOpenThresh) {
    // move forward slightly into the junction before stopping
       // tune this (200–300ms works for most bots)
    stop();

    turnByAngle(+90.0f);   // RIGHT
    PID_step();
    delay(500);
    return;
  }

  else if (front < stopFront && right <sideOpenThresh && left < sideOpenThresh) {
    turnByAngle(-180.0f);
  }

 
}
// -------------------- Arduino setup/loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(trigf, OUTPUT);
  pinMode(trigl, OUTPUT);
  pinMode(trigr, OUTPUT);
  pinMode(echof, INPUT);
  pinMode(echol, INPUT);
  pinMode(echor, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(stdby, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(stdby, HIGH);
  analogWrite(PWMA, baseSpeed);
  analogWrite(PWMB, baseSpeed);

  Wire.begin();
  initMPU();
  calibrateGyroZ(600); // IMPORTANT: keep bot still during boot
  Serial.println("MPU initialized & gyro calibrated.");
}


void loop() {
  toggleMotors();
  if (!motorsEnabled) { driveMotors(0,0); return; }

  rightWallControlStep();
}