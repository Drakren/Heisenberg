#include <Wire.h>
#include <EEPROM.h>

// ==========================================
// 1. HARDWARE & PIN DEFINITIONS
// ==========================================
#define MPU_ADDR    0x69
#define PWR_MGMT_1  0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

// Ultrasonic pins
int trigf = 12, echof = 11;
int trigr = 9,  echor = 8;
int trigl = 13, echol = 10;

// Motor pins
int in1 = 7, in2 = 5;
int in3 = 4, in4 = 2;
int PWMA = 6, PWMB = 3;

// Encoder Pins
#define LEFT_A   A0
#define LEFT_B   A1
#define RIGHT_A  A2
#define RIGHT_B  A3

// ==========================================
// 2. CONSTANTS & GAINS
// ==========================================
const int exploreSpeed = 80;  // Safe, steady speed for mapping
const int runSpeed     = 130; // Aggressive speed for the final run!

const int turnPWM      = 38;  
const int PWM_DEADBAND = 30; 

const float LEFT_SCALE  = 1.00; 
const float RIGHT_SCALE = 1.00;

// Maze physicals
const int sideOpenThresh = 18;  
const int stopFront      = 5;   
const int corridorThresh = 16;
const long TICKS_FULL_CELL = 1030; 

// PID Gains
float Kp = 1.824, Ki = 0.001, Kd = 0.44;
float K_wall = 1.5, K_yaw = 0.9; 

// ==========================================
// 3. GLOBAL VARIABLES
// ==========================================
float leftDist = 10.0, rightDist = 10.0, frontDist = 10.0; 
float integ = 0, lastErr = 0;
unsigned long lastPID = 0;

volatile long leftCount = 0;
volatile long rightCount = 0;
uint8_t lastPortC;

float currentYaw = 0.0;
unsigned long lastYawTime = 0;
float gzBias = 0.0f;
float filteredRate = 0.0; 
const float ANGLE_TOL = 3.0f; 
float alpha = 0.15; 

// ==========================================
// 4. FLOODFILL & MAZE VARIABLES
// ==========================================
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

byte currentX = 0;
byte currentY = 0;
byte currentHeading = NORTH; 

byte distances[16][16];
byte walls[16][16]; 

// EEPROM Addresses
const int EEPROM_ADDR_WALLS = 0;
const int EEPROM_ADDR_DIST  = 256;
const int EEPROM_ADDR_FLAG  = 512;
const int EEPROM_ADDR_RESET = 513; // For double reset detection

bool speedRunMode = false;
bool goalReached = false;

// Targets for returning to start
bool returningToStart = false;
byte targetX = 3; 
byte targetY = 3;

// ==========================================
// 5. INTERRUPTS & SENSORS
// ==========================================
ISR(PCINT1_vect) {
  uint8_t currentPortC = PINC;
  uint8_t changed = currentPortC ^ lastPortC;

  if (changed & (1 << PC0)) {
    if (currentPortC & (1 << PC0)) {  
      if (currentPortC & (1 << PC1)) leftCount--;
      else leftCount++;
    }
  }
  if (changed & (1 << PC2)) {
    if (currentPortC & (1 << PC2)) {  
      if (currentPortC & (1 << PC3)) rightCount++;
      else rightCount--;
    }
  }
  lastPortC = currentPortC;
}

static inline int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

float readGyroZ() { 
  return (readGyroZRaw() / 131.0f) - gzBias; 
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(PWR_MGMT_1); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(GYRO_CONFIG); Wire.write(0x00); Wire.endTransmission();
}

void calibrateGyroZ(uint16_t samples = 500) {
  delay(300);
  long sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  gzBias = (sum / (float)samples) / 131.0f; 
}

void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastYawTime) / 1000.0f;
  lastYawTime = now;
  if (dt > 0 && dt < 0.1) { 
      filteredRate = (alpha * readGyroZ()) + ((1.0f - alpha) * filteredRate);
      currentYaw += filteredRate * dt;
  }
}

float getDistanceSafe(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 5000); 
  if (duration == 0) return 1000; 
  return duration * 0.034 / 2; 
}

void readAllSensors() {
  leftDist  = getDistanceSafe(trigl, echol); delay(15);
  frontDist = getDistanceSafe(trigf, echof); delay(15);
  rightDist = getDistanceSafe(trigr, echor); delay(15);
}

// ==========================================
// 6. MOTOR CONTROL PRIMITIVES
// ==========================================
void stopMotors() {
  digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
  analogWrite(PWMA, 255);  analogWrite(PWMB, 255);
  
  delay(60); 
  
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(PWMA, 0);   analogWrite(PWMB, 0);
}

void driveMotors(int left, int right) {
  float lf = constrain(left * LEFT_SCALE, -255, 255);
  float rf = constrain(right * RIGHT_SCALE, -255, 255);

  if (abs(lf) < PWM_DEADBAND) lf = (lf > 0) ? PWM_DEADBAND : -PWM_DEADBAND;
  if (abs(rf) < PWM_DEADBAND) rf = (rf > 0) ? PWM_DEADBAND : -PWM_DEADBAND;

  if (lf >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else         { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }

  if (rf >= 0) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  }
  else         { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }

  analogWrite(PWMA, abs((int)lf));
  analogWrite(PWMB, abs((int)rf));
}

void resetPID() {
  integ = 0; lastErr = 0; currentYaw = 0; filteredRate = 0.0;
  lastYawTime = millis(); lastPID = millis();
}

void turnByAngle(float targetAngleDeg) {
  resetPID();
  if (targetAngleDeg < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); 
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  
  } else {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); 
  }
  
  analogWrite(PWMA, turnPWM); analogWrite(PWMB, turnPWM);
  float targetAbs = fabs(targetAngleDeg);

  while (true) {
    updateYaw();
    if (fabs(currentYaw) >= targetAbs - ANGLE_TOL) break;
  }
  stopMotors(); delay(100);
}

void driveCellsForward(int numCells) {
  resetPID();
  noInterrupts(); leftCount = 0; rightCount = 0; interrupts();
  
  long targetTicks = TICKS_FULL_CELL * numCells;
  
  int activeSpeed = speedRunMode ? runSpeed : exploreSpeed;

  while (true) {
    noInterrupts();
    long currentL = abs(leftCount); long currentR = abs(rightCount);
    interrupts();

    if ((currentL + currentR) / 2 >= targetTicks) break;

    updateYaw();
    
    leftDist  = getDistanceSafe(trigl, echol);
    frontDist = getDistanceSafe(trigf, echof);
    rightDist = getDistanceSafe(trigr, echor);
    
    if (frontDist > 0 && frontDist < stopFront) {
      Serial.println(F("Crash avoided! Stopping early."));
      break; 
    }
    
    float err = 0;
    float idealDist = corridorThresh / 2.0; 
    
    if (leftDist < corridorThresh && rightDist < corridorThresh) {
        err = rightDist - leftDist;
    } 
    else if (leftDist < corridorThresh) {
        float wallErr = idealDist - leftDist;
        err = (wallErr * K_wall) - (currentYaw * K_yaw);
    } 
    else if (rightDist < corridorThresh) {
        float wallErr = rightDist - idealDist;
        err = (wallErr * K_wall) - (currentYaw * K_yaw);
    } 
    else {
        err = -currentYaw * K_yaw;
    }

    unsigned long now = millis();
    float dt = (now - lastPID) / 1000.0f;
    lastPID = now;
    if (dt <= 0) dt = 0.05;

    integ += err * dt;
    integ = constrain(integ, -100.0f, 100.0f);
    float deriv = (err - lastErr) / dt;
    float corr  = Kp*err + Ki*integ + Kd*deriv;
    lastErr = err;

    driveMotors(activeSpeed - (int)corr, activeSpeed + (int)corr);
  }
  stopMotors(); 
  
  if (!speedRunMode) {
    delay(100);
  }
}

// ==========================================
// 7. MAZE & ALGORITHM LOGIC
// ==========================================
bool hasWall(byte x, byte y, byte dir) {
  return (walls[x][y] & (1 << dir)) != 0;
}

void setWall(byte x, byte y, byte dir) {
  walls[x][y] |= (1 << dir);
  if (dir == NORTH && y < 15) walls[x][y+1] |= (1 << SOUTH);
  if (dir == SOUTH && y > 0)  walls[x][y-1] |= (1 << NORTH);
  if (dir == EAST  && x < 15) walls[x+1][y] |= (1 << WEST);
  if (dir == WEST  && x > 0)  walls[x-1][y] |= (1 << EAST);
}

void mapWalls() {
  if (speedRunMode) return; 
  readAllSensors();
  if (frontDist < sideOpenThresh) setWall(currentX, currentY, currentHeading);
  if (rightDist < sideOpenThresh) setWall(currentX, currentY, (currentHeading + 1) % 4);
  if (leftDist < sideOpenThresh) setWall(currentX, currentY, (currentHeading + 3) % 4);
}

void initializeMaze() {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      walls[x][y] = 0;
      distances[x][y] = 255; 
      if (x == 0)  setWall(x, y, WEST);
      if (x == 15) setWall(x, y, EAST);
      if (y == 0)  setWall(x, y, SOUTH);
      if (y == 15) setWall(x, y, NORTH);
    }
  }
  setWall(0, 4, SOUTH);
}

void printMaze() {
  Serial.println(F("========================================"));
  Serial.print(F("Robot at: ("));
  Serial.print(currentX); Serial.print(F(","));
  Serial.print(currentY); Serial.print(F(") | Heading: "));
  
  if (currentHeading == NORTH) Serial.println(F("NORTH ^"));
  else if (currentHeading == EAST) Serial.println(F("EAST >"));
  else if (currentHeading == SOUTH) Serial.println(F("SOUTH v"));
  else if (currentHeading == WEST) Serial.println(F("WEST <"));
  Serial.println(F("----------------------------------------"));

  for (int y = 15; y >= 0; y--) {
    for (int x = 0; x < 16; x++) {
      if (x == currentX && y == currentY) {
        Serial.print(F("[XX] ")); 
      } else if (x == 3 && y == 3) {
        Serial.print(F(" ** "));
      } else {
        byte dist = distances[x][y];
        if (dist == 255) {
          Serial.print(F(" --  ")); 
        } else {
          if (dist < 10) Serial.print(F("  "));
          else if (dist < 100) Serial.print(F(" "));
          Serial.print(dist);
          Serial.print(F("  "));
        }
      }
    }
    Serial.println(); 
  }
  Serial.println(F("========================================"));
}

void floodFill() {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      distances[x][y] = 255;
    }
  }

  byte qX[256], qY[256];
  byte head = 0, tail = 0;

  // Dynamically paths to targetX and targetY
  byte goals[1][2] = {{targetX, targetY}};
  for(int i=0; i<1; i++) {
    distances[goals[i][0]][goals[i][1]] = 0;
    qX[tail] = goals[i][0]; qY[tail] = goals[i][1];
    tail++;
  }

  while (head != tail) {
    byte cx = qX[head]; byte cy = qY[head];
    head++;
    byte currentDist = distances[cx][cy];

    if (!hasWall(cx, cy, NORTH) && distances[cx][cy+1] == 255) {
      distances[cx][cy+1] = currentDist + 1;
      qX[tail] = cx; qY[tail] = cy+1; tail++;
    }
    if (!hasWall(cx, cy, EAST) && distances[cx+1][cy] == 255) {
      distances[cx+1][cy] = currentDist + 1;
      qX[tail] = cx+1; qY[tail] = cy; tail++;
    }
    if (!hasWall(cx, cy, SOUTH) && distances[cx][cy-1] == 255) {
      distances[cx][cy-1] = currentDist + 1;
      qX[tail] = cx; qY[tail] = cy-1; tail++;
    }
    if (!hasWall(cx, cy, WEST) && distances[cx-1][cy] == 255) {
      distances[cx-1][cy] = currentDist + 1;
      qX[tail] = cx-1; qY[tail] = cy; tail++;
    }
  }
}

byte calculateNextHeading(byte checkX, byte checkY, byte checkHeading) {
  byte bestDir = checkHeading; 
  byte minVal = 255;

  for (int i = 0; i < 4; i++) {
    byte checkDir = (checkHeading + i) % 4; 
    
    if (!hasWall(checkX, checkY, checkDir)) {
      byte neighborVal = 255;
      if (checkDir == NORTH) neighborVal = distances[checkX][checkY+1];
      if (checkDir == EAST)  neighborVal = distances[checkX+1][checkY];
      if (checkDir == SOUTH) neighborVal = distances[checkX][checkY-1];
      if (checkDir == WEST)  neighborVal = distances[checkX-1][checkY];

      if (neighborVal < minVal) {
        minVal = neighborVal;
        bestDir = checkDir;
      }
    }
  }
  return bestDir;
}

int getContinuousStraightCells() {
  int cells = 1;
  byte scanX = currentX;
  byte scanY = currentY;
  byte scanHeading = currentHeading;

  while(true) {
    if (scanHeading == NORTH) scanY++;
    if (scanHeading == EAST)  scanX++;
    if (scanHeading == SOUTH) scanY--;
    if (scanHeading == WEST)  scanX--;

    if (distances[scanX][scanY] == 0) break;

    byte nextPredictedHeading = calculateNextHeading(scanX, scanY, scanHeading);
    
    if (nextPredictedHeading == scanHeading) {
      cells++;
    } else {
      break; 
    }
  }
  return cells;
}

void turnTo(byte targetHeading) {
  if (targetHeading == currentHeading) return; 

  int turnDiff = targetHeading - currentHeading;
  if (turnDiff == 1 || turnDiff == -3) {
    turnByAngle(-90.0); 
  } else if (turnDiff == -1 || turnDiff == 3) {
    turnByAngle(90.0); 
  } else {
    turnByAngle(180.0); 
  }
  currentHeading = targetHeading;
}

void updateCoordinates() {
  if (currentHeading == NORTH) currentY++;
  if (currentHeading == EAST)  currentX++;
  if (currentHeading == SOUTH) currentY--;
  if (currentHeading == WEST)  currentX--;
}

// ==========================================
// 8. EEPROM STORAGE
// ==========================================
void saveMaze() {
  EEPROM.put(EEPROM_ADDR_WALLS, walls);
  EEPROM.put(EEPROM_ADDR_DIST, distances);
  EEPROM.put(EEPROM_ADDR_FLAG, (byte)0xAA); 
  Serial.println(F("Saved Map to EEPROM!"));
}

void loadMaze() {
  byte flag;
  EEPROM.get(EEPROM_ADDR_FLAG, flag);
  if (flag == 0xAA) {
    EEPROM.get(EEPROM_ADDR_WALLS, walls);
    EEPROM.get(EEPROM_ADDR_DIST, distances);
    speedRunMode = true;
    Serial.println(F("Map Loaded! Speed Run Mode."));
  } else {
    Serial.println(F("No valid map in EEPROM. Exploring."));
    initializeMaze();
  }
}

// ==========================================
// 9. MAIN ARDUINO SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(trigf, OUTPUT); pinMode(echof, INPUT);
  pinMode(trigl, OUTPUT); pinMode(echol, INPUT);
  pinMode(trigr, OUTPUT); pinMode(echor, INPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  pinMode(LEFT_A, INPUT_PULLUP);  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP); pinMode(RIGHT_B, INPUT_PULLUP);
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
  lastPortC = PINC;

  Wire.begin();
  initMPU();
  calibrateGyroZ(500); 

  // Double-Press Reset Logic (Kept untouched as requested)
  byte resetFlag;
  EEPROM.get(EEPROM_ADDR_RESET, resetFlag);
  
  if (resetFlag == 0xBB) {
    // Flag is 0xBB, board was reset TWICE in a row
    EEPROM.put(EEPROM_ADDR_RESET, (byte)0x00); // Clear the flag
    loadMaze();
    Serial.println(F("Double Reset Detected! Speed Run starts in 3 seconds..."));
    delay(3000); 
  } else {
    // First boot / single reset. Set the flag to 0xBB.
    EEPROM.put(EEPROM_ADDR_RESET, (byte)0xBB);
    // Give the user 750 milliseconds to press reset again to trigger the mode.
    delay(750); 
    initializeMaze();
  }
}

void loop() {
  // Clear the double-press flag once the main loop starts safely
  static bool firstLoop = true;
  if (firstLoop) {
    EEPROM.put(EEPROM_ADDR_RESET, (byte)0x00);
    firstLoop = false;
  }

  if (goalReached) return; 

  if (distances[currentX][currentY] == 0) {
    stopMotors();
    
    if (!speedRunMode && !returningToStart) {
      // Phase 1: Reached the center! Switch target to (0,0)
      Serial.println(F("Center reached! Navigating back to Start..."));
      returningToStart = true;
      targetX = 0;
      targetY = 0;
      floodFill(); // Recalculate immediately so distance is > 0
      delay(1000); // Pause for a second before doing the 180 turn
    } 
    else if (!speedRunMode && returningToStart) {
      // === UPDATED PHASE 2 LOGIC (15-SECOND DELAY) ===
      Serial.println(F("Mapping complete! 180 Turn and 15s Delay..."));
      
      // 1. Point the map back to the center (3,3) for the speed run
      targetX = 3; 
      targetY = 3;
      floodFill(); 
      saveMaze();  // Save it to EEPROM just in case!
      
      // 2. Execute the 180-degree turn to face back into the maze
      byte nextHeading = calculateNextHeading(currentX, currentY, currentHeading);
      turnTo(nextHeading);
      
      // 3. Stop for exactly 15 seconds so you can place the bot perfectly by hand
      stopMotors();
      Serial.println(F("Bot stopped. Place it perfectly! You have 15 seconds..."));
      delay(7000);
      
      // 4. Reset encoders and PID right before launch so manual adjustments aren't registered as tracking errors
      noInterrupts(); 
      leftCount = 0; 
      rightCount = 0; 
      interrupts();
      resetPID();
      
      // 5. Engage speed run mode and instantly launch
      speedRunMode = true;
      Serial.println(F("Speed Run Engaged! Launching..."));
      return; 
    } 
    else if (speedRunMode) {
      // Phase 3: Reached the center during the speed run!
      Serial.println(F("Speed Run Finished!"));
      goalReached = true;
      while(true); 
    }
    
    return; // Skip the rest of this loop iteration
  }

  if (!speedRunMode) {
    mapWalls();
    floodFill();
  }

  byte nextHeading = calculateNextHeading(currentX, currentY, currentHeading);
  turnTo(nextHeading);

  int cellsToDrive = 1;
  if (speedRunMode) {
    cellsToDrive = getContinuousStraightCells(); 
  }

  driveCellsForward(cellsToDrive);

  for (int i = 0; i < cellsToDrive; i++) {
    updateCoordinates();
  }
}
