#include <Wire.h>

#define MPU_ADDR 0x68
#define PWR_MGMT_1  0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

unsigned long lastTime;

// === Initialize MPU9250 ===
void initMPU() {
  Wire.begin();
  
  // Wake up
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

// === Read raw gyro Z (deg/sec) ===
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t gz_raw = Wire.read() << 8 | Wire.read();
  return gz_raw / 131.0;  // dps
}

// === Turn by target angle ===
void turnByAngle(float targetAngleDeg) {
  float yaw = 0;
  lastTime = millis();

  // Decide direction
  if (targetAngleDeg > 0) {
    // Turning right
    // setMotorRight();   // your motor code here
  } else {
    // Turning left
    // setMotorLeft();
  }

  while (abs(yaw) < abs(targetAngleDeg)) {
    float gz_dps = readGyroZ();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    yaw += gz_dps * dt;

    Serial.print("Yaw: ");
    Serial.println(yaw);
  }

  // Stop motors after turn
  // stopMotors();

  Serial.print("Turn complete: ");
  Serial.println(yaw);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initMPU();
  Serial.println("MPU Initialized!");
}

void loop() {
  // Example usage:
  turnByAngle(90);   // right turn
  delay(2000);

  turnByAngle(-90);  // left turn
  delay(2000);

  turnByAngle(180);  // U-turn
  delay(2000);
}
