#define TRIG_PIN 10
#define ECHO_PIN 11

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time (max 20 ms timeout = ~3.4m)
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);

  // Convert to cm
  float distance;
  if (duration == 0) {
    distance = -1; // no reading
  } else {
    distance = (duration * 0.034) / 2.0;
  }

  // Print result
  Serial.print("Right Sensor: ");
  if (distance < 0) {
    Serial.println("No Echo");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(200); // small delay before next reading
}
