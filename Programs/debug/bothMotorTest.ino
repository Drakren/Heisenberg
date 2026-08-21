// Motor Pins
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 7;

int en1 = 5;
int en2 = 6;
int stdby = 8;

int buttonPin = 9;  // Button at D9

unsigned long buttonPressTime = 0;
bool buttonHeld = false;
bool motorsEnabled = false; // Toggle state

void setup() {
  Serial.begin(9600);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  pinMode(stdby, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP); // Button with pull-up resistor

  // Start with motors OFF
  digitalWrite(stdby, LOW);
}

void moveForward() {
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  // Button pressed (active LOW because of INPUT_PULLUP)
  if (buttonState == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis(); // Start timing
    }
    if ((millis() - buttonPressTime) >= 2000 && !buttonHeld) {
      // Toggle state after 2s hold
      motorsEnabled = !motorsEnabled;
      buttonHeld = true; 
    }
  } else {
    // Button released → reset press timer
    buttonPressTime = 0;
    buttonHeld = false;
  }

  // Apply motor state
  if (motorsEnabled) {
    digitalWrite(stdby, HIGH);    
    analogWrite(en1, 150);
    analogWrite(en2, 150);
    moveForward();
  } else {
    digitalWrite(stdby, LOW);
  }
}
