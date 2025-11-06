// ---------------- PIN CONFIGURATION ----------------
const int trigPin = 9;
const int echoPin = 8;

// Analog IR sensors
const int irLeft = A0;
const int irRight = A1;

// Motor driver pins (L298N/L293D)
const int IN1 = 5; // Left motor forward
const int IN2 = 4; // Left motor backward
const int IN3 = 7; // Right motor forward
const int IN4 = 6; // Right motor backward

// ---------------- NEURO-FUZZY PARAMETERS ----------------
float wNear = 0.7;
float wMedium = 0.4;
float wFar = 0.1;

// Dynamic weights for side sensors
float wLeft = 0.5;
float wRight = 0.5;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("=== Neuro-Fuzzy Obstacle Avoidance (Dynamic Weight + Motor Control) ===");
}

// ---------------- ULTRASONIC READ FUNCTION ----------------
long readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// ---------------- MOTOR CONTROL FUNCTIONS ----------------
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ---------------- FUZZY MEMBERSHIP FUNCTIONS (DISTANCE) ----------------
float fuzzyNear(long d) {
  if (d <= 10) return 1.0;
  else if (d <= 20) return (20.0 - d) / 10.0;
  else return 0.0;
}

float fuzzyMedium(long d) {
  if (d <= 10 || d >= 40) return 0.0;
  else if (d <= 25) return (d - 10.0) / 15.0;
  else return (40.0 - d) / 15.0;
}

float fuzzyFar(long d) {
  if (d <= 25) return 0.0;
  else if (d <= 50) return (d - 25.0) / 25.0;
  else return 1.0;
}

// ---------------- FUZZY MEMBERSHIP FUNCTIONS (IR INTENSITY) ----------------
float fuzzyLow(int val) {
  if (val <= 200) return 1.0;
  else if (val <= 400) return (400.0 - val) / 200.0;
  else return 0.0;
}

float fuzzyMediumIR(int val) {
  if (val <= 300 || val >= 700) return 0.0;
  else if (val <= 500) return (val - 300.0) / 200.0;
  else return (700.0 - val) / 200.0;
}

float fuzzyHigh(int val) {
  if (val <= 600) return 0.0;
  else if (val <= 900) return (val - 600.0) / 300.0;
  else return 1.0;
}

// ---------------- DYNAMIC WEIGHT UPDATE ----------------
void updateWeights(int leftVal, int rightVal) {
  float leftIntensity = constrain(leftVal / 1023.0, 0.0, 1.0);
  float rightIntensity = constrain(rightVal / 1023.0, 0.0, 1.0);

  wLeft = 1.0 - leftIntensity;
  wRight = 1.0 - rightIntensity;

  float sum = wLeft + wRight;
  wLeft /= sum;
  wRight /= sum;
}

// ---------------- MAIN LOOP ----------------
void loop() {
  long distance = readUltrasonic();
  int leftVal = analogRead(irLeft);
  int rightVal = analogRead(irRight);

  updateWeights(leftVal, rightVal);

  float nearVal = fuzzyNear(distance);
  float medVal = fuzzyMedium(distance);
  float farVal = fuzzyFar(distance);

  float leftMed = fuzzyMediumIR(leftVal);
  float leftHigh = fuzzyHigh(leftVal);
  float rightMed = fuzzyMediumIR(rightVal);
  float rightHigh = fuzzyHigh(rightVal);

  float actLeft = (leftMed * 0.5 + leftHigh * 1.0) * wLeft;
  float actRight = (rightMed * 0.5 + rightHigh * 1.0) * wRight;

  Serial.print("Dist: "); Serial.print(distance);
  Serial.print(" cm | L_IR: "); Serial.print(leftVal);
  Serial.print(" | R_IR: "); Serial.print(rightVal);
  Serial.print(" | wL: "); Serial.print(wLeft, 2);
  Serial.print(" | wR: "); Serial.print(wRight, 2);
  Serial.print(" --> ");

  // ---------------- FUZZY DECISION RULES WITH MOTION ----------------
  if (actLeft > 0.6 && actRight > 0.6 && nearVal < 0.4) {
    Serial.println("Blocked both sides → BACK");
    moveBackward();
    delay(600);
    stopMotors();
  }
  else if (actLeft > 0.6 && actRight < 0.4) {
    Serial.println("Left obstacle → TURN RIGHT");
    turnRight();
    delay(400);
  }
  else if (actRight > 0.6 && actLeft < 0.4) {
    Serial.println("Right obstacle → TURN LEFT");
    turnLeft();
    delay(400);
  }
  else if (nearVal > 0.6) {
    if (wLeft > wRight) {
      Serial.println("Front obstacle → BACK + LEFT turn");
      moveBackward();
      delay(500);
      turnLeft();
      delay(400);
    } else {
      Serial.println("Front obstacle → BACK + RIGHT turn");
      moveBackward();
      delay(500);
      turnRight();
      delay(400);
    }
  }
  else if (medVal > 0.3) {
    if (actLeft > actRight) {
      Serial.println("Medium + Slight left → Adjust RIGHT");
      turnRight();
      delay(300);
    } else if (actRight > actLeft) {
      Serial.println("Medium + Slight right → Adjust LEFT");
      turnLeft();
      delay(300);
    } else {
      Serial.println("Medium + Balanced → Slow Forward");
      moveForward();
      delay(300);
    }
  }
  else if (farVal > 0.3 && actLeft < 0.3 && actRight < 0.3) {
    Serial.println("Far + Clear → FORWARD");
    moveForward();
  }
  else {
    Serial.println("Uncertain → STOP & Scan");
    stopMotors();
  }

  delay(300);
}
