/*
 * Mecanum Wheel Wiring Test Script
 * For Arduino Mega with 4 mecanum wheels
 * Using 2x L298N motor drivers
 *
 * L298N #1 (Left Side):
 *   Motor A (Front Left): ENA=Pin6, IN1=Pin2, IN2=Pin3
 *   Motor B (Rear Left):  ENB=Pin7, IN3=Pin4, IN4=Pin5
 *
 * L298N #2 (Right Side):
 *   Motor A (Front Right): ENA=Pin12, IN1=Pin8, IN2=Pin9
 *   Motor B (Rear Right):  ENB=Pin44, IN3=Pin10, IN4=Pin11
 *   NOTE: Avoid pin 13 - it has built-in LED that interferes with PWM
 */

// L298N #1 - Left Side Motors
// Front Left Motor (Motor A)
const int FL_PWM = 6;    // ENA
const int FL_DIR1 = 2;   // IN1
const int FL_DIR2 = 3;   // IN2

// Rear Left Motor (Motor B)
const int RL_PWM = 7;    // ENB
const int RL_DIR1 = 4;   // IN3
const int RL_DIR2 = 5;   // IN4

// L298N #2 - Right Side Motors
// Front Right Motor (Motor A)
const int FR_PWM = 12;   // ENA
const int FR_DIR1 = 8;   // IN1
const int FR_DIR2 = 9;   // IN2

// Rear Right Motor (Motor B)
const int RR_PWM = 44;   // ENB (changed from 13 - pin 13 has built-in LED)
const int RR_DIR1 = 10;  // IN3
const int RR_DIR2 = 11;  // IN4

// Test parameters
const int TEST_SPEED = 150;  // PWM value (0-255)
const int TEST_DURATION = 2000;  // milliseconds
const int PAUSE_DURATION = 1000;  // pause between tests

void setup() {
  Serial.begin(115200);

  // Initialize all motor pins
  pinMode(FL_PWM, OUTPUT);
  pinMode(FL_DIR1, OUTPUT);
  pinMode(FL_DIR2, OUTPUT);

  pinMode(FR_PWM, OUTPUT);
  pinMode(FR_DIR1, OUTPUT);
  pinMode(FR_DIR2, OUTPUT);

  pinMode(RL_PWM, OUTPUT);
  pinMode(RL_DIR1, OUTPUT);
  pinMode(RL_DIR2, OUTPUT);

  pinMode(RR_PWM, OUTPUT);
  pinMode(RR_DIR1, OUTPUT);
  pinMode(RR_DIR2, OUTPUT);

  // Stop all motors initially
  stopAllMotors();

  Serial.println("======================================");
  Serial.println("Mecanum Wheel Wiring Test");
  Serial.println("======================================");
  Serial.println();
  Serial.println("This test will run each wheel in both directions.");
  Serial.println("Watch each wheel and verify it spins as indicated.");
  Serial.println();
  Serial.println("Waiting 3 seconds before starting...");
  delay(3000);
}

void loop() {
  // Test Front Left wheel
  Serial.println("\n--- Testing FRONT LEFT wheel ---");
  Serial.println("Direction: FORWARD");
  setMotor(FL_PWM, FL_DIR1, FL_DIR2, TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION);

  Serial.println("Direction: BACKWARD");
  setMotor(FL_PWM, FL_DIR1, FL_DIR2, -TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION * 2);

  // Test Front Right wheel
  Serial.println("\n--- Testing FRONT RIGHT wheel ---");
  Serial.println("Direction: FORWARD");
  setMotor(FR_PWM, FR_DIR1, FR_DIR2, TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION);

  Serial.println("Direction: BACKWARD");
  setMotor(FR_PWM, FR_DIR1, FR_DIR2, -TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION * 2);

  // Test Rear Left wheel
  Serial.println("\n--- Testing REAR LEFT wheel ---");
  Serial.println("Direction: FORWARD");
  setMotor(RL_PWM, RL_DIR1, RL_DIR2, TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION);

  Serial.println("Direction: BACKWARD");
  setMotor(RL_PWM, RL_DIR1, RL_DIR2, -TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION * 2);

  // Test Rear Right wheel
  Serial.println("\n--- Testing REAR RIGHT wheel ---");
  Serial.println("Direction: FORWARD");
  setMotor(RR_PWM, RR_DIR1, RR_DIR2, TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION);

  Serial.println("Direction: BACKWARD");
  setMotor(RR_PWM, RR_DIR1, RR_DIR2, -TEST_SPEED);
  delay(TEST_DURATION);
  stopAllMotors();
  delay(PAUSE_DURATION * 2);

  // Test complete
  Serial.println("\n======================================");
  Serial.println("Test cycle complete!");
  Serial.println("======================================");
  Serial.println("Waiting 5 seconds before repeating...");
  delay(5000);
}

void setMotor(int pwmPin, int dir1Pin, int dir2Pin, int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, abs(speed));
  } else if (speed < 0) {
    // Backward
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else {
    // Stop
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopAllMotors() {
  setMotor(FL_PWM, FL_DIR1, FL_DIR2, 0);
  setMotor(FR_PWM, FR_DIR1, FR_DIR2, 0);
  setMotor(RL_PWM, RL_DIR1, RL_DIR2, 0);
  setMotor(RR_PWM, RR_DIR1, RR_DIR2, 0);
}
