/*
 * LukeBot Motor Control
 * Arduino Mega firmware for Mecanum wheel control
 *
 * Handles low-level motor control via L298N motor drivers
 * Receives high-level commands from Jetson via USB Serial
 *
 * Motor Configuration:
 * - L298N #1 (Left side): Front-left (FL) and Rear-left (RL) motors
 * - L298N #2 (Right side): Front-right (FR) and Rear-right (RR) motors
 *
 * Pin Configuration:
 * L298N #1:
 *   IN1 -> Pin 2, IN2 -> Pin 3, IN3 -> Pin 4, IN4 -> Pin 5
 *   ENA -> Pin 6 (PWM), ENB -> Pin 7 (PWM)
 *
 * L298N #2:
 *   IN1 -> Pin 8, IN2 -> Pin 9, IN3 -> Pin 10, IN4 -> Pin 11
 *   ENA -> Pin 12 (PWM), ENB -> Pin 44 (PWM)
 *   NOTE: Avoid pin 13 - it has built-in LED that interferes with PWM
 */

// Motor pins - L298N #1 (Left side)
const int L298N1_IN1 = 2;
const int L298N1_IN2 = 3;
const int L298N1_IN3 = 4;
const int L298N1_IN4 = 5;
const int L298N1_ENA = 6;  // PWM
const int L298N1_ENB = 7;  // PWM

// Motor pins - L298N #2 (Right side)
const int L298N2_IN1 = 8;
const int L298N2_IN2 = 9;
const int L298N2_IN3 = 10;
const int L298N2_IN4 = 11;
const int L298N2_ENA = 12; // PWM
const int L298N2_ENB = 44; // PWM (changed from 13 - pin 13 has built-in LED)

const int MIN_SPEED = 30;
const int MAX_SPEED = 255;

// Rotation rate calibration (degrees per second at speed=50)
// This is an estimate - tune based on actual robot behavior
const float ROTATION_RATE_AT_50_SPEED = 30.0;  // degrees/second at speed=50

void setup() {
  Serial.begin(115200);

  // Initialize all pins
  pinMode(L298N1_IN1, OUTPUT);
  pinMode(L298N1_IN2, OUTPUT);
  pinMode(L298N1_IN3, OUTPUT);
  pinMode(L298N1_IN4, OUTPUT);
  pinMode(L298N1_ENA, OUTPUT);
  pinMode(L298N1_ENB, OUTPUT);

  pinMode(L298N2_IN1, OUTPUT);
  pinMode(L298N2_IN2, OUTPUT);
  pinMode(L298N2_IN3, OUTPUT);
  pinMode(L298N2_IN4, OUTPUT);
  pinMode(L298N2_ENA, OUTPUT);
  pinMode(L298N2_ENB, OUTPUT);

  stopAllMotors();

  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      parseCommand(command);
    }
  }
}

void parseCommand(String command) {
  if (command.length() == 0) {
    return;
  }

  // Parse command
  int commaIndex = command.indexOf(',');
  String cmd;
  String param;

  if (commaIndex > 0) {
    cmd = command.substring(0, commaIndex);
    param = command.substring(commaIndex + 1);
  } else {
    cmd = command;
    param = "";
  }

  cmd.toUpperCase();
  cmd.trim();

  if (cmd == "MOVE_FORWARD") {
    int speed = (param.length() > 0) ? param.toInt() : 50;
    speed = constrain(speed, 0, 100);
    moveForward(speed);
  }
  else if (cmd == "MOVE_BACKWARD") {
    int speed = (param.length() > 0) ? param.toInt() : 50;
    speed = constrain(speed, 0, 100);
    moveBackward(speed);
  }
  else if (cmd == "TURN_LEFT") {
    // Parse angle,speed format (e.g., "90,30" = 90 degrees at 30% speed)
    float angle = 90.0;  // default
    int speed = 50;      // default

    if (param.length() > 0) {
      int commaIndex = param.indexOf(',');
      if (commaIndex > 0) {
        angle = param.substring(0, commaIndex).toFloat();
        speed = param.substring(commaIndex + 1).toInt();
      } else {
        angle = param.toFloat();
      }
    }
    speed = constrain(speed, 0, 100);
    turnLeft(angle, speed);
  }
  else if (cmd == "TURN_RIGHT") {
    // Parse angle,speed format (e.g., "90,30" = 90 degrees at 30% speed)
    float angle = 90.0;  // default
    int speed = 50;      // default

    if (param.length() > 0) {
      int commaIndex = param.indexOf(',');
      if (commaIndex > 0) {
        angle = param.substring(0, commaIndex).toFloat();
        speed = param.substring(commaIndex + 1).toInt();
      } else {
        angle = param.toFloat();
      }
    }
    speed = constrain(speed, 0, 100);
    turnRight(angle, speed);
  }
  else if (cmd == "STOP") {
    stopAllMotors();
  }
  else {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(cmd);
  }
}

void moveForward(int speed) {
  int pwm = map(speed, 0, 100, MIN_SPEED, MAX_SPEED);

  // All wheels forward
  digitalWrite(L298N1_IN1, HIGH);
  digitalWrite(L298N1_IN2, LOW);
  digitalWrite(L298N1_IN3, HIGH);
  digitalWrite(L298N1_IN4, LOW);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);

  digitalWrite(L298N2_IN1, HIGH);
  digitalWrite(L298N2_IN2, LOW);
  digitalWrite(L298N2_IN3, HIGH);
  digitalWrite(L298N2_IN4, LOW);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);

  Serial.print("OK: Moving forward at speed ");
  Serial.println(speed);
}

void moveBackward(int speed) {
  int pwm = map(speed, 0, 100, MIN_SPEED, MAX_SPEED);

  // All wheels backward
  digitalWrite(L298N1_IN1, LOW);
  digitalWrite(L298N1_IN2, HIGH);
  digitalWrite(L298N1_IN3, LOW);
  digitalWrite(L298N1_IN4, HIGH);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);

  digitalWrite(L298N2_IN1, LOW);
  digitalWrite(L298N2_IN2, HIGH);
  digitalWrite(L298N2_IN3, LOW);
  digitalWrite(L298N2_IN4, HIGH);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);

  Serial.print("OK: Moving backward at speed ");
  Serial.println(speed);
}

void turnLeft(float angle, int speed) {
  speed = constrain(speed, 0, 100);
  int pwm = map(speed, 0, 100, MIN_SPEED, MAX_SPEED);

  // Left wheels backward, right wheels forward
  digitalWrite(L298N1_IN1, LOW);
  digitalWrite(L298N1_IN2, HIGH);
  digitalWrite(L298N1_IN3, LOW);
  digitalWrite(L298N1_IN4, HIGH);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);

  digitalWrite(L298N2_IN1, HIGH);
  digitalWrite(L298N2_IN2, LOW);
  digitalWrite(L298N2_IN3, HIGH);
  digitalWrite(L298N2_IN4, LOW);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);

  Serial.print("OK: Turning left ");
  Serial.print(angle);
  Serial.print(" degrees at speed ");
  Serial.println(speed);
  // Note: Python controls timing via STOP command, no delay here
}

void turnRight(float angle, int speed) {
  speed = constrain(speed, 0, 100);
  int pwm = map(speed, 0, 100, MIN_SPEED, MAX_SPEED);

  // Left wheels forward, right wheels backward
  digitalWrite(L298N1_IN1, HIGH);
  digitalWrite(L298N1_IN2, LOW);
  digitalWrite(L298N1_IN3, HIGH);
  digitalWrite(L298N1_IN4, LOW);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);

  digitalWrite(L298N2_IN1, LOW);
  digitalWrite(L298N2_IN2, HIGH);
  digitalWrite(L298N2_IN3, LOW);
  digitalWrite(L298N2_IN4, HIGH);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);

  Serial.print("OK: Turning right ");
  Serial.print(angle);
  Serial.print(" degrees at speed ");
  Serial.println(speed);
  // Note: Python controls timing via STOP command, no delay here
}

void stopAllMotors() {
  digitalWrite(L298N1_IN1, LOW);
  digitalWrite(L298N1_IN2, LOW);
  digitalWrite(L298N1_IN3, LOW);
  digitalWrite(L298N1_IN4, LOW);
  analogWrite(L298N1_ENA, 0);
  analogWrite(L298N1_ENB, 0);

  digitalWrite(L298N2_IN1, LOW);
  digitalWrite(L298N2_IN2, LOW);
  digitalWrite(L298N2_IN3, LOW);
  digitalWrite(L298N2_IN4, LOW);
  analogWrite(L298N2_ENA, 0);
  analogWrite(L298N2_ENB, 0);

  Serial.println("OK: Stopped");
}
