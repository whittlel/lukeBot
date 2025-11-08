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
 *   ENA -> Pin 12 (PWM), ENB -> Pin 13 (PWM)
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
const int L298N2_ENB = 13; // PWM

// Motor control constants
const int MIN_SPEED = 30;  // Minimum speed to overcome friction
const int MAX_SPEED = 255; // Maximum PWM value

// Serial communication
const int BAUD_RATE = 115200;
String inputString = "";         // String to hold incoming data
boolean stringComplete = false;  // Whether the string is complete

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  inputString.reserve(200);  // Reserve 200 bytes for input string
  
  // Initialize motor pins as outputs
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
  
  // Stop all motors initially
  stopAllMotors();
  
  // Send ready signal
  Serial.println("READY");
}

void loop() {
  // Check for serial input
  if (stringComplete) {
    // Parse and execute command
    parseCommand(inputString);
    
    // Clear the string
    inputString = "";
    stringComplete = false;
  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseCommand(String command) {
  command.trim();  // Remove whitespace
  
  if (command.length() == 0) {
    return;
  }
  
  // Parse command: COMMAND,PARAMETER
  int commaIndex = command.indexOf(',');
  String cmd = command.substring(0, commaIndex);
  cmd.toUpperCase();
  
  if (cmd == "MOVE_FORWARD") {
    if (commaIndex > 0) {
      int speed = command.substring(commaIndex + 1).toInt();
      speed = constrain(speed, 0, 100);
      moveForward(speed);
    } else {
      moveForward(50);  // Default speed
    }
  }
  else if (cmd == "MOVE_BACKWARD") {
    if (commaIndex > 0) {
      int speed = command.substring(commaIndex + 1).toInt();
      speed = constrain(speed, 0, 100);
      moveBackward(speed);
    } else {
      moveBackward(50);  // Default speed
    }
  }
  else if (cmd == "TURN_LEFT") {
    if (commaIndex > 0) {
      float angle = command.substring(commaIndex + 1).toFloat();
      turnLeft(angle);
    } else {
      turnLeft(90.0);  // Default angle
    }
  }
  else if (cmd == "TURN_RIGHT") {
    if (commaIndex > 0) {
      float angle = command.substring(commaIndex + 1).toFloat();
      turnRight(angle);
    } else {
      turnRight(90.0);  // Default angle
    }
  }
  else if (cmd == "STOP") {
    stopAllMotors();
  }
  else {
    // Unknown command
    Serial.print("ERROR: Unknown command: ");
    Serial.println(cmd);
  }
}

// Motor control functions

void moveForward(int speed) {
  // Convert speed (0-100) to PWM (MIN_SPEED-255)
  int pwm = map(speed, 0, 100, MIN_SPEED, MAX_SPEED);
  
  // All wheels forward
  // L298N #1: FL (motors on ENA) and RL (motors on ENB)
  digitalWrite(L298N1_IN1, HIGH);  // FL forward
  digitalWrite(L298N1_IN2, LOW);
  digitalWrite(L298N1_IN3, HIGH);   // RL forward
  digitalWrite(L298N1_IN4, LOW);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);
  
  // L298N #2: FR (motors on ENA) and RR (motors on ENB)
  digitalWrite(L298N2_IN1, HIGH);   // FR forward
  digitalWrite(L298N2_IN2, LOW);
  digitalWrite(L298N2_IN3, HIGH);  // RR forward
  digitalWrite(L298N2_IN4, LOW);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);
  
  Serial.println("OK: Moving forward");
}

void moveBackward(int speed) {
  // Convert speed (0-100) to PWM (MIN_SPEED-255)
  int pwm = map(speed, 0, 100, MIN_SPEED, MAX_SPEED);
  
  // All wheels backward
  // L298N #1: FL and RL
  digitalWrite(L298N1_IN1, LOW);   // FL backward
  digitalWrite(L298N1_IN2, HIGH);
  digitalWrite(L298N1_IN3, LOW);   // RL backward
  digitalWrite(L298N1_IN4, HIGH);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);
  
  // L298N #2: FR and RR
  digitalWrite(L298N2_IN1, LOW);   // FR backward
  digitalWrite(L298N2_IN2, HIGH);
  digitalWrite(L298N2_IN3, LOW);   // RR backward
  digitalWrite(L298N2_IN4, HIGH);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);
  
  Serial.println("OK: Moving backward");
}

void turnLeft(float angle) {
  // TODO: Implement angle-based turning with odometry
  // For now, use simple differential drive turning
  int pwm = map(50, 0, 100, MIN_SPEED, MAX_SPEED);  // Default speed
  
  // Left wheels backward, right wheels forward (rotate in place)
  // L298N #1: FL and RL backward
  digitalWrite(L298N1_IN1, LOW);
  digitalWrite(L298N1_IN2, HIGH);
  digitalWrite(L298N1_IN3, LOW);
  digitalWrite(L298N1_IN4, HIGH);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);
  
  // L298N #2: FR and RR forward
  digitalWrite(L298N2_IN1, HIGH);
  digitalWrite(L298N2_IN2, LOW);
  digitalWrite(L298N2_IN3, HIGH);
  digitalWrite(L298N2_IN4, LOW);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);
  
  Serial.print("OK: Turning left ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void turnRight(float angle) {
  // TODO: Implement angle-based turning with odometry
  // For now, use simple differential drive turning
  int pwm = map(50, 0, 100, MIN_SPEED, MAX_SPEED);  // Default speed
  
  // Left wheels forward, right wheels backward (rotate in place)
  // L298N #1: FL and RL forward
  digitalWrite(L298N1_IN1, HIGH);
  digitalWrite(L298N1_IN2, LOW);
  digitalWrite(L298N1_IN3, HIGH);
  digitalWrite(L298N1_IN4, LOW);
  analogWrite(L298N1_ENA, pwm);
  analogWrite(L298N1_ENB, pwm);
  
  // L298N #2: FR and RR backward
  digitalWrite(L298N2_IN1, LOW);
  digitalWrite(L298N2_IN2, HIGH);
  digitalWrite(L298N2_IN3, LOW);
  digitalWrite(L298N2_IN4, HIGH);
  analogWrite(L298N2_ENA, pwm);
  analogWrite(L298N2_ENB, pwm);
  
  Serial.print("OK: Turning right ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void stopAllMotors() {
  // Stop all motors
  // L298N #1
  digitalWrite(L298N1_IN1, LOW);
  digitalWrite(L298N1_IN2, LOW);
  digitalWrite(L298N1_IN3, LOW);
  digitalWrite(L298N1_IN4, LOW);
  analogWrite(L298N1_ENA, 0);
  analogWrite(L298N1_ENB, 0);
  
  // L298N #2
  digitalWrite(L298N2_IN1, LOW);
  digitalWrite(L298N2_IN2, LOW);
  digitalWrite(L298N2_IN3, LOW);
  digitalWrite(L298N2_IN4, LOW);
  analogWrite(L298N2_ENA, 0);
  analogWrite(L298N2_ENB, 0);
  
  Serial.println("OK: Stopped");
}

