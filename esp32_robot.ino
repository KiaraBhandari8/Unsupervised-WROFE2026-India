/*
 * ESP32 Robot Controller
 * Receives commands from Raspberry Pi via Serial
 * Controls: Servo, N20 Motors, reads Gyroscope
 */

#include <Servo.h>
#include <Wire.h>

// ===== PIN CONFIGURATION =====
const int SERVO_PIN = 13;        // Servo signal pin
const int MOTOR_IN1 = 26;         // Motor driver IN1
const int MOTOR_IN2 = 27;        // Motor driver IN2
const int MOTOR_PWM = 14;        // Motor PWM pin

// ===== I2C GYROSCOPE =====
const int MPU_ADDR = 0x68;       // I2C address for MPU-6050

// ===== VARIABLES =====
Servo steeringServo;
int currentServoAngle = 90;
int targetServoAngle = 90;
int motorSpeed = 0;

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Robot Controller Starting...");
  
  // Initialize Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);
  
  // Initialize Motor Pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Stop motor initially
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  
  // Initialize Gyroscope
  Wire.begin(21, 22);  // SDA, SCL
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to 0 (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.println("ESP32 Ready!");
  Serial.println("Waiting for commands from Pi...");
}

// ===== LOOP =====
void loop() {
  // Check for serial commands from Raspberry Pi
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Smooth servo movement
  if (currentServoAngle != targetServoAngle) {
    if (currentServoAngle < targetServoAngle) currentServoAngle++;
    else currentServoAngle--;
    steeringServo.write(currentServoAngle);
  }
  
  // Print gyro data every 100ms
  static unsigned long lastGyro = 0;
  if (millis() - lastGyro > 100) {
    printGyroData();
    lastGyro = millis();
  }
  
  delay(5);  // Small delay for stability
}

// ===== PROCESS COMMANDS =====
void processCommand(String cmd) {
  Serial.println("Received: " + cmd);
  
  if (cmd.startsWith("INIT")) {
    // Initialize/reset
    steeringServo.write(90);
    targetServoAngle = 90;
    currentServoAngle = 90;
    stopMotor();
    Serial.println("INIT: Reset complete");
  }
  
  else if (cmd.startsWith("SERVO:")) {
    // Set servo angle: SERVO:90
    int angle = cmd.substring(7).toInt();
    targetServoAngle = constrain(angle, 0, 180);
    Serial.println("SERVO set to: " + String(targetServoAngle));
  }
  
  else if (cmd.startsWith("FORWARD:")) {
    // Set forward speed: FORWARD:65 (percentage)
    int speedPercent = cmd.substring(8).toInt();
    motorSpeed = map(speedPercent, 0, 100, 0, 255);
    forward(motorSpeed);
    Serial.println("FORWARD speed: " + String(speedPercent) + "%");
  }
  
  else if (cmd.startsWith("BACKWARD:")) {
    // Set backward speed: BACKWARD:65
    int speedPercent = cmd.substring(10).toInt();
    motorSpeed = map(speedPercent, 0, 100, 0, 255);
    backward(motorSpeed);
    Serial.println("BACKWARD speed: " + String(speedPercent) + "%");
  }
  
  else if (cmd.startsWith("STOP")) {
    stopMotor();
    Serial.println("MOTOR: Stopped");
  }
  
  else if (cmd.startsWith("STATUS")) {
    printStatus();
  }
  
  else {
    Serial.println("Unknown command: " + cmd);
  }
}

// ===== MOTOR CONTROL =====
void forward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, speed);
}

void backward(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_PWM, speed);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  motorSpeed = 0;
}

// ===== GYROSCOPE DATA =====
void printGyroData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  // Read accelerometer values
  int16_t accX = Wire.read() << 8 | Wire.read();
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  
  // Read gyroscope values
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  // Print in format: GYRO:ax,ay,az,gx,gy,gz
  Serial.print("GYRO:");
  Serial.print(accX); Serial.print(",");
  Serial.print(accY); Serial.print(",");
  Serial.print(accZ); Serial.print(",");
  Serial.print(gyroX); Serial.print(",");
  Serial.print(gyroY); Serial.print(",");
  Serial.println(gyroZ);
}

void printStatus() {
  Serial.print("STATUS: Servo=");
  Serial.print(currentServoAngle);
  Serial.print(" Motor=");
  Serial.print(motorSpeed);
  Serial.print(" GyroOK");
  Serial.println();
}

/*
 * Arduino IDE Settings:
 * - Board: ESP32 Dev Module
 * - Upload Speed: 115200
 * - Flash Frequency: 80MHz
 * 
 * Libraries needed:
 * - Servo (built-in)
 * - Wire (built-in)
 */