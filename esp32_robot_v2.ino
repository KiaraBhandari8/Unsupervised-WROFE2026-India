/*
 * ESP32 Robot Controller v2
 * Features:
 * - Receives commands from Pi via Serial
 * - Controls Servo and N20 Motors
 * - Reads MPU6050 Gyroscope
 * - Reads TF-Luna LIDAR (I2C)
 */

#include <Servo.h>
#include <Wire.h>

// ===== PIN CONFIGURATION =====
const int SERVO_PIN = 13;
const int MOTOR_IN1 = 26;
const int MOTOR_IN2 = 27;
const int MOTOR_PWM = 14;

// ===== I2C DEVICES =====
const int MPU_ADDR = 0x68;      // Gyroscope
const int TF_LUNA_ADDR = 0x10;   // LIDAR

// ===== VARIABLES =====
Servo steeringServo;
int currentServoAngle = 90;
int targetServoAngle = 90;
int motorSpeed = 0;

// Gyro data
int16_t gyroX, gyroY, gyroZ;
int16_t accX, accY, accZ;

// LIDAR data
int lidarDistance = 9999;

unsigned long lastLidarRead = 0;
unsigned long lastGyroRead = 0;

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Robot Controller v2 ===");
  
  // Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);
  
  // Motor
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  stopMotor();
  
  // I2C for Gyro and LIDAR
  Wire.begin(21, 22);  // SDA, SCL
  
  // Init MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Check TF-Luna LIDAR
  Wire.beginTransmission(TF_LUNA_ADDR);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("TF-Luna LIDAR detected!");
  } else {
    Serial.println("No LIDAR detected - using simulation mode");
  }
  
  Serial.println("ESP32 Ready!");
}

// ===== LOOP =====
void loop() {
  // Serial commands from Pi
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }
  
  // Smooth servo
  if (currentServoAngle != targetServoAngle) {
    if (currentServoAngle < targetServoAngle) currentServoAngle++;
    else currentServoAngle--;
    steeringServo.write(currentServoAngle);
  }
  
  // Read sensors periodically
  unsigned long now = millis();
  
  if (now - lastLidarRead > 50) {
    readLidar();
    lastLidarRead = now;
  }
  
  if (now - lastGyroRead > 20) {
    readGyro();
    lastGyroRead = now;
    printSensorData();
  }
  
  delay(5);
}

// ===== COMMANDS =====
void processCommand(String cmd) {
  Serial.println("CMD: " + cmd);
  
  if (cmd == "INIT") {
    steeringServo.write(90);
    targetServoAngle = 90;
    currentServoAngle = 90;
    stopMotor();
  }
  
  else if (cmd.startsWith("SERVO:")) {
    int angle = cmd.substring(7).toInt();
    targetServoAngle = constrain(angle, 0, 180);
  }
  
  else if (cmd.startsWith("FORWARD:")) {
    int speed = cmd.substring(8).toInt();
    motorSpeed = map(speed, 0, 100, 0, 255);
    forward(motorSpeed);
  }
  
  else if (cmd.startsWith("BACKWARD:")) {
    int speed = cmd.substring(10).toInt();
    motorSpeed = map(speed, 0, 100, 0, 255);
    backward(motorSpeed);
  }
  
  else if (cmd == "STOP") {
    stopMotor();
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

// ===== SENSORS =====
void readLidar() {
  // TF-Luna Read
  Wire.beginTransmission(TF_LUNA_ADDR);
  Wire.write(0x00);  // Register for distance
  Wire.endTransmission(false);
  Wire.requestFrom(TF_LUNA_ADDR, 2);
  
  if (Wire.available() >= 2) {
    byte high = Wire.read();
    byte low = Wire.read();
    lidarDistance = (high << 8) | low;
  }
}

void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void printSensorData() {
  // Format: GYRO:ax,ay,az,gx,gy,gz,LIDAR:dist
  Serial.print("GYRO:");
  Serial.print(accX); Serial.print(",");
  Serial.print(accY); Serial.print(",");
  Serial.print(accZ); Serial.print(",");
  Serial.print(gyroX); Serial.print(",");
  Serial.print(gyroY); Serial.print(",");
  Serial.print(gyroZ); Serial.print(",");
  Serial.print(lidarDistance);
  Serial.println();
}

/*
 * WIRING:
 * 
 * ESP32 Pin -> Component
 * ---------------------
 * GPIO 13 -> Servo Signal (Yellow/White)
 * GPIO 26 -> Motor Driver IN1
 * GPIO 27 -> Motor Driver IN2  
 * GPIO 14 -> Motor Driver PWM
 * GPIO 21 -> I2C SDA (Gyro + LIDAR)
 * GPIO 22 -> I2C SCL (Gyro + LIDAR)
 * 
 * External Power:
 * - Servo: 5-6V (from BEC/ESC)
 * - N20 Motor: 6V (from BEC)
 * - ESP32: 5V USB or 5-12V input
 * 
 * I2C Devices (daisy chained):
 * - MPU-6050: SDA->21, SCL->22, ADR->GND
 * - TF-Luna:  SDA->21, SCL->22, I2C addr 0x10
 */