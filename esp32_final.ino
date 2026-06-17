#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_CHANNEL 0
#define MOTOR_PWMA 25
#define MOTOR_AIN1 26
#define MOTOR_AIN2 27
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  Wire.begin(21, 22);
  
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, 0);
  
  pwm.begin();
  pwm.setPWMFreq(60);
  
  int centerPulse = map(90, 0, 180, 123, 615);
  pwm.setPWM(SERVO_CHANNEL, 0, centerPulse);
  
  Serial.println("[ESP32 SYSTEM] Online. Servo test...");
  
  // Quick hardware test: sweep servo to verify it works
  for (int a = 60; a <= 120; a += 10) {
    int p = map(a, 0, 180, 123, 615);
    pwm.setPWM(SERVO_CHANNEL, 0, p);
    delay(100);
  }
  pwm.setPWM(SERVO_CHANNEL, 0, map(90, 0, 180, 123, 615));
  Serial.println("Servo test done. Waiting for commands...");
}

void loop() {
  // Read from whichever port has data (USB/Serial OR GPIO/Serial2)
  String command = "";
  if (Serial2.available() > 0) {
    command = Serial2.readStringUntil('\n');
  } else if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
  } else {
    return;  // nothing to read
  }
  
  command.trim();
  
  int strIdx = command.indexOf("STR:");
  int spdIdx = command.indexOf(",SPD:");
  
  if (strIdx != -1 && spdIdx != -1) {
    int steeringAngle = command.substring(strIdx + 4, spdIdx).toInt();
    int motorSpeed = command.substring(spdIdx + 5).toInt();
    
    steeringAngle = max(0, min(180, steeringAngle));
    int servoPulse = map(steeringAngle, 0, 180, 123, 615);
    pwm.setPWM(SERVO_CHANNEL, 0, servoPulse);
    
    Serial.print("CMD angle=");
    Serial.print(steeringAngle);
    Serial.print(" pulse=");
    Serial.print(servoPulse);
    Serial.print(" speed=");
    Serial.println(motorSpeed);
    
    motorSpeed = max(0, min(255, motorSpeed));
    if (motorSpeed == 0) {
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, LOW);
      analogWrite(MOTOR_PWMA, 0);
    } else {
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      analogWrite(MOTOR_PWMA, motorSpeed);
    }
  }
}
