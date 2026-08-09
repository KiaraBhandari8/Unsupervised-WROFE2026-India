import time
import sys # For clean exit
import keyboard

from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep # Already imported, keeping for clarity if used elsewhere

import board
from adafruit_servokit import ServoKit
from adafruit_extended_bus import ExtendedI2C as I2C_Extended

# --- Motor Control Setup ---
# Define GPIO pins (BCM numbering)
# For Motor A
AIN1 = DigitalOutputDevice(24)  # Connected to TB6612FNG AIN1
AIN2 = DigitalOutputDevice(23)  # Connected to TB6612FNG AIN2
PWMA_ENABLE = PWMOutputDevice(12)  # Changed to PWMOutputDevice for speed control
STBY = DigitalOutputDevice(25)  # Connected to TB6612FNG STBY

MOTOR_SPEED = 0.8 # Default motor speed (can be adjusted)

def motor_forward(speed=MOTOR_SPEED):
    """
    Drives the motor forward at given speed (0.0 to 1.0)
    """
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA_ENABLE.value = speed

def motor_backward(speed=MOTOR_SPEED):
    """
    Drives the motor backward at given speed (0.0 to 1.0).
    """
    STBY.on()
    AIN1.off()
    AIN2.on()
    PWMA_ENABLE.value = speed

def motor_stop():
    """
    Stops the motor (coasting).
    """
    STBY.on()
    AIN1.off()
    AIN2.off()
    PWMA_ENABLE.value = 0

def motor_brake():
    """
    Brakes the motor (short brake).
    PWMA must be high for brake to be effective.
    """
    STBY.on()
    AIN1.on()
    AIN2.on()
    PWMA_ENABLE.value = 1

def motor_standby():
    """
    Puts the motor driver in standby mode to save power.
    """
    PWMA_ENABLE.value = 0
    STBY.off()

def robot_forward():
    print("Robot: Moving Forward")
    motor_forward(MOTOR_SPEED)

def robot_forward_speed(MOTOR_SPEED):
    print("Robot: Moving Forward")
    motor_forward(MOTOR_SPEED)

def robot_backward():
    print("Robot: Moving Backward")
    motor_backward(MOTOR_SPEED)

def robot_stop():
    print("Robot: Stopping")
    motor_stop()

def robot_left_90():
    adjust_servo_angle(90)
    motor_forward(MOTOR_SPEED)
    sleep(2.5)

def robot_right_90():
    adjust_servo_angle(50)
    motor_forward(MOTOR_SPEED)
    sleep(2.5)

# --- Servo Control Setup ---
i2c_bus = I2C_Extended(4) # Use the bus ID (e.g., 4) that you set in config.txt
kit = ServoKit(channels=16, i2c=i2c_bus, address=0x41)

SERVO_INDEX = 1
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_INCREMENT = 5 # Degrees to move per key press

# Set initial servo position and pulse width range
kit.servo[SERVO_INDEX].set_pulse_width_range(500, 2500)
current_servo_angle = 90 # Initial angle
kit.servo[SERVO_INDEX].angle = current_servo_angle
print(f"Servo {SERVO_INDEX} initialized to {current_servo_angle} degrees.")

def adjust_servo_angle(new_angle):
        kit.servo[SERVO_INDEX].angle = round(new_angle)
        print(f"Servo {SERVO_INDEX} moved to {round(new_angle)} degrees.")


if __name__ == "__main__":
    adjust_servo_angle(95)
    # for _ in range(5):
    #     robot_forward()
    #     sleep(1)
    #     # robot_backward()
    #     # sleep(1)
    # robot_stop()
    # robot_left_90()

# sudo /home/pi8/wrofe2025/env_test/bin/python /home/pi8/wrofe2025/robot_control_keys_1.py
# sudo /home/pi8/wrofe2025/env_test/bin/python /home/pi8/wrofe2025/rck_2.p