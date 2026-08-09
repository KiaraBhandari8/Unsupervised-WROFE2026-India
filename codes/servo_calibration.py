#!/usr/bin/env python3
"""
Interactive Servo Calibration Tool

Controls ONLY the steering servo.
Motor always remains stopped.

Controls:
    a  : -1°
    d  : +1°
    q  : -5°
    e  : +5°
    c  : Return to center value
    l  : Hard left
    r  : Hard right
    s  : Show current angle
    x  : Exit

When you find the correct center and endpoints,
copy the printed values into your main robot code.

Example:
    python3 servo_calibration.py
"""

import serial
import time

PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

START_ANGLE = 97

MIN_SERVO = 0
MAX_SERVO = 180

SERVO_STEP = 20
BIG_STEP = 5

angle = START_ANGLE


def send_packet(ser, steering):
    packet = f"STR:{steering},SPD:0\n"
    ser.write(packet.encode("utf-8"))


def print_help():
    print("\n========== SERVO CALIBRATION ==========")
    print("a : -1°")
    print("d : +1°")
    print("q : -5°")
    print("e : +5°")
    print("c : Return to start angle")
    print("l : Full left (0°)")
    print("r : Full right (180°)")
    print("s : Show current angle")
    print("x : Exit")
    print("=======================================\n")


def main():
    global angle

    try:
        ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE, timeout=0.05)
    except Exception as e:
        print(f"Failed to open serial: {e}")
        return

    time.sleep(0.5)

    send_packet(ser, angle)

    print_help()
    print(f"Current servo angle: {angle}°")

    while True:

        cmd = input("> ").strip().lower()

        if cmd == "a":
            angle -= SERVO_STEP

        elif cmd == "d":
            angle += SERVO_STEP

        elif cmd == "q":
            angle -= BIG_STEP

        elif cmd == "e":
            angle += BIG_STEP

        elif cmd == "c":
            angle = START_ANGLE

        elif cmd == "l":
            angle = MIN_SERVO

        elif cmd == "r":
            angle = MAX_SERVO

        elif cmd == "s":
            print(f"Current angle = {angle}°")
            continue

        elif cmd == "x":
            break

        else:
            print("Unknown command.")
            continue

        angle = max(MIN_SERVO, min(MAX_SERVO, angle))

        send_packet(ser, angle)

        print(f"Servo -> {angle}°")

    print("\nStopping...")

    send_packet(ser, START_ANGLE)
    time.sleep(0.2)

    ser.close()

    print("\nFinal values:")
    print(f"SERVO_CENTER_ANGLE = {START_ANGLE}")
    print("SERVO_HARD_LEFT = ???")
    print("SERVO_HARD_RIGHT = ???")


if __name__ == "__main__":
    main()