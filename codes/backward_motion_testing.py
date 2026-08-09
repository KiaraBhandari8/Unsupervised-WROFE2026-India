"""
Standalone test: backward motion only.

Isolates the CORNER_ALIGN_BACKWARD behavior from the main control loop
so you can tune duration/speed without camera or LiDAR running.

Usage:
    python3 test_backward_motion.py
    python3 test_backward_motion.py --duration 1.5 --speed 150
"""

import argparse
import signal
import sys
import time
import serial

# --- Match these to your main script's constants ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200
SERVO_CENTER_ANGLE = 95
ROBOT_MANEUVER_SPEED = 150
CORNER_BACKWARD_DURATION = 1.5
CORNER_BRAKE_DELAY = 0.25

ser = None


def send_packet(ser_port, steering, speed):
    if ser_port and ser_port.is_open:
        packet = f"STR:{steering},SPD:{speed}\n"
        ser_port.write(packet.encode("utf-8"))
        print(f"[SENT] {packet.strip()}")


def emergency_stop(signum=None, frame=None):
    print("\n[EMERGENCY STOP] Halting and closing serial...")
    if ser and ser.is_open:
        for _ in range(3):
            send_packet(ser, SERVO_CENTER_ANGLE, 0)
            time.sleep(0.03)
        ser.close()
    sys.exit(0)


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGQUIT, emergency_stop)


def main():
    global ser

    parser = argparse.ArgumentParser(description="Test backward motion only")
    parser.add_argument("--duration", type=float, default=CORNER_BACKWARD_DURATION,
                         help="Seconds to reverse (default: %(default)s)")
    parser.add_argument("--speed", type=int, default=ROBOT_MANEUVER_SPEED,
                         help="Reverse speed magnitude, 0-255 (default: %(default)s)")
    parser.add_argument("--port", type=str, default=PI_TO_ESP_PORT,
                         help="Serial port to ESP32 (default: %(default)s)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, BAUD_RATE_ESP, timeout=0.05)
        print(f"[INFO] Connected to ESP32 on {args.port}")
    except Exception as e:
        print(f"[FATAL] Could not open serial port: {e}")
        sys.exit(1)

    time.sleep(0.5)  # let serial settle

    print("[TEST] Centering steering, ensuring stopped...")
    send_packet(ser, SERVO_CENTER_ANGLE, 0)
    time.sleep(CORNER_BRAKE_DELAY)

    print(f"[TEST] Reversing at speed -{args.speed} for {args.duration}s...")
    start_time = time.monotonic()
    while time.monotonic() - start_time < args.duration:
        send_packet(ser, SERVO_CENTER_ANGLE, -args.speed)
        time.sleep(0.05)  # ~20Hz command rate, matches main loop's 0.02-0.05s cadence

    print("[TEST] Braking...")
    send_packet(ser, SERVO_CENTER_ANGLE, 0)
    time.sleep(0.3)

    print("[TEST] Done. Closing serial.")
    ser.close()


if __name__ == "__main__":
    main()