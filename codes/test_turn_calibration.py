"""
Standalone test: turn calibration.

Turns the robot a target number of degrees (default 85) in one direction,
using the ESP32's gyro yaw telemetry as the stop condition (not a fixed timer).
Useful for checking that:
  - the steering servo actually reaches its commanded hard-turn angle
  - the resulting turn radius/motor combo produces a real ~85 degree yaw change
  - direction (left/right) matches what you expect

No ESP32 firmware changes needed -- this just drives the existing STR:/SPD:
protocol like the main control loop does.

Usage:
    python3 test_turn_calibration.py
    python3 test_turn_calibration.py --direction left --degrees 90 --speed 160
"""

import argparse
import signal
import sys
import time
import serial

PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

SERVO_CENTER_ANGLE = 95
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0

DEFAULT_SPEED = 80            # matches ROBOT_MANEUVER_SPEED in main script
DEFAULT_TARGET_DEGREES = 30.0
DEFAULT_SAFETY_TIMEOUT = 3.0   # hard cap in seconds, in case gyro data stalls
BRAKE_DELAY = 0.25

ser = None
current_yaw = 0.0


def send_packet(ser_port, steering, speed):
    if ser_port and ser_port.is_open:
        packet = f"STR:{steering},SPD:{speed}\n"
        ser_port.write(packet.encode("utf-8"))


def read_yaw_updates():
    """Drain any pending YAW: lines from the ESP32 and update current_yaw."""
    global current_yaw
    while ser.in_waiting > 0:
        try:
            raw_line = ser.readline().decode("utf-8", errors="ignore").strip()
            if raw_line.startswith("YAW:"):
                current_yaw = float(raw_line.split(":")[1])
        except Exception:
            pass


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

    parser = argparse.ArgumentParser(description="Test turning calibration (gyro-based)")
    parser.add_argument("--direction", choices=["left", "right"], default="right",
                         help="Turn direction (default: %(default)s)")
    parser.add_argument("--degrees", type=float, default=DEFAULT_TARGET_DEGREES,
                         help="Target yaw change in degrees (default: %(default)s)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                         help="Drive speed while pivoting, 0-255 (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_SAFETY_TIMEOUT,
                         help="Safety time cap in seconds (default: %(default)s)")
    parser.add_argument("--port", type=str, default=PI_TO_ESP_PORT,
                         help="Serial port to ESP32 (default: %(default)s)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, BAUD_RATE_ESP, timeout=0.05)
        print(f"[INFO] Connected to ESP32 on {args.port}")
    except Exception as e:
        print(f"[FATAL] Could not open serial port: {e}")
        sys.exit(1)

    time.sleep(0.5)     # let serial settle
    read_yaw_updates()  # drain any startup noise

    print("[TEST] Centering steering, ensuring stopped...")
    send_packet(ser, SERVO_CENTER_ANGLE, 0)
    time.sleep(BRAKE_DELAY)

    # Record baseline yaw locally on the Pi side. We deliberately do NOT rely
    # on the firmware's RST_YAW command -- it currently never triggers,
    # since the main Serial2 read loop drains all bytes before the RST_YAW
    # check block ever sees one. Tracking our own baseline sidesteps that.
    read_yaw_updates()
    baseline_yaw = current_yaw
    print(f"[TEST] Baseline yaw: {baseline_yaw:.2f}°")

    hard_servo_angle = SERVO_HARD_RIGHT if args.direction == "right" else SERVO_HARD_LEFT
    print(f"[TEST] Turning {args.direction.upper()} (servo={hard_servo_angle}) "
          f"until yaw delta >= {args.degrees}° or {args.timeout}s elapses...")

    start_time = time.monotonic()
    while True:
        read_yaw_updates()
        yaw_delta = abs(current_yaw - baseline_yaw)
        elapsed = time.monotonic() - start_time

        send_packet(ser, hard_servo_angle, args.speed)
        print(f"[TEST] Yaw delta: {yaw_delta:6.2f}\u00b0 / {args.degrees}\u00b0  | elapsed: {elapsed:4.2f}s", end="\r")

        if yaw_delta >= args.degrees:
            print(f"\n[TEST] Target reached. Yaw delta: {yaw_delta:.2f}°")
            break
        if elapsed >= args.timeout:
            print(f"\n[TEST] WARNING: Safety timeout hit before target reached "
                  f"(only {yaw_delta:.2f}° / {args.degrees}°). Check gyro / servo / wiring.")
            break

        time.sleep(0.01)

    print("[TEST] Braking...")
    send_packet(ser, SERVO_CENTER_ANGLE, 0)
    time.sleep(0.3)

    print("[TEST] Done. Closing serial.")
    ser.close()


if __name__ == "__main__":
    main()