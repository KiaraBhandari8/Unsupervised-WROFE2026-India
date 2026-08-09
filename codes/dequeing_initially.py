#!/usr/bin/env python3
"""
Standalone parking-lot exit maneuver for WRO 2026 robot.
Run this by itself, independent of the main robot_control_loop.py.

Sequence:
  1. Read LiDAR left/right averages to decide round direction.
  2. Pivot ~90deg in place (CW if left<right, CCW if right<left).
  3. Drive straight until front LiDAR distance <= ~500mm (rack center).
  4. Pivot ~90deg the OPPOSITE direction to face the track.
  5. Stop, hand control back (exits cleanly / can be chained into main script).

Requires the same hardware config as the main script: ESP32 over serial
(steering+speed commands, YAW: telemetry), and the 2D LiDAR via
lidar_steering_new.LidarScanner.
"""

import sys
import time
import threading
import signal
import serial

try:
    from lidar_steering_new import LidarScanner
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to import LidarScanner: {e}")
    sys.exit(1)

# --- SERIAL / HARDWARE CONFIG ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400

# --- SERVO / SPEED CONSTANTS (match main script) ---
SERVO_CENTER_ANGLE = 100
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0

# --- PARKING EXIT TUNABLES ---
PARKING_EXIT_TARGET_DEGREES = 80.0
PARKING_EXIT_PIVOT_SPEED = 100
PARKING_EXIT_PIVOT_SAFETY_TIMEOUT = 2.5
PARKING_EXIT_STRAIGHT_TARGET_MM = 300.0
PARKING_EXIT_STRAIGHT_SPEED = 150
PARKING_EXIT_STRAIGHT_SAFETY_TIMEOUT = 4.0
CORNER_BRAKE_DELAY = 0.25

# --- FRONT-CONE CONFIG (for get_compensated_front_distance) ---
FRONT_SCAN_ANGLE_DEG = 15

# --- GLOBAL STATE ---
global_shutdown_event = threading.Event()
esp_ser = None
lidar_scanner = None
current_yaw = 0.0

latest_lidar_data = {}
lidar_data_lock = threading.Lock()


def send_esp_packet(ser_port, steering, speed):
    if ser_port and ser_port.is_open and not global_shutdown_event.is_set():
        try:
            packet = f"STR:{steering},SPD:{speed}\n"
            ser_port.write(packet.encode('utf-8'))
        except Exception:
            pass


def emergency_shutdown_handler(signum, frame):
    print("\n[EMERGENCY BRAKE] Shutdown signal captured! Halting hardware...")
    global_shutdown_event.set()
    global esp_ser, lidar_scanner
    if esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
        except Exception as e:
            print(f"[CLEANUP ERROR] {e}")
    if lidar_scanner:
        try:
            lidar_scanner.disconnect()
        except Exception as e:
            print(f"[CLEANUP ERROR] {e}")
    print("[SUCCESS] Systems isolated. Exiting.\n")
    sys.exit(0)


signal.signal(signal.SIGINT, emergency_shutdown_handler)
signal.signal(signal.SIGQUIT, emergency_shutdown_handler)


def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data
    print("[SYSTEM] LiDAR background thread active.")
    try:
        while not global_shutdown_event.is_set():
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data.copy()
            time.sleep(0.01)
    except Exception as e:
        if not global_shutdown_event.is_set():
            print(f"[CRITICAL] LiDAR thread collapsed: {e}")


def yaw_reader_thread_func():
    global current_yaw
    while not global_shutdown_event.is_set():
        try:
            while esp_ser.in_waiting > 0:
                raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_line.startswith("YAW:"):
                    current_yaw = float(raw_line.split(":")[1])
        except Exception:
            pass
        time.sleep(0.01)


def get_compensated_front_distance(scan_data, yaw):
    import numpy as np
    if not scan_data:
        return 2000.0
    yaw_offset = int(round(yaw))
    dynamic_angles = range(-FRONT_SCAN_ANGLE_DEG + yaw_offset, FRONT_SCAN_ANGLE_DEG + yaw_offset + 1)
    compensated_points = []
    yaw_rad = np.radians(yaw)
    for a in dynamic_angles:
        if a in scan_data and scan_data[a] > 0:
            compensated_points.append(scan_data[a] * np.cos(yaw_rad))
    if not compensated_points:
        return 2000.0
    return sum(compensated_points) / len(compensated_points)


def get_lr_averages(scan_data):
    left_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
    right_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
    avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
    avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
    return avg_left, avg_right


def reset_yaw():
    global current_yaw
    esp_ser.write(b"RST_YAW\n")
    esp_ser.flush()
    time.sleep(0.1)
    current_yaw = 0.0


def run_pivot(direction, target_degrees):
    """direction: 'CW' or 'CCW'. Blocks until yaw target reached or safety timeout."""
    baseline_yaw = current_yaw  # should be 0 right after reset_yaw()
    servo = SERVO_HARD_RIGHT if direction == "CW" else SERVO_HARD_LEFT
    start_time = time.monotonic()

    print(f"[PARKING EXIT] Starting {direction} pivot, target {target_degrees}deg")
    send_esp_packet(esp_ser, servo, PARKING_EXIT_PIVOT_SPEED)

    while not global_shutdown_event.is_set():
        yaw_delta = current_yaw - baseline_yaw
        elapsed = time.monotonic() - start_time

        if direction == "CW":
            pivot_complete = yaw_delta <= -target_degrees
        else:
            pivot_complete = yaw_delta >= target_degrees

        timed_out = elapsed >= PARKING_EXIT_PIVOT_SAFETY_TIMEOUT

        if pivot_complete or timed_out:
            if timed_out and not pivot_complete:
                print(f"[PARKING EXIT] WARNING: {direction} pivot safety timeout hit "
                      f"(only {yaw_delta:+.1f}deg / target {target_degrees}deg). Check gyro.")
            else:
                print(f"[PARKING EXIT] {direction} pivot complete ({yaw_delta:+.1f}deg).")
            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
            time.sleep(CORNER_BRAKE_DELAY)
            return

        time.sleep(0.02)


def run_straight_to_center():
    """Drives straight until front LiDAR distance <= target, or safety timeout."""
    start_time = time.monotonic()
    print(f"[PARKING EXIT] Driving straight to rack center (~{PARKING_EXIT_STRAIGHT_TARGET_MM:.0f}mm front target)")
    send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, PARKING_EXIT_STRAIGHT_SPEED)

    while not global_shutdown_event.is_set():
        with lidar_data_lock:
            scan_data = latest_lidar_data.copy()
        front_dist = get_compensated_front_distance(scan_data, current_yaw)
        elapsed = time.monotonic() - start_time

        reached = front_dist <= PARKING_EXIT_STRAIGHT_TARGET_MM
        timed_out = elapsed >= PARKING_EXIT_STRAIGHT_SAFETY_TIMEOUT

        if reached or timed_out:
            if timed_out and not reached:
                print(f"[PARKING EXIT] WARNING: Straight-phase safety timeout hit "
                      f"(front={front_dist:.1f}mm, target={PARKING_EXIT_STRAIGHT_TARGET_MM:.0f}mm).")
            else:
                print(f"[PARKING EXIT] Reached rack center (front={front_dist:.1f}mm).")
            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
            time.sleep(CORNER_BRAKE_DELAY)
            return

        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, PARKING_EXIT_STRAIGHT_SPEED)
        time.sleep(0.02)


def main():
    global esp_ser, lidar_scanner

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] Serial connection established with ESP32.")
    except Exception as e:
        print(f"[FATAL] Serial bridge init failed on {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    try:
        lidar_scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        lidar_scanner.connect()
        print("[INFO] LiDAR connected.")
    except Exception as e:
        print(f"[FATAL] LiDAR init failed: {e}")
        emergency_shutdown_handler(None, None)
        return

    lidar_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
    lidar_thread.daemon = True
    lidar_thread.start()

    yaw_thread = threading.Thread(target=yaw_reader_thread_func)
    yaw_thread.daemon = True
    yaw_thread.start()

    time.sleep(0.5)  # let sensors populate

    try:
        # --- 1. DETECT DIRECTION ---
        with lidar_data_lock:
            scan_data = latest_lidar_data.copy()
        avg_left, avg_right = get_lr_averages(scan_data)
        print(f"[PARKING EXIT] [DETECT] Left: {avg_left:.1f}mm | Right: {avg_right:.1f}mm")

        if avg_left < avg_right:
            first_direction = "CW"
            print("[PARKING EXIT] Left < Right -> CLOCKWISE round. First pivot: CW.")
        else:
            first_direction = "CCW"
            print("[PARKING EXIT] Right < Left -> COUNTER-CLOCKWISE round. First pivot: CCW.")

        second_direction = "CCW" if first_direction == "CW" else "CW"

        # --- 2. PIVOT 1 ---
        reset_yaw()
        run_pivot(first_direction, PARKING_EXIT_TARGET_DEGREES)

        # --- 3. STRAIGHT TO RACK CENTER ---
        reset_yaw()
        run_straight_to_center()

        # --- 4. PIVOT 2 (opposite direction) ---
        reset_yaw()
        run_pivot(second_direction, PARKING_EXIT_TARGET_DEGREES)

        print("[PARKING EXIT] Maneuver complete. Robot should now be facing the track.")
        reset_yaw()

    except Exception as e:
        print(f"[SYSTEM FAILURE] {e}")
    finally:
        emergency_shutdown_handler(None, None)


if __name__ == '__main__':
    print("--- Parking Lot Exit Maneuver (Standalone) ---")
    main()