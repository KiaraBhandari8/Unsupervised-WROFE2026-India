"""
debug_side_avoidance.py

Standalone debug harness for LIDAR side-wall-avoidance. Drives the robot for
real (sends steering/speed packets to the ESP32) so you can watch the
avoidance behavior live, isolated from the camera / corner-state-machine /
Flask parts of the full control loop.

No gyro/yaw dependency anywhere -- side zones are fixed sensor-frame angles.

ALL SETTINGS ARE PLAIN VARIABLES BELOW -- edit these directly and re-run,
no command-line flags needed.

Run:
    python3 debug_side_avoidance.py

Ctrl+C sends a stop packet and exits cleanly.
"""

import sys
import time
import serial

try:
    from lidar_steering_new import LidarScanner
except ImportError as e:
    print(f"[FATAL] Could not import LidarScanner from lidar_steering_new: {e}")
    sys.exit(1)


# =====================================================================
# EDIT THESE TO CHANGE BEHAVIOR -- no command-line flags, just change and re-run
# =====================================================================
DRIVE_SPEED = 155          # forward speed sent to ESP32 while testing (0-255)
MIN_POINTS = 1           # require at least this many rays under PANIC distance
                            # before trusting a min() reading as a real trigger.
                            # 1 = filter off (any single ray can trigger).
                            # Try 2 or 3 if you're seeing spurious spikes in open space.
LOOP_RATE_SEC = 0.05        # delay between loop iterations
RAW_DUMP = False            # True = print every raw (angle, distance) pair each loop (verbose)
DRY_RUN = False              # True = compute + print only, do NOT write to ESP32 serial

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400
ESP_PORT = "/dev/ttyAMA0"
ESP_BAUD = 115200
# =====================================================================


# --- side-avoidance tunables, mirrored from the main control file ---
LIDAR_SIDE_WARN_DISTANCE_MM = 300
LIDAR_SIDE_PANIC_DISTANCE_MM = 150
LIDAR_SIDE_STEER_MIN_MAGNITUDE = 6
LIDAR_SIDE_STEER_MAX_MAGNITUDE = 32

RIGHT_ZONE_ANGLES = range(50, 90)
LEFT_ZONE_ANGLES = range(-90, -50)

FRONT_SCAN_ANGLE_DEG = 15
FRONT_SAFETY_STOP_MM = 220.0   # basic front safety brake, no gyro/yaw involved

SERVO_CENTER_ANGLE = 100
SERVO_MIN_ANGLE = SERVO_CENTER_ANGLE - 20
SERVO_MAX_ANGLE = SERVO_CENTER_ANGLE + 20


def get_fixed_front_distance(scan_data):
    if not scan_data:
        return 2000.0
    pts = [scan_data[a] for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
           if a in scan_data and scan_data[a] > 0]
    return (sum(pts) / len(pts)) if pts else 2000.0


def get_side_zone_points(scan_data):
    """Fixed sensor-frame zones. No yaw, no rotation -- just the raw angles."""
    right_points = [(a, scan_data[a]) for a in RIGHT_ZONE_ANGLES if a in scan_data and scan_data[a] > 0]
    left_points = [(a, scan_data[a]) for a in LEFT_ZONE_ANGLES if a in scan_data and scan_data[a] > 0]
    return left_points, right_points


def compute_side_avoidance_offset(min_dist_mm):
    """Far away (>= WARN) -> no correction. Close (< WARN) -> proportional correction
    ramping to max at/inside PANIC distance."""
    if min_dist_mm is None or min_dist_mm >= LIDAR_SIDE_WARN_DISTANCE_MM:
        return 0.0

    span = max(LIDAR_SIDE_WARN_DISTANCE_MM - LIDAR_SIDE_PANIC_DISTANCE_MM, 1.0)
    proximity_frac = min(1.0, (LIDAR_SIDE_WARN_DISTANCE_MM - min_dist_mm) / span)
    proximity_frac = max(0.0, proximity_frac)

    magnitude = LIDAR_SIDE_STEER_MIN_MAGNITUDE + proximity_frac * (
        LIDAR_SIDE_STEER_MAX_MAGNITUDE - LIDAR_SIDE_STEER_MIN_MAGNITUDE
    )
    return magnitude


def count_points_under_panic(points):
    return sum(1 for (_, d) in points if d <= LIDAR_SIDE_PANIC_DISTANCE_MM)


def send_esp_packet(ser_port, steering, speed):
    packet = f"STR:{int(round(steering))},SPD:{int(speed)}\n"
    if DRY_RUN:
        return packet
    if ser_port and ser_port.is_open:
        try:
            ser_port.write(packet.encode('utf-8'))
        except Exception as e:
            print(f"[WARN] Serial write failed: {e}")
    return packet


def main():
    print(f"[SYSTEM] Connecting to LiDAR on {LIDAR_PORT} @ {LIDAR_BAUD}...")
    scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
    try:
        scanner.connect()
    except Exception as e:
        print(f"[FATAL] Could not connect to LiDAR: {e}")
        sys.exit(1)

    esp_ser = None
    if not DRY_RUN:
        print(f"[SYSTEM] Connecting to ESP32 on {ESP_PORT} @ {ESP_BAUD}...")
        try:
            esp_ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.05)
        except Exception as e:
            print(f"[FATAL] Could not open ESP32 serial port: {e}")
            scanner.disconnect()
            sys.exit(1)
    else:
        print("[SYSTEM] DRY_RUN=True: no serial writes will be sent, packets only printed.")

    print("[SYSTEM] Driving. Ctrl+C to stop.\n")
    print(f"[CONFIG] SPEED={DRIVE_SPEED}  MIN_POINTS={MIN_POINTS}  "
          f"WARN={LIDAR_SIDE_WARN_DISTANCE_MM}mm  PANIC={LIDAR_SIDE_PANIC_DISTANCE_MM}mm  "
          f"MIN_MAG={LIDAR_SIDE_STEER_MIN_MAGNITUDE}  MAX_MAG={LIDAR_SIDE_STEER_MAX_MAGNITUDE}  "
          f"front_safety_stop={FRONT_SAFETY_STOP_MM}mm\n")

    frame = 0
    try:
        while True:
            scan_data = scanner.get_scan_data()
            if not scan_data:
                time.sleep(LOOP_RATE_SEC)
                continue

            front_dist = get_fixed_front_distance(scan_data)

            # --- basic front safety brake, no gyro/yaw involved ---
            if front_dist < FRONT_SAFETY_STOP_MM:
                packet = send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                print(f"[{frame:05d}] [FRONT SAFETY STOP] Front:{front_dist:.0f}mm < {FRONT_SAFETY_STOP_MM:.0f}mm "
                      f"-> sent: {packet.strip()}")
                frame += 1
                time.sleep(LOOP_RATE_SEC)
                continue

            left_points, right_points = get_side_zone_points(scan_data)
            left_dists = [d for (_, d) in left_points]
            right_dists = [d for (_, d) in right_points]
            left_min = min(left_dists) if left_dists else None
            right_min = min(right_dists) if right_dists else None

            # --- optional noise filter: require MIN_POINTS rays under PANIC before trusting min() ---
            left_panic_count = count_points_under_panic(left_points)
            right_panic_count = count_points_under_panic(right_points)

            left_min_filtered = left_min
            right_min_filtered = right_min
            if left_min is not None and left_min <= LIDAR_SIDE_PANIC_DISTANCE_MM and left_panic_count < MIN_POINTS:
                left_min_filtered = None
            if right_min is not None and right_min <= LIDAR_SIDE_PANIC_DISTANCE_MM and right_panic_count < MIN_POINTS:
                right_min_filtered = None

            left_offset = compute_side_avoidance_offset(left_min_filtered)
            right_offset = compute_side_avoidance_offset(right_min_filtered)

            # --- decide steering / speed, same priority rule as the main file ---
            if right_offset > 0.0 or left_offset > 0.0:
                if right_offset >= left_offset:
                    target_servo = SERVO_CENTER_ANGLE - right_offset
                    mode = f"SIDE AVOID (right, {right_min:.0f}mm)" if right_min is not None else "SIDE AVOID (right)"
                else:
                    target_servo = SERVO_CENTER_ANGLE + left_offset
                    mode = f"SIDE AVOID (left, {left_min:.0f}mm)" if left_min is not None else "SIDE AVOID (left)"
            else:
                target_servo = SERVO_CENTER_ANGLE
                mode = "STRAIGHT"

            final_servo = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, target_servo))
            packet = send_esp_packet(esp_ser, final_servo, DRIVE_SPEED)

            l_str = f"{left_min:.0f}mm" if left_min is not None else "N/A"
            r_str = f"{right_min:.0f}mm" if right_min is not None else "N/A"
            flag_l = " [FILTERED-NOISE]" if (left_min is not None and left_min_filtered is None) else ""
            flag_r = " [FILTERED-NOISE]" if (right_min is not None and right_min_filtered is None) else ""

            print(f"[{frame:05d}] {mode:28s} | Front:{front_dist:6.0f}mm | "
                  f"L:{l_str:>8s} (n<PANIC:{left_panic_count}){flag_l} off:{left_offset:5.1f} | "
                  f"R:{r_str:>8s} (n<PANIC:{right_panic_count}){flag_r} off:{right_offset:5.1f} | "
                  f"sent: {packet.strip()}")

            if RAW_DUMP:
                print(f"          LEFT  raw: {sorted(left_points, key=lambda p: p[1])}")
                print(f"          RIGHT raw: {sorted(right_points, key=lambda p: p[1])}")

            frame += 1
            time.sleep(LOOP_RATE_SEC)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received, stopping...")
    finally:
        if esp_ser and esp_ser.is_open:
            try:
                for _ in range(3):
                    esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                    esp_ser.flush()
                    time.sleep(0.03)
                esp_ser.close()
                print("[CLEANUP] Stop packet sent, ESP32 serial closed.")
            except Exception as e:
                print(f"[CLEANUP ERROR] {e}")
        try:
            scanner.disconnect()
            print("[CLEANUP] LiDAR disconnected cleanly.")
        except Exception:
            pass


if __name__ == "__main__":
    main()