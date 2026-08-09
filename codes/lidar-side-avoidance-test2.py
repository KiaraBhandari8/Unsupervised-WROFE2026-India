"""
debug_side_avoidance.py (fixed)

Standalone debug harness for LIDAR side-wall-avoidance -- drives the robot
for real so you can watch/tune this behavior in isolation from the camera /
corner-state-machine / Flask parts of the full control loop.

=====================================================================
WHAT CHANGED FROM THE ORIGINAL VERSION, AND WHY
=====================================================================
The original had two bugs that combined to make avoidance dominate even in
open space with no real wall nearby:

  1. The MIN_POINTS noise filter only applied when min_dist <= PANIC_DISTANCE
     (150mm). Between PANIC and WARN (150-300mm) there was NO point-count
     check at all -- a single stray ray anywhere in that 150mm-wide band was
     enough to start a proportional correction. One noisy reflection at
     280mm looked identical to a real wall at 280mm.

  2. There was no persistence. A single frame's reading was enough to both
     enter AND exit avoidance mode every single loop -- nothing required the
     condition to hold before acting, so it could flicker in and out of
     "SIDE AVOID" mode continuously even when nothing was actually close.

Fix, in three parts:
  1. ONE trigger distance (SIDE_TRIGGER_DISTANCE_MM) instead of two zones --
     simpler to reason about, and removes the "unprotected middle band" bug.
  2. Point-count requirement applies uniformly: a side only counts as
     "seeing a close wall" if at least MIN_TRIGGER_POINTS rays are under the
     trigger distance, at ANY distance under that threshold -- not just
     once already inside a tighter panic radius.
  3. Frame-persistence debounce: a side must satisfy the point-count
     condition for TRIGGER_CONFIRM_FRAMES consecutive frames before
     avoidance actually engages, and once engaged, must see
     RELEASE_CONFIRM_FRAMES consecutive clear frames before releasing.
     This is the same debounce pattern used elsewhere in the main file
     (e.g. OBSTACLE_MISS_EXIT_FRAMES) -- applied symmetrically here on
     both entry and exit.

Result: in open space, nothing ever crosses the point-count + persistence
bar, so target_servo stays at SERVO_CENTER_ANGLE (straight) the entire
time -- exactly the "should not move unless a real panic distance occurs"
behavior you want to verify before this gets wired back into the main file.

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
DRIVE_SPEED = 155           # forward speed sent to ESP32 while testing (0-255)
LOOP_RATE_SEC = 0.05        # delay between loop iterations
RAW_DUMP = False            # True = print every raw (angle, distance) pair each loop (verbose)
DRY_RUN = False             # True = compute + print only, do NOT write to ESP32 serial

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400
ESP_PORT = "/dev/ttyAMA0"
ESP_BAUD = 115200
# =====================================================================


# --- side-avoidance tunables (single-threshold + persistence design) ---
SIDE_TRIGGER_DISTANCE_MM = 200      # a side is "close" once rays read under this
SIDE_STEER_MIN_MAGNITUDE = 10       # correction magnitude right at the trigger edge
SIDE_STEER_MAX_MAGNITUDE = 30       # correction magnitude when nearly touching (0mm)

MIN_TRIGGER_POINTS = 3              # rays under SIDE_TRIGGER_DISTANCE_MM required to count as "real"
                                     # (was only checked below PANIC before; now checked uniformly)
TRIGGER_CONFIRM_FRAMES = 3          # consecutive frames the point-count condition must hold to ENGAGE
RELEASE_CONFIRM_FRAMES = 4          # consecutive CLEAR frames required to RELEASE once engaged
                                     # (slightly higher than confirm -- bias toward safety, not flicker)

RIGHT_ZONE_ANGLES = range(50, 90)
LEFT_ZONE_ANGLES = range(-90, -50)

FRONT_SCAN_ANGLE_DEG = 15
FRONT_SAFETY_STOP_MM = 220.0        # basic front safety brake, no gyro/yaw involved

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


def compute_side_avoidance_magnitude(min_dist_mm):
    """Only called once a side is already ENGAGED (see debounce logic in main loop).
    Scales smoothly from SIDE_STEER_MIN_MAGNITUDE at the trigger edge up to
    SIDE_STEER_MAX_MAGNITUDE as min_dist_mm approaches 0."""
    if min_dist_mm is None:
        return SIDE_STEER_MIN_MAGNITUDE

    proximity_frac = min(1.0, max(0.0, (SIDE_TRIGGER_DISTANCE_MM - min_dist_mm) / SIDE_TRIGGER_DISTANCE_MM))
    return SIDE_STEER_MIN_MAGNITUDE + proximity_frac * (SIDE_STEER_MAX_MAGNITUDE - SIDE_STEER_MIN_MAGNITUDE)


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
    print(f"[CONFIG] SPEED={DRIVE_SPEED}  TRIGGER_DIST={SIDE_TRIGGER_DISTANCE_MM}mm  "
          f"MIN_POINTS={MIN_TRIGGER_POINTS}  CONFIRM_FRAMES={TRIGGER_CONFIRM_FRAMES}  "
          f"RELEASE_FRAMES={RELEASE_CONFIRM_FRAMES}  "
          f"MIN_MAG={SIDE_STEER_MIN_MAGNITUDE}  MAX_MAG={SIDE_STEER_MAX_MAGNITUDE}  "
          f"front_safety_stop={FRONT_SAFETY_STOP_MM}mm\n")

    frame = 0

    # --- debounce state, kept across loop iterations, one per side ---
    left_confirm_streak = 0
    right_confirm_streak = 0
    left_clear_streak = 0
    right_clear_streak = 0
    left_engaged = False
    right_engaged = False

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

            # --- point-count condition: how many rays are under the trigger distance, on EACH side ---
            left_under = [d for d in left_dists if d < SIDE_TRIGGER_DISTANCE_MM]
            right_under = [d for d in right_dists if d < SIDE_TRIGGER_DISTANCE_MM]
            left_raw_trigger = len(left_under) >= MIN_TRIGGER_POINTS
            right_raw_trigger = len(right_under) >= MIN_TRIGGER_POINTS

            # --- ENTRY debounce: require TRIGGER_CONFIRM_FRAMES consecutive raw triggers ---
            left_confirm_streak = left_confirm_streak + 1 if left_raw_trigger else 0
            right_confirm_streak = right_confirm_streak + 1 if right_raw_trigger else 0

            if not left_engaged and left_confirm_streak >= TRIGGER_CONFIRM_FRAMES:
                left_engaged = True
                left_clear_streak = 0
            if not right_engaged and right_confirm_streak >= TRIGGER_CONFIRM_FRAMES:
                right_engaged = True
                right_clear_streak = 0

            # --- EXIT debounce: once engaged, require RELEASE_CONFIRM_FRAMES consecutive clear frames ---
            if left_engaged:
                left_clear_streak = left_clear_streak + 1 if not left_raw_trigger else 0
                if left_clear_streak >= RELEASE_CONFIRM_FRAMES:
                    left_engaged = False
                    left_confirm_streak = 0
            if right_engaged:
                right_clear_streak = right_clear_streak + 1 if not right_raw_trigger else 0
                if right_clear_streak >= RELEASE_CONFIRM_FRAMES:
                    right_engaged = False
                    right_confirm_streak = 0

            left_min = min(left_dists) if left_dists else None
            right_min = min(right_dists) if right_dists else None

            left_offset = compute_side_avoidance_magnitude(left_min) if left_engaged else 0.0
            right_offset = compute_side_avoidance_magnitude(right_min) if right_engaged else 0.0

            # --- decide steering / speed ---
            if right_engaged or left_engaged:
                if right_offset >= left_offset:
                    target_servo = SERVO_CENTER_ANGLE - right_offset
                    mode = f"SIDE AVOID (right, {right_min:.0f}mm, n={len(right_under)})" if right_min is not None else "SIDE AVOID (right)"
                else:
                    target_servo = SERVO_CENTER_ANGLE + left_offset
                    mode = f"SIDE AVOID (left, {left_min:.0f}mm, n={len(left_under)})" if left_min is not None else "SIDE AVOID (left)"
            else:
                target_servo = SERVO_CENTER_ANGLE
                mode = "STRAIGHT"

            final_servo = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, target_servo))
            packet = send_esp_packet(esp_ser, final_servo, DRIVE_SPEED)

            l_str = f"{left_min:.0f}mm" if left_min is not None else "N/A"
            r_str = f"{right_min:.0f}mm" if right_min is not None else "N/A"

            # Only print every frame when something is actually happening (engaged, or building
            # toward engagement) -- stays quiet in genuinely open space instead of spamming.
            if left_engaged or right_engaged or left_confirm_streak > 0 or right_confirm_streak > 0:
                print(f"[{frame:05d}] {mode:32s} | Front:{front_dist:6.0f}mm | "
                      f"L:{l_str:>8s} under={len(left_under)} streak={left_confirm_streak}/{TRIGGER_CONFIRM_FRAMES} engaged={left_engaged} | "
                      f"R:{r_str:>8s} under={len(right_under)} streak={right_confirm_streak}/{TRIGGER_CONFIRM_FRAMES} engaged={right_engaged} | "
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