"""
corner_and_wallfollow_control.py
==================================
Refactored, procedural cornering and wall-following module.
Vision-based (orange/blue line) corner-signature detection has been
removed. The LiDAR-only corner detection trigger (e.g. your
split-and-merge / RDP corner-segment pipeline) should call
`execute_cornering()` directly once it flags a corner.

UPDATE (this revision):
  - _execute_backward_maneuver() now actively steers to hold
    BACKWARD_WALL_TARGET_MM off the tracked wall while reversing,
    using the same get_wall_parallel_error() blend (alignment +
    distance-hold) from lidar_steering_new.py that _align_to_wall()
    already uses. Pass target_distance_mm=None to that call if you
    ever want pure-alignment (no distance-hold) reversing instead.
  - The backward maneuver's primary stop condition is now the rear
    HC-SR04 ultrasonic sensor crossing ULTRASONIC_STOP_DISTANCE_MM,
    not a fixed timer. CORNER_BACKWARD_DURATION is kept as a safety
    ceiling in case the ultrasonic sensor never returns a reading.
"""

import sys
import time
import threading
import signal
import numpy as np

try:
    import serial
except ImportError:
    serial = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

from lidar_steering_new_parallel import (
    LidarScanner,
    PIDController,
    get_wall_parallel_error,
    get_wall_parallel_sector_stats,
    PARALLEL_TOLERANCE_MM,
)

# ============================================================
# RUNTIME FLAGS & HARDWARE CONFIG
# ============================================================
MOTOR_LIVE = True
CLOCKWISE_WALL_FOLLOWING = True

PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400

# --- HC-SR04 ultrasonic (rear-facing, used to gate the backward maneuver) ---
ULTRASONIC_TRIG_PIN = 23   # BCM GPIO23 / physical pin 16
ULTRASONIC_ECHO_PIN = 24   # BCM GPIO24 / physical pin 18 (via voltage divider)
ULTRASONIC_ECHO_TIMEOUT_SEC = 0.04  # 40ms, matches the tested standalone script
ULTRASONIC_STOP_DISTANCE_MM = 200.0

# ============================================================
# CONTROL CONSTANTS
# ============================================================
SERVO_CENTER_ANGLE = 95
SPEED = 255
ROBOT_CRUISE_SPEED = SPEED
ROBOT_MANEUVER_SPEED = SPEED
CORNER_PIVOT_SPEED = SPEED
CORNER_BRAKE_DELAY = 0.0
CORNER_PIVOT_SAFETY_TIMEOUT = 2.5
CORNER_BACKWARD_DURATION = 4  # now a safety-timeout ceiling, not the primary stop trigger
CORNER_DETECTION_COOLDOWN_SEC = 10
FRONT_SCAN_ANGLE_DEG = 15
WALL_LOSS_THRESHOLD_MM = 400.0
WALL_FOLLOW_TARGET_MM = 600
SERVO_CLAMP = 20

LANE_REFERENCE_ARC_THRESHOLD_MM = 500.0
CORNER_APPROACH_TRIGGER_MM = 350.0

CORNER_ARC_STEER_OFFSET = 60
CORNER_ARC_PIVOT_SPEED = SPEED
CORNER_ARC_PIVOT_SAFETY_TIMEOUT = 2.0

TURN_TARGET_RIGHT_DEGREES = 70.0
TURN_TARGET_LEFT_DEGREES = 70.0
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0
WALL_ALIGN_CREEP_SPEED = SPEED
WALL_ALIGN_SAFETY_TIMEOUT = 3.0
WALL_ALIGN_NO_WALL_TIMEOUT = 1.0

# Backward-reverse wall straightening (Lane 1 post-pivot).
# target_distance_mm passed to get_wall_parallel_error() -- set this to None
# at the call site inside _execute_backward_maneuver() if you want pure
# front/rear alignment while reversing, with no distance-hold term.
BACKWARD_WALL_TARGET_MM = 500.0

LIDAR_STALE_TIMEOUT_SEC = 0.5

STATE_LOG_EVERY_N_FRAMES = 5  # throttle per-loop debug prints inside corner sub-states

# ============================================================
# GLOBALS & LOCKS
# ============================================================
global_shutdown_event = threading.Event()
esp_ser = None
lidar_scanner = None

latest_lidar_data = {}
lidar_data_lock = threading.Lock()
last_lidar_update_time = 0.0

current_yaw = 0.0

latest_status_snapshot = {
    "state": "STARTING", "front_mm": None, "left_mm": None, "right_mm": None,
    "cooldown_remaining": 0.0, "motor_live": MOTOR_LIVE,
}
status_lock = threading.Lock()

wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)
alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)
backward_align_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)

_ultrasonic_ready = False

# ============================================================
# HELPER FUNCTIONS
# ============================================================
def update_status_snapshot(**kwargs):
    with status_lock:
        latest_status_snapshot.update(kwargs)

def send_esp_packet(steering, speed, tag=""):
    packet = f"STR:{steering},SPD:{speed}\n"
    if not MOTOR_LIVE:
        return
    if esp_ser and esp_ser.is_open and not global_shutdown_event.is_set():
        try:
            esp_ser.write(packet.encode('utf-8'))
        except Exception as e:
            print(f"[SERIAL WRITE ERROR] {e}")


def emergency_shutdown_handler(signum, frame):
    global_shutdown_event.set()
    if MOTOR_LIVE and esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
        except Exception:
            pass
    if lidar_scanner:
        try:
            lidar_scanner.disconnect()
        except Exception:
            pass
    if GPIO is not None and _ultrasonic_ready:
        try:
            GPIO.cleanup()
        except Exception:
            pass
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_shutdown_handler)
signal.signal(signal.SIGQUIT, emergency_shutdown_handler)

# ============================================================
# ULTRASONIC (HC-SR04) HANDLERS
# ============================================================
def init_ultrasonic():
    """Sets up the rear-facing HC-SR04 used to gate the backward maneuver.
    Safe to call even when RPi.GPIO isn't importable (e.g. dev machine) --
    get_ultrasonic_distance_mm() will just return None in that case."""
    global _ultrasonic_ready
    if GPIO is None:
        print("[ULTRASONIC] RPi.GPIO not available -- backward maneuver will "
              "rely on the timed safety fallback only.")
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ULTRASONIC_TRIG_PIN, GPIO.OUT)
    GPIO.setup(ULTRASONIC_ECHO_PIN, GPIO.IN)
    GPIO.output(ULTRASONIC_TRIG_PIN, False)
    _ultrasonic_ready = True
    print(f"[ULTRASONIC] Ready on TRIG=GPIO{ULTRASONIC_TRIG_PIN} "
          f"ECHO=GPIO{ULTRASONIC_ECHO_PIN}.")


def get_ultrasonic_distance_mm():
    """Single-shot HC-SR04 read, same pulse timing as the tested standalone
    script. Returns distance in mm (converted from the sensor's native cm)
    to match the LiDAR units used everywhere else in this module, or None
    on timeout / missing GPIO."""
    if GPIO is None or not _ultrasonic_ready:
        return None

    GPIO.output(ULTRASONIC_TRIG_PIN, True)
    time.sleep(0.00001)  # 10us trigger pulse
    GPIO.output(ULTRASONIC_TRIG_PIN, False)

    pulse_start = time.time()
    pulse_end = time.time()

    timeout = time.time() + ULTRASONIC_ECHO_TIMEOUT_SEC
    while GPIO.input(ULTRASONIC_ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    timeout = time.time() + ULTRASONIC_ECHO_TIMEOUT_SEC
    while GPIO.input(ULTRASONIC_ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    pulse_duration = pulse_end - pulse_start
    distance_cm = (pulse_duration * 34300) / 2  # speed of sound, round trip
    return distance_cm * 10.0

# ============================================================
# HARDWARE & THREAD HANDLERS
# ============================================================
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, last_lidar_update_time
    consecutive_failures = 0
    while not global_shutdown_event.is_set():
        try:
            data = scanner_instance.get_scan_data()
        except Exception:
            data = None
        if data:
            with lidar_data_lock:
                latest_lidar_data = data.copy()
            last_lidar_update_time = time.monotonic()
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if consecutive_failures >= 15:
                try:
                    scanner_instance.disconnect()
                except Exception:
                    pass
                time.sleep(0.5)
                try:
                    scanner_instance.connect()
                    consecutive_failures = 0
                except Exception:
                    time.sleep(1.0)
        time.sleep(0.01)

# ============================================================
# UPDATED HARDWARE & DATA FETCHING HELPERS
# ============================================================
def get_scan_and_front_dist(scanner_instance=None):
    """
    Fetches fresh scan data from the live LiDAR scanner instance
    if available, updates local cache, and calculates average front distance.
    """
    global latest_lidar_data

    # 1. Pull active data directly from live LiDAR instance if available
    if scanner_instance is not None:
        try:
            live_data = scanner_instance.get_scan_data()
            if live_data:
                with lidar_data_lock:
                    latest_lidar_data = live_data.copy()
        except Exception:
            pass

    # 2. Read from local cache
    with lidar_data_lock:
        scan_data = latest_lidar_data.copy()

    if not scan_data:
        return scan_data, 2000.0

    pts = [scan_data[a] for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
           if a in scan_data and scan_data[a] > 0]
    front_dist = sum(pts) / len(pts) if pts else 2000.0
    return scan_data, front_dist


def update_serial_yaw():
    """Reads serial pipeline from ESP32 to update live IMU orientation."""
    global current_yaw, esp_ser
    if esp_ser and esp_ser.is_open:
        while esp_ser.in_waiting > 0:
            try:
                raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_line.startswith("YAW:"):
                    current_yaw = float(raw_line.split(":")[1])
            except Exception:
                pass

def send_reset_yaw():
    """Flushes serial input buffer and forces Python state reset."""
    global esp_ser, current_yaw
    if not MOTOR_LIVE:
        return
    if esp_ser and esp_ser.is_open:
        try:
            esp_ser.reset_input_buffer()  # Flush old queued packets
            esp_ser.write(b"RST_YAW\n")
            esp_ser.flush()
            current_yaw = 0.0
        except Exception as e:
            print(f"[SERIAL ERROR] Failed to reset Yaw: {e}")

# ============================================================
# CORNER EXECUTION SUB-FUNCTIONS
# ============================================================
def _align_to_wall(speed, direction_multiplier=1.0, scanner_instance=None, tag="ALIGN"):
    """Reusable wall alignment routine with live LiDAR updates and serial sync.

    `tag` is only used to label debug prints (e.g. "PRE-ALIGN", "POST-PIVOT ALIGN",
    "WALL ALIGN") so the same function's logs are distinguishable across the three
    call sites in execute_cornering(), matching the debug-harness log format.
    """
    start_time = time.monotonic()
    no_wall_start_time = 0.0
    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
    alignment_pid.reset()
    frame_counter = 0

    while not global_shutdown_event.is_set():
        frame_counter += 1
        should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

        update_serial_yaw()
        scan_data, _ = get_scan_and_front_dist(scanner_instance)
        front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
        parallel_error = get_wall_parallel_error(scan_data, align_side)
        elapsed = time.monotonic() - start_time

        if parallel_error is None:
            if no_wall_start_time == 0.0:
                no_wall_start_time = time.monotonic()
            no_wall_elapsed = time.monotonic() - no_wall_start_time
        else:
            no_wall_start_time = 0.0
            no_wall_elapsed = 0.0

        is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
        hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
        no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

        if should_log:
            err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
            print(f"[{tag}] Side:{align_side} Front:{front_avg} Rear:{rear_avg} "
                  f"Err:{err_str} FrontPts:{front_count} RearPts:{rear_count} Elapsed:{elapsed:.2f}s")

        if is_aligned or hard_timeout or no_wall_timeout:
            reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
            print(f"[{tag}] Exit ({reason}). Elapsed:{elapsed:.2f}s")
            send_esp_packet(SERVO_CENTER_ANGLE, 0)
            time.sleep(CORNER_BRAKE_DELAY)
            break

        if parallel_error is None:
            send_esp_packet(SERVO_CENTER_ANGLE, int(speed * direction_multiplier))
        else:
            normalized_error = parallel_error if align_side == "left" else -parallel_error
            pid_output = direction_multiplier * alignment_pid.update(normalized_error)
            target = SERVO_CENTER_ANGLE - pid_output
            final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - SERVO_CLAMP, SERVO_CENTER_ANGLE + SERVO_CLAMP)))
            send_esp_packet(final_servo, int(speed * direction_multiplier))

        time.sleep(0.02)


def _approach_wall(scanner_instance=None):
    """Creeps toward corner wall with live LiDAR data until front distance target is met."""
    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
    wall_follow_pid.reset()
    alignment_pid.reset()
    frame_counter = 0

    while not global_shutdown_event.is_set():
        frame_counter += 1
        should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

        update_serial_yaw()
        scan_data, front_dist = get_scan_and_front_dist(scanner_instance)

        if front_dist < CORNER_APPROACH_TRIGGER_MM:
            print(f"[APPROACH] Trigger reached: Front {front_dist:.1f}mm < "
                  f"{CORNER_APPROACH_TRIGGER_MM:.0f}mm. Braking...")
            send_esp_packet(SERVO_CENTER_ANGLE, 0)
            time.sleep(CORNER_BRAKE_DELAY)
            break

        tracked_side_pts = [scan_data[a] for a in (range(-90, -39) if CLOCKWISE_WALL_FOLLOWING else range(40, 91))
                            if a in scan_data and scan_data[a] > 0]

        servo, mode_note = compute_wall_follow_servo(scan_data, tracked_side_pts, align_side, wall_follow_pid, alignment_pid)
        if should_log:
            print(f"[APPROACH] Front:{front_dist:7.1f}mm / {CORNER_APPROACH_TRIGGER_MM:.0f}mm "
                  f"| {mode_note} | servo={servo}")
        send_esp_packet(servo, ROBOT_CRUISE_SPEED)
        time.sleep(0.02)


def execute_cornering(ser_port=None, external_scan_data=None, scanner_instance=None):
    """
    Primary corner maneuver handler.
    Accepts:
      - ser_port: Active serial port connected to ESP32
      - external_scan_data: Snapshot LiDAR scan dict
      - scanner_instance: Live LidarScanner instance object from main thread

    Call this directly once your LiDAR-based corner detector (e.g. the
    split-and-merge / RDP L-shape corner segmenter) flags a corner.

    Turn direction (LEFT/RIGHT) is a fixed property of the course direction
    (CLOCKWISE_WALL_FOLLOWING), set by the corner-signature file at import time.
    Lane number / arc-vs-inplace, however, is now decided at the SECOND stop
    (right after the approach-wall creep brakes at CORNER_APPROACH_TRIGGER_MM),
    using a fresh, stationary LiDAR reading -- not the noisy first-stop signature
    snapshot.
    """
    global esp_ser, latest_lidar_data, lidar_scanner

    # Bind active handles from main program context
    if ser_port is not None:
        esp_ser = ser_port
    if scanner_instance is not None:
        lidar_scanner = scanner_instance

    if external_scan_data:
        with lidar_data_lock:
            latest_lidar_data = external_scan_data.copy()

    print("\n[CORNER] Executing corner maneuver sequence...")

    # Brake before the pre-align creep
    send_esp_packet(SERVO_CENTER_ANGLE, 0)
    time.sleep(CORNER_BRAKE_DELAY)

    # Turn direction is fixed by course direction -- not inferred from distances.
    turn_direction = "RIGHT" if CLOCKWISE_WALL_FOLLOWING else "LEFT"

    # --- FIRST STOP -> SECOND STOP: pre-align then creep in to the corner wall ---
    _align_to_wall(WALL_ALIGN_CREEP_SPEED, direction_multiplier=1.0, scanner_instance=lidar_scanner, tag="PRE-ALIGN")
    _approach_wall(scanner_instance=lidar_scanner)
    # _approach_wall() already brakes + sleeps CORNER_BRAKE_DELAY once front_dist
    # drops below CORNER_APPROACH_TRIGGER_MM -- robot is stationary at the second
    # stop now, so this is the clean moment to sample left/right for the lane call.

    # --- SECOND STOP: decide arc vs. in-place turn from a fresh, settled reading ---
    scan_data, _ = get_scan_and_front_dist(lidar_scanner)
    left_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
    right_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
    avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
    avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0

    locked_lane_reference_mm = avg_left if CLOCKWISE_WALL_FOLLOWING else avg_right

    if locked_lane_reference_mm > LANE_REFERENCE_ARC_THRESHOLD_MM:
        lane_number = 1
        use_reverse_arc = True
    else:
        lane_number = "2/3"
        use_reverse_arc = False

    print(f"[CORNER] Second-stop lane read: L={avg_left:.0f}mm R={avg_right:.0f}mm "
          f"locked_ref={locked_lane_reference_mm:.0f}mm -> lane={lane_number} "
          f"arc={use_reverse_arc} dir={turn_direction}")

    # Execute pivot with live LiDAR sampling
    _execute_pivot(turn_direction, lane_number, use_reverse_arc)

    # if lane_number == 1:
    _align_to_wall(WALL_ALIGN_CREEP_SPEED, direction_multiplier=-1.0, scanner_instance=lidar_scanner, tag="POST-PIVOT ALIGN")
    _execute_backward_maneuver(scanner_instance=lidar_scanner)

    # _align_to_wall(WALL_ALIGN_CREEP_SPEED, direction_multiplier=1.0, scanner_instance=lidar_scanner, tag="WALL ALIGN")

    # Clean up and reset PID / Yaw
    send_reset_yaw()
    time.sleep(0.1)
    wall_follow_pid.reset()
    alignment_pid.reset()
    print("[CORNER] Corner maneuver completed successfully.\n")


def _execute_pivot(turn_direction, lane_number, use_reverse_arc):
    """Executes turn while actively parsing live YAW from ESP32."""
    global current_yaw
    send_reset_yaw()
    # time.sleep(0.1)
    current_yaw = 0.0
    start_time = time.monotonic()

    frame_counter = 0

    if use_reverse_arc:
        if turn_direction == "RIGHT":
            arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
        else:  # LEFT
            arc_steer_angle = SERVO_CENTER_ANGLE + CORNER_ARC_STEER_OFFSET

        send_esp_packet(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)
        target_deg = abs(TURN_TARGET_RIGHT_DEGREES)  # magnitude only — fine as-is since you compare abs(current_yaw)

        while not global_shutdown_event.is_set():
            frame_counter += 1
            should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

            update_serial_yaw()
            elapsed = time.monotonic() - start_time
            arc_complete = abs(current_yaw) >= target_deg
            timed_out = elapsed >= CORNER_ARC_PIVOT_SAFETY_TIMEOUT

            if should_log:
                print(f"[ARC PIVOT] Yaw:{current_yaw:+.1f}° / {target_deg:.0f}° "
                      f"({turn_direction}) steer={arc_steer_angle}° | Elapsed:{elapsed:.2f}s")

            if arc_complete or timed_out:
                if timed_out and not arc_complete:
                    print(f"[ARC PIVOT] WARNING: timeout before target reached ({current_yaw:+.1f}°).")
                print(f"[ARC PIVOT] Complete ({current_yaw:+.1f}°). Braking...")
                break
            send_esp_packet(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)  # keep refreshing (see earlier note)
            time.sleep(0.02)
    else:
        final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
        send_esp_packet(final_servo, 0)
        time.sleep(0.15)
        send_esp_packet(final_servo, CORNER_PIVOT_SPEED)

        target_deg = abs(TURN_TARGET_RIGHT_DEGREES if turn_direction == "RIGHT" else TURN_TARGET_LEFT_DEGREES)

        while not global_shutdown_event.is_set():
            frame_counter += 1
            should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

            update_serial_yaw()  # Live IMU feedback processing
            elapsed = time.monotonic() - start_time

            # Compare absolute magnitude only (works for both LEFT and RIGHT)
            pivot_complete = abs(current_yaw) >= target_deg
            timed_out = elapsed >= CORNER_PIVOT_SAFETY_TIMEOUT

            if should_log:
                print(f"[PIVOT] Yaw:{current_yaw:+.1f}° / target {target_deg:.0f}° "
                      f"({turn_direction}) servo={final_servo}° | Elapsed:{elapsed:.2f}s")

            if pivot_complete or timed_out:
                if timed_out and not pivot_complete:
                    print(f"[PIVOT] WARNING: timeout before target reached ({current_yaw:+.1f}°).")
                print(f"[PIVOT] Complete ({current_yaw:+.1f}°). Braking...")
                break
            time.sleep(0.02)

    send_esp_packet(SERVO_CENTER_ANGLE, 0)
    time.sleep(CORNER_BRAKE_DELAY)


def _execute_backward_maneuver(scanner_instance=None):
    """
    Reverse maneuver for Lane 1 post-pivot positioning.

    Two behaviors added on top of the old fixed-timer-only version:

      1. Wall straightening while reversing: each loop iteration pulls a
         fresh scan and computes get_wall_parallel_error(scan_data, side,
         target_distance_mm=BACKWARD_WALL_TARGET_MM) -- the SAME blended
         alignment + distance-hold error _align_to_wall() uses going
         forward, from lidar_steering_new.py. That error is run through a
         PID (backward_align_pid) and applied as a servo correction while
         the speed command stays negative (reverse). Pass
         target_distance_mm=None in the call below instead if you want
         pure front/rear alignment with no distance-hold term.

      2. Stop condition: this now stops primarily when the rear HC-SR04
         ultrasonic sensor reads <= ULTRASONIC_STOP_DISTANCE_MM (400mm),
         not after a fixed duration. CORNER_BACKWARD_DURATION is kept as
         a safety-timeout ceiling in case the ultrasonic sensor never
         returns a valid reading (wiring fault, sensor out of range, etc.)
         so the robot can never reverse indefinitely.
    """
    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
    backward_align_pid.reset()
    start_time = time.monotonic()
    frame_counter = 0
    stop_reason = "timeout"

    print(f"[BACKWARD] Reversing. Target wall dist {BACKWARD_WALL_TARGET_MM:.0f}mm | "
          f"stop at ultrasonic <= {ULTRASONIC_STOP_DISTANCE_MM:.0f}mm "
          f"(safety ceiling {CORNER_BACKWARD_DURATION:.2f}s)...")

    while not global_shutdown_event.is_set():
        frame_counter += 1
        should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

        update_serial_yaw()
        elapsed = time.monotonic() - start_time

        rear_dist_mm = get_ultrasonic_distance_mm()
        print(f"back:{rear_dist_mm}")
        if rear_dist_mm is not None and rear_dist_mm <= ULTRASONIC_STOP_DISTANCE_MM:
            stop_reason = "ultrasonic"
            break
        if elapsed >= CORNER_BACKWARD_DURATION:
            stop_reason = "timeout"
            break

        scan_data, _ = get_scan_and_front_dist(scanner_instance)
        # target_distance_mm=None here instead would give pure front/rear
        # alignment (no distance-hold) while reversing.
        parallel_error = get_wall_parallel_error(scan_data, align_side, target_distance_mm=BACKWARD_WALL_TARGET_MM)

        if parallel_error is None:
            servo = SERVO_CENTER_ANGLE
        else:
            normalized_error = parallel_error if align_side == "left" else -parallel_error
            pid_output = backward_align_pid.update(normalized_error)
            servo = int(round(np.clip(SERVO_CENTER_ANGLE + pid_output,
                                       SERVO_CENTER_ANGLE - SERVO_CLAMP,
                                       SERVO_CENTER_ANGLE + SERVO_CLAMP)))

        if should_log:
            us_str = f"{rear_dist_mm:.0f}mm" if rear_dist_mm is not None else "N/A"
            err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
            print(f"[BACKWARD] Ultrasonic:{us_str} Err:{err_str} servo={servo} Elapsed:{elapsed:.2f}s")

        send_esp_packet(servo, -ROBOT_MANEUVER_SPEED)
        time.sleep(0.02)

    send_esp_packet(SERVO_CENTER_ANGLE, 0)
    print(f"[BACKWARD] Complete ({stop_reason}, {time.monotonic() - start_time:.2f}s elapsed).")
    # time.sleep(0.3)


# ============================================================
# WALL FOLLOWING CONTROL
# ============================================================
def compute_wall_follow_servo(scan_data, side_pts, align_side, wall_pid, align_pid):
    if side_pts:
        avg_wall = sum(side_pts) / len(side_pts)
        if avg_wall <= WALL_LOSS_THRESHOLD_MM:
            wall_error = (avg_wall - WALL_FOLLOW_TARGET_MM) if align_side == "left" else (WALL_FOLLOW_TARGET_MM - avg_wall)
            pid_output = wall_pid.update(wall_error)
            servo = SERVO_CENTER_ANGLE - pid_output
            return int(round(np.clip(servo, SERVO_CENTER_ANGLE - SERVO_CLAMP, SERVO_CENTER_ANGLE + SERVO_CLAMP))), "wall_follow"

    parallel_error = get_wall_parallel_error(scan_data, align_side)
    if parallel_error is None:
        return SERVO_CENTER_ANGLE, "no wall"
    normalized_error = parallel_error if align_side == "left" else -parallel_error
    pid_output = align_pid.update(normalized_error)
    servo = SERVO_CENTER_ANGLE - pid_output
    return int(round(np.clip(servo, SERVO_CENTER_ANGLE - SERVO_CLAMP, SERVO_CENTER_ANGLE + SERVO_CLAMP))), "parallel_align"


# ============================================================
# MAIN LOOP
# ============================================================
def control_loop():
    global esp_ser, lidar_scanner

    if serial is not None:
        try:
            esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        except Exception:
            esp_ser = None

    init_ultrasonic()

    try:
        lidar_scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        lidar_scanner.connect()
        threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,), daemon=True).start()
    except Exception as e:
        print(f"[FATAL] LiDAR required: {e}")
        sys.exit(1)

    corner_cooldown_end_time = 0.0
    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
    frame_counter = 0

    print("[SYSTEM] Autonomous loop running...")

    try:
        while not global_shutdown_event.is_set():
            frame_counter += 1
            should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

            update_serial_yaw()
            scan_data, avg_front = get_scan_and_front_dist()

            if (time.monotonic() - last_lidar_update_time) > LIDAR_STALE_TIMEOUT_SEC:
                send_esp_packet(SERVO_CENTER_ANGLE, 0, "lidar stale brake")
                time.sleep(0.02)
                continue

            in_cooldown = time.monotonic() < corner_cooldown_end_time

            if should_log:
                cooldown_note = f" | COOLDOWN {corner_cooldown_end_time - time.monotonic():.1f}s" if in_cooldown else ""
                print(f"[WATCHING] Front:{avg_front:7.1f}mm{cooldown_note}")

            # TODO: plug in your LiDAR-based corner detector here, e.g.:
            #   corner_flagged = your_split_and_merge_corner_detector(scan_data)
            #   if not in_cooldown and corner_flagged:
            #       execute_cornering()
            #       corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
            #       continue

            # Default Wall Following
            tracked_side_pts = [scan_data[a] for a in (range(-90, -39) if CLOCKWISE_WALL_FOLLOWING else range(40, 91))
                                if a in scan_data and scan_data[a] > 0]
            servo, _ = compute_wall_follow_servo(scan_data, tracked_side_pts, align_side, wall_follow_pid, alignment_pid)
            send_esp_packet(servo, ROBOT_CRUISE_SPEED)

            time.sleep(0.02)

    except Exception as e:
        print(f"[SYSTEM FAILURE] {e}")
    finally:
        emergency_shutdown_handler(None, None)

if __name__ == '__main__':
    control_loop()