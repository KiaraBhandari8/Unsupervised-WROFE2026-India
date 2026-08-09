import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response
import threading
import time
import os
import serial
import signal  # Native signal event tracking utility

# --- IMPORT CUSTOM VISION AND LIDAR EXTENSIONS ---
try: 
    from image_frame_combine_outer_inner_depth import process_frame_for_steering
    from lidar_steering_new import LidarScanner, PIDController, get_wall_parallel_error, get_wall_parallel_sector_stats, PARALLEL_TOLERANCE_MM
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to mount local tracking components: {e}")
    sys.exit(1)

# --- GLOBAL SHUTDOWN SYSTEM TRACKERS ---
global_shutdown_event = threading.Event()  # Master termination trigger event flag
esp_ser = None                              # Global handle for ESP32 serial link
lidar_scanner = None                        # Global handle for LiDAR object
picam2 = None                               # Global handle for camera driver

# --- LIDAR CONTROL DESIGN PARAMETERS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 200  # Front trigger distance line (20 cm)
WALL_LOSS_THRESHOLD_MM = 350.0  # Open pocket validation limit to ignore missing walls (NORMAL driving only)
CLOCKWISE_WALL_FOLLOWING = True  # Dynamically modified tracking direction flag
WALL_FOLLOW_TARGET_MM = 600

# =====================================================================
# FIX: proportional side-avoidance parameters, replacing the old fixed
# 180mm-trigger / fixed-15-degree response.
# -----------------------------------------------------------------------
# The old logic only reacted once a ray got closer than 180mm, with a
# flat 15-degree correction regardless of whether the wall was at
# 179mm or 20mm. On a fast-moving robot that's too late and too weak --
# by the time the correction lands, the chassis has already drifted
# into the wall ("rubbing"). This replaces it with:
#   - a wider WARN zone that starts correcting early, softly
#   - a harder PANIC zone that ramps the correction up sharply
#   - correction magnitude scaled to how deep into that zone the
#     closest point actually is, instead of one fixed value
# =====================================================================
LIDAR_SIDE_WARN_DISTANCE_MM = 250     # start a gentle correction here
LIDAR_SIDE_PANIC_DISTANCE_MM = 150    # by here, apply max correction
LIDAR_SIDE_STEER_MIN_MAGNITUDE = 6    # correction right as WARN zone is entered
LIDAR_SIDE_STEER_MAX_MAGNITUDE = 30   # correction at/inside PANIC distance
LIDAR_SIDE_AVOIDANCE_SPEED = 150      # slower than cruise speed while actively correcting


# NEW: require at least this many rays inside the WARN zone before trusting a
# close min() reading as real -- filters out single-ray sensor noise/reflections
# that would otherwise trigger a correction in genuinely open space.
LIDAR_SIDE_MIN_TRIGGER_POINTS = 2

# Kept for reference / backward compatibility with any external tuning notes --
# no longer used directly now that avoidance is proportional.
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180
LIDAR_LEFT_SIDE_DISTANCE_MM = 180

# =====================================================================
# FIX: real, always-acted-on front emergency brake.
# -----------------------------------------------------------------------
# The previous code called calculate_steering_error(..., safety_distance_mm=150)
# purely for its side-effect of printing "Commanding STOP" and returning
# 9999.0 when something was under 150mm dead ahead -- but that return
# value (side_alert) was never actually read or acted on anywhere in the
# control loop. That's why the log showed nine consecutive
# "WARNING! Obstacle at 33mm. Commanding STOP." lines while the robot
# kept driving into the wall. This constant/branch below actually stops
# (and briefly reverses) the chassis using avg_front_baseline, which is
# already computed every loop.
# =====================================================================
FRONT_EMERGENCY_STOP_MM = 150
FRONT_EMERGENCY_REVERSE_SPEED = 150
FRONT_EMERGENCY_REVERSE_SEC = 2


# Hardware Servo Limits
LIDAR_SERVO_MIN_ANGLE = 10
LIDAR_SERVO_MAX_ANGLE = 170

# --- OBSTACLE SIGHT THRESHOLD BOUNDARIES ---
FRONT_TURN_TRIGGER_MM = 200.0  # Strict 20cm front trigger boundary
FRONT_SCAN_ANGLE_DEG = 15      # Width of front scan cone (+/- 15°)

# --- GLOBAL BUFFER LOCKS AND REGISTERS ---
output_frame = None
output_frame_lock = threading.Lock()

latest_lidar_data = {}
lidar_data_lock = threading.Lock()
latest_tof_distance_mm = None
tof_data_lock = threading.Lock()
tof_sensor = None

latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

app = Flask(__name__)

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- CONTROL DESIGN CONSTANTS (8-BIT EXECUTION LAYER) ---
SERVO_CENTER_ANGLE = 100       # Absolute mechanical steering straight alignment midpoint
ROBOT_CRUISE_SPEED = 180      # Operational forward driving speed sent to ESP32 (0-255)
ROBOT_MANEUVER_SPEED = 155    # TUNED (was 155) -- backward docking speed, ported from corner_detection_debug.py

# --- NEW: INDEPENDENT VISION CALIBRATION PARAMETERS ---
STEERING_GAIN_GREEN = 0.1     # Baseline multiplier that keeps Green working perfectly
STEERING_GAIN_RED = 0.14      # INCREASE THIS to make the steering more aggressive for Red
RED_CLEARANCE_OFFSET = 0      # Static angular nudge (in degrees) to apush the chassis wider right

# NEW: Obstacle-avoidance exit debounce. Once avoidance latches on, this many
# CONSECUTIVE frames with no obstacle detected are required before the robot
# is allowed to release back to wall-align. This is frame-count based (not
# time-based) -- tune it against your actual measured loop FPS (see the
# on-screen FPS readout) so it corresponds to roughly 0.3-0.6s of confirmed
# clear view. Too low and glare/saturation blips will still cause premature
# exits; too high and the robot will coast past a genuinely-cleared obstacle
# on stale steering for too long.
OBSTACLE_MISS_EXIT_FRAMES = 10

# NEW: after obstacle avoidance fully clears, drive straight for this long before
# handing off to wall-align. Prevents the alignment PID from grabbing control while
# the chassis is still mid-clear of the obstacle / still at an angle from avoidance.
POST_AVOID_STRAIGHT_DURATION_SEC = 1.0

# --- GYRO DRIFT MANAGEMENT ---
PERIODIC_YAW_RESET_INTERVAL_SEC = 5.0   # NEW: re-zero gyro this often during normal driving
PERIODIC_YAW_RESET_MAX_DRIFT_DEG = 3.0  # NEW: only reset if already close to straight (avoids PID discontinuity)

CORNER_DETECTION_COOLDOWN_SEC = 10
CORNER_PIVOT_SPEED = 140              # TUNED (was 140), ported from corner_detection_debug.py
CORNER_PIVOT_SAFETY_TIMEOUT = 2.5
CORNER_BACKWARD_DURATION = 1.5        # TUNED (was 4)
CORNER_BACKWARD_TOF_TARGET_MM = 190.0
CORNER_BACKWARD_TOF_TOLERANCE_MM = 10.0
CORNER_BRAKE_DELAY = 0.20             # TUNED (was 0.25)
CORNER_CHECK_LOG_EVERY_N_FRAMES = 10
STATE_LOG_EVERY_N_FRAMES = 10         # throttle per-frame prints inside active corner sub-states

# --- CORNER SIGNATURE / LANE DETECTION PARAMETERS ---
CORNER_SIGNATURE_STOP_DELAY_SEC = 0.25          # brake pause when corner signature first detected
CORNER_SIGNATURE_FRONT_TRIGGER_MM = 1000.0      # TUNED (was 700.0) -- how far out a corner signature can fire
LANE_REFERENCE_ARC_THRESHOLD_MM = 600.0         # 60cm lane-1 cutoff
CORNER_APPROACH_TRIGGER_MM = 400.0              # TUNED (was 550.0) -- unified 2nd-stop front trigger, both pivot & arc

# --- ARC-REVERSE CORNER PARAMETERS (RIGHT turn, lane-1 wide case only) ---
CORNER_ARC_STEER_OFFSET = 60             # degrees off-center while reversing (toward SERVO_HARD_LEFT side)
CORNER_ARC_PIVOT_SPEED = 140             # TUNED (was 120), reverse speed magnitude during the arc
CORNER_ARC_PIVOT_SAFETY_TIMEOUT = 2.0    # TUNED (was 4.0)
CORNER_ARC_CLEAR_VIEW_TOF_MM = 400.0     # exit arc early once rear ToF reads ~40cm, for a clear front view sooner

TURN_TARGET_RIGHT_DEGREES = 68.0    # TUNED (was 80.0)
TURN_TARGET_LEFT_DEGREES = 68.0     # TUNED (was 80.0)
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0
WALL_ALIGN_CREEP_SPEED = 140
WALL_ALIGN_SAFETY_TIMEOUT = 3.0
WALL_ALIGN_NO_WALL_TIMEOUT = 1.0

# Dedicated wall-loss threshold for the corner-approach phase specifically -- kept SEPARATE
# from WALL_LOSS_THRESHOLD_MM (used by normal driving) so tuning the corner behavior doesn't
# silently change normal wall-following sensitivity elsewhere in the file.
CORNER_APPROACH_WALL_LOSS_THRESHOLD_MM = 400.0   # TUNED, ported from corner_detection_debug.py

# --- CAMERA CONFIGURATION MATRIX ---
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
HSV_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   
HSV_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  

# --- DEBUG MATRIX CONFIGURATION ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

# --- ROBOT LOGIC STATES ---


class RobotState:
    INITIALIZING = "INITIALIZING"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    VISION_OBSTACLE_AVOIDANCE = "VISION_OBSTACLE_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    LAP_TERMINATION = "LAP_TERMINATION"
    STOP = "STOP"
    WALL_ALIGN_CORRECTION = "WALL_ALIGN_CORRECTION"
    CORNER_PRE_ALIGN = "CORNER_PRE_ALIGN"          # NEW: straighten against wall before approach
    CORNER_APPROACH_WALL = "CORNER_APPROACH_WALL"
    CORNER_ACTIVE_PIVOT = "CORNER_ACTIVE_PIVOT"
    CORNER_ARC_ACTIVE_PIVOT = "CORNER_ARC_ACTIVE_PIVOT"
    CORNER_ALIGN_BACKWARD = "CORNER_ALIGN_BACKWARD"
    CORNER_POST_PIVOT_ALIGN = "CORNER_POST_PIVOT_ALIGN"
    FRONT_EMERGENCY_STOP = "FRONT_EMERGENCY_STOP"  # NEW
    POST_AVOID_STRAIGHT = "POST_AVOID_STRAIGHT"  # NEW: brief straight coast after avoidance clears

current_robot_state = RobotState.INITIALIZING
current_yaw = 0.0

def get_fixed_front_distance(scan_data):
    """Front distance using a fixed +/-FRONT_SCAN_ANGLE_DEG cone, ignoring yaw."""
    if not scan_data:
        return 2000.0

    fixed_points = [scan_data[a] for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
                     if a in scan_data and scan_data[a] > 0]

    if not fixed_points:
        return 2000.0

    return sum(fixed_points) / len(fixed_points)


# --- PACKET SYSTEM TRANSMISSION WRAPPERS ---
def send_esp_packet(ser_port, steering, speed):
    """Encapsulates control variables into standard serial strings safely."""
    if ser_port and ser_port.is_open and not global_shutdown_event.is_set():
        try:
            packet = f"STR:{steering},SPD:{speed}\n"
            ser_port.write(packet.encode('utf-8'))
        except Exception:
            pass

# ====================================================
# CRITICAL SIGNAL INTERCEPT HANDLER (Graceful Braking)
# ====================================================
def emergency_shutdown_handler(signum, frame):
    """Intercepts terminal signals immediately to safely kill mechanical movement."""
    print("\n\n[EMERGENCY BRAKE] Shutdown signal captured! Halting hardware registers...")
    global_shutdown_event.set()
    camera_thread_stop_event.set()
    
    global esp_ser, lidar_scanner, picam2
    
    if esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
            print("[CLEANUP] Safety stop dispatched. Serial interface closed securely.")
        except Exception as e:
            print(f"[CLEANUP ERROR] Failed to flush serial stop command: {e}")
            
    if lidar_scanner:
        try:
            lidar_scanner.disconnect()
            print("[CLEANUP] LiDAR scanner safely disconnected.")
        except Exception as e:
            print(f"[CLEANUP ERROR] Failed to kill lidar spin: {e}")
            
    if picam2:
        try:
            picam2.stop()
            print("[CLEANUP] Picamera2 resource array unmounted.")
        except:
            pass
            
    print("[SUCCESS] All mechanical systems isolated. Exiting clean.\n")
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_shutdown_handler)   # Intercepts Ctrl+C
signal.signal(signal.SIGQUIT, emergency_shutdown_handler)  # Intercepts Ctrl+\

# --- COLOR PROCESSING MASKS ---
def filter_blue_objects(hsv_frame):
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    return cv2.dilate(mask, kernel, iterations=2)

def detect_color_binary(mask, threshold=4000):
    return cv2.countNonZero(mask) > threshold

def get_compensated_front_distance(scan_data, current_yaw):
    """Dynamically shifts the scanning cone window and applies cosine projection math."""
    if not scan_data:
        return 2000.0

    yaw_offset = int(round(current_yaw))
    dynamic_angles = range(-FRONT_SCAN_ANGLE_DEG + yaw_offset, FRONT_SCAN_ANGLE_DEG + yaw_offset + 1)

    compensated_points = []
    yaw_rad = np.radians(current_yaw)

    for a in dynamic_angles:
        if a in scan_data and scan_data[a] > 0:
            raw_distance = scan_data[a]
            true_distance = raw_distance * np.cos(yaw_rad)
            compensated_points.append(true_distance)

    if not compensated_points:
        return 2000.0

    return sum(compensated_points) / len(compensated_points)


# =====================================================================
# FIX: proportional side-avoidance steering.
# Returns a signed servo offset (degrees) to add away from a close wall,
# scaling smoothly from 0 at LIDAR_SIDE_WARN_DISTANCE_MM up to
# LIDAR_SIDE_STEER_MAX_MAGNITUDE at/inside LIDAR_SIDE_PANIC_DISTANCE_MM,
# instead of the old single fixed-magnitude jump that only fired once
# the robot was already very close.
# =====================================================================
def compute_side_avoidance_offset(min_dist_mm):
    if min_dist_mm is None or min_dist_mm >= LIDAR_SIDE_WARN_DISTANCE_MM:
        return 0.0

    span = max(LIDAR_SIDE_WARN_DISTANCE_MM - LIDAR_SIDE_PANIC_DISTANCE_MM, 1.0)
    proximity_frac = min(1.0, (LIDAR_SIDE_WARN_DISTANCE_MM - min_dist_mm) / span)
    proximity_frac = max(0.0, proximity_frac)

    magnitude = LIDAR_SIDE_STEER_MIN_MAGNITUDE + proximity_frac * (
        LIDAR_SIDE_STEER_MAX_MAGNITUDE - LIDAR_SIDE_STEER_MIN_MAGNITUDE
    )
    return magnitude

# --- LIDAR DATA ACQUISITION BACKGROUND TASK ---
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("[SYSTEM] LiDAR background ingestion thread active.")
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
# --- TOF DATA ACQUISITION BACKGROUND TASK ---
def tof_acquisition_thread_func(sensor_instance):
    global latest_tof_distance_mm, tof_data_lock
    print("[SYSTEM] ToF background ingestion thread active.")
    tof_log_counter = 0
    while not global_shutdown_event.is_set():
        try:
            dist = sensor_instance.range
            with tof_data_lock:
                latest_tof_distance_mm = dist
            if tof_log_counter % 20 == 0:
                print(f"[TOF RAW] range={dist}mm")
        except Exception as e:
            with tof_data_lock:
                latest_tof_distance_mm = None
            if tof_log_counter % 20 == 0:
                print(f"[TOF RAW] read failed: {e}")
        tof_log_counter += 1
        time.sleep(0.03)

# --- CAMERA ACQUISITION BACKGROUND TASK ---
# OPTIMIZATION: derive the HSV source from the already-downscaled processing_frame_rgb
# instead of re-resizing the full-resolution captured frame a second time. This removes
# one full-res cv2.resize() call per loop iteration.
def camera_acquisition_thread_func(picam2_instance, stop_event, processing_size, hsv_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("[SYSTEM] Camera thread active. Processing dual-resize frame array.")
    try:
        while not stop_event.is_set() and not global_shutdown_event.is_set():
            captured_frame_rgb = picam2_instance.capture_array()

            processing_frame_rgb = cv2.resize(captured_frame_rgb, processing_size, interpolation=cv2.INTER_AREA)
            frame_bgr = cv2.cvtColor(processing_frame_rgb, cv2.COLOR_RGB2BGR)

            # Downscale further FROM the already-shrunk processing frame, not the raw capture.
            hsv_source_frame = cv2.resize(processing_frame_rgb, hsv_processing_size, interpolation=cv2.INTER_AREA)
            hsv_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_RGB2HSV)

            with camera_frame_lock:
                latest_processed_frames['rgb'] = processing_frame_rgb
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['hsv'] = hsv_frame

            # Small yield so this thread doesn't starve the control loop / Flask thread.
            time.sleep(0.001)
    except Exception as e:
        if not global_shutdown_event.is_set():
            print(f"[CRITICAL] Camera acquisition thread crashed: {e}")

# --- MAIN ROBOT NAVIGATION EXECUTION ENGINE ---
def robot_control_loop():
    global output_frame, output_frame_lock, current_robot_state, latest_processed_frames, camera_frame_lock
    global CLOCKWISE_WALL_FOLLOWING, current_yaw, esp_ser, lidar_scanner, picam2

    # Initialize Hardware Serial Bus Connection
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] High-speed serial connection established with ESP32 execution layer.")
    except Exception as e:
        print(f"[FATAL] Serial bridge initialization failed on {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # ====================================================
    # NEW: STARTUP GYRO RESET
    # Zero the gyro right away, before anything else spins up, so current_yaw
    # starts from a known-good baseline of 0.0 rather than whatever the ESP32's
    # IMU happened to be reporting at power-on (residual drift, mounting offset,
    # or leftover state from a previous run).
    # ====================================================
    print("[INFO] Performing startup gyro reset...")
    try:
        esp_ser.write(b"RST_YAW\n")
        esp_ser.flush()
        time.sleep(0.2)
        # Drain any immediate YAW: lines so we don't act on stale pre-reset values.
        while esp_ser.in_waiting > 0:
            esp_ser.readline()
    except Exception as e:
        print(f"[WARN] Startup gyro reset failed to send: {e}")
    current_yaw = 0.0
    print("[INFO] Startup gyro reset complete. current_yaw = 0.0")

    # Initialize Camera Pipelines
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(1) 

    processing_size = (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    hsv_processing_size = (HSV_PROCESSING_WIDTH, HSV_PROCESSING_HEIGHT)

    camera_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, processing_size, hsv_processing_size)
    )
    camera_thread.daemon = True
    camera_thread.start()

    # Initialize LiDAR Sensors
    try:
        lidar_scanner = LidarScanner(port='/dev/ttyUSB0', baudrate=230400)
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        print("[INFO] LiDAR scanner pipeline mounted safely.")
    except Exception as e:
        print(f"[WARN] LiDAR interface offline: {e}. Switching to vision fallback maps.")
        lidar_scanner = None
    # Initialize Rear ToF Distance Sensor
    global tof_sensor
    try:
        import board
        import busio
        import adafruit_vl53l0x
        i2c_tof = busio.I2C(board.SCL, board.SDA)
        tof_sensor = adafruit_vl53l0x.VL53L0X(i2c_tof)
        tof_thread = threading.Thread(target=tof_acquisition_thread_func, args=(tof_sensor,))
        tof_thread.daemon = True
        tof_thread.start()
        print("[INFO] Rear ToF sensor pipeline mounted safely.")
    except Exception as e:
        print(f"[WARN] ToF sensor offline: {e}. Backward phase will rely on timeout only.")
        tof_sensor = None

    # Initialize Controller Mathematics Loops
    gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
    wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)
    alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)

    # Set Initial Behavioral States
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
    turn_count = 0
    baseline_start_yaw = 0.0
    turn_direction = None
    use_reverse_arc = False
    locked_lane_reference_mm = None   # distance to tracked wall, locked at corner-detection instant
    lane_number = None                # 1 (arc-eligible) or "2/3" (pivot-only)
    pivot_phase_start_time = 0.0
    backward_phase_start_time = 0.0
    corner_cooldown_end_time = 0.0
    corner_check_frame_counter = 0
    state_log_frame_counter = 0       # throttles prints inside corner sub-states
    align_phase_start_time = 0.0
    align_return_state = None
    was_avoiding_obstacle = False
    align_no_wall_start_time = 0.0
    post_pivot_align_start_time = 0.0
    post_pivot_align_no_wall_start_time = 0.0
    pre_align_start_time = 0.0            # NEW: pre-turn wall-straighten phase timer
    pre_align_no_wall_start_time = 0.0    # NEW: pre-turn "no wall visible" sub-timer
    last_periodic_yaw_reset_time = time.monotonic()   # NEW: periodic drift-correction tracker

        # replace the two flags:
    obstacle_avoidance_active = False
    obstacle_miss_streak = 0
    last_avoid_servo_angle = SERVO_CENTER_ANGLE
    last_avoid_logic_label = None
    post_avoid_straight_start_time = 0.0
    post_avoid_active = False   # NEW: independent of current_robot_state

    # FIX: front-emergency-stop phase registers.
    front_emergency_active = False
    front_emergency_start_time = 0.0

    # Blue Line Crossing Telemetry Registers
    blue_count = 0
    prev_blue_state = False
    blue_cooldown_end_time = 0.0
    
    print(f"[SYSTEM] Calibration complete. Initial State: {current_robot_state}")


    CORNER_ACTIVE_STATES = [
        RobotState.CORNER_PRE_ALIGN, RobotState.CORNER_APPROACH_WALL,
        RobotState.CORNER_ACTIVE_PIVOT, RobotState.CORNER_ARC_ACTIVE_PIVOT,
        RobotState.CORNER_POST_PIVOT_ALIGN, RobotState.CORNER_ALIGN_BACKWARD,
        RobotState.WALL_ALIGN_CORRECTION,
    ]

    display_text = "MODE: Initializing"

    try:
        while not global_shutdown_event.is_set():
            loop_start_time = time.monotonic()

            while esp_ser.in_waiting > 0:
                try:
                    raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line.startswith("YAW:"):
                        current_yaw = float(raw_line.split(":")[1])
                except Exception:
                    pass

            if (current_robot_state not in CORNER_ACTIVE_STATES
                and abs(current_yaw) < PERIODIC_YAW_RESET_MAX_DRIFT_DEG
                and (time.monotonic() - last_periodic_yaw_reset_time) >= PERIODIC_YAW_RESET_INTERVAL_SEC):
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                current_yaw = 0.0
                last_periodic_yaw_reset_time = time.monotonic()
                print(f"[GYRO] Periodic drift-correction reset (state={current_robot_state}).")

            with camera_frame_lock:
                if not latest_processed_frames:
                    time.sleep(0.01)
                    continue
                frame_bgr = latest_processed_frames['bgr'].copy()
                hsv = latest_processed_frames['hsv'].copy()

            scan_data = {}
            if lidar_scanner:
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            avg_front_baseline = get_compensated_front_distance(scan_data, current_yaw)
            fixed_front_distance = get_fixed_front_distance(scan_data)

            left_pts = [scan_data[a] for a in range(-105, -75) if a in scan_data and scan_data[a] > 0]
            right_pts = [scan_data[a] for a in range(75, 105) if a in scan_data and scan_data[a] > 0]
            avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
            avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
            in_cooldown = time.monotonic() < corner_cooldown_end_time

            if scan_data and corner_check_frame_counter % CORNER_CHECK_LOG_EVERY_N_FRAMES == 0:
                cooldown_note = f" | COOLDOWN ({corner_cooldown_end_time - time.monotonic():.1f}s left)" if in_cooldown else ""
                print(f"[{current_robot_state}] [CORNER CHECK] Front: {avg_front_baseline:.1f}mm | Left: {avg_left:.1f}mm | Right: {avg_right:.1f}mm | Yaw: {current_yaw:+.1f}°{cooldown_note}")

            corner_check_frame_counter += 1

            is_near_field_mode = avg_front_baseline < 1100.0
            processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                frame_bgr, use_outer_roi_and_bottom_point=is_near_field_mode
            )
            vision_angle = -1 * vision_angle

            current_timestamp = time.time()
            blue_mask = filter_blue_objects(hsv)
            blue_in_view = detect_color_binary(blue_mask, threshold=4000)

            if not blue_in_view and prev_blue_state:
                if current_timestamp > blue_cooldown_end_time:
                    blue_count += 1
                    print(f"[RACE telemet] Blue line passed! Total lines crossed: {blue_count}/12")
                    blue_cooldown_end_time = current_timestamp + 5.0
            prev_blue_state = blue_in_view

            # ====================================================
            # PRIORITY LEVEL 0: LAP TERMINATION OVERRIDE
            # ====================================================
            if current_robot_state == RobotState.LAP_TERMINATION or turn_count >= 12:
                current_robot_state = RobotState.LAP_TERMINATION
                print("\n==========================================================")
                print(f"[MATCH COMPLETE] 12 Race Turns Logged! Locking wheels to finish...")
                print("==========================================================")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                time.sleep(4.0)
                
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                print("[SYSTEM] Hard race shutdown executed successfully.")
                break

            # ====================================================
            # PRIORITY LEVEL 1: ACTIVE CORNERING RUNTIME EXECUTION
            # ====================================================
            if current_robot_state in [RobotState.CORNER_PRE_ALIGN, RobotState.CORNER_APPROACH_WALL, RobotState.CORNER_ACTIVE_PIVOT, RobotState.CORNER_ARC_ACTIVE_PIVOT, RobotState.CORNER_POST_PIVOT_ALIGN, RobotState.CORNER_ALIGN_BACKWARD]:

                state_log_frame_counter += 1
                should_log_state = (state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

                if current_robot_state == RobotState.CORNER_PRE_ALIGN:
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                    parallel_error = get_wall_parallel_error(scan_data, align_side)
                    elapsed_pre_align = time.monotonic() - pre_align_start_time

                    if parallel_error is None:
                        if pre_align_no_wall_start_time == 0.0:
                            pre_align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - pre_align_no_wall_start_time
                    else:
                        pre_align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    hard_timeout = elapsed_pre_align >= WALL_ALIGN_SAFETY_TIMEOUT
                    no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                    display_text = (
                        f"[CORNER] Phase 1.5: Pre-Turn Straighten | Side: {align_side.title()} | Err: {parallel_error:.1f}mm"
                        if parallel_error is not None else
                        f"[CORNER] Phase 1.5: Pre-Turn Straighten | Side: {align_side.title()} | Err: N/A"
                    )

                    if should_log_state:
                        print(
                            f"[{current_robot_state}] [PRE-TURN STRAIGHTEN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                            f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                            f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_pre_align:.2f}s"
                        )

                    if is_aligned or hard_timeout or no_wall_timeout:
                        reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                        print(f"[{current_robot_state}] [PRE-TURN STRAIGHTEN] Exit condition reached ({reason}). Proceeding to corner approach.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)
                        alignment_pid.reset()
                        current_robot_state = RobotState.CORNER_APPROACH_WALL
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                    else:
                        if parallel_error is None:
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = alignment_pid.update(normalized_error)
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            send_esp_packet(esp_ser, final_servo_angle, WALL_ALIGN_CREEP_SPEED)

                elif current_robot_state == RobotState.CORNER_APPROACH_WALL:
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

                    if avg_front_baseline < CORNER_APPROACH_TRIGGER_MM:
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Approach limit reached. Front {avg_front_baseline:.1f}mm < {CORNER_APPROACH_TRIGGER_MM:.0f}mm. Applying hard brake...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Resetting gyro registers before turn...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        pivot_phase_start_time = time.monotonic()
                        baseline_start_yaw = current_yaw

                        if use_reverse_arc:
                            arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Entering reverse arc pivot (Lane 1): steer={arc_steer_angle}° speed=-{CORNER_ARC_PIVOT_SPEED}")
                            current_robot_state = RobotState.CORNER_ARC_ACTIVE_PIVOT
                            send_esp_packet(esp_ser, arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)
                        else:
                            final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Locking wheels to extreme pivot angle (Lane {lane_number}): {final_servo}°")
                            send_esp_packet(esp_ser, final_servo, 0)
                            time.sleep(0.15)
                            current_robot_state = RobotState.CORNER_ACTIVE_PIVOT
                            send_esp_packet(esp_ser, final_servo, CORNER_PIVOT_SPEED)
                    else:
                        wall_pts = left_pts if CLOCKWISE_WALL_FOLLOWING else right_pts
                        parallel_error = get_wall_parallel_error(scan_data, align_side)

                        if wall_pts:
                            avg_wall = sum(wall_pts) / len(wall_pts)
                            if avg_wall > CORNER_APPROACH_WALL_LOSS_THRESHOLD_MM:
                                if parallel_error is None:
                                    approach_servo_angle = SERVO_CENTER_ANGLE
                                    display_text = f"[CORNER] Phase 2: Approach (Wall Lost, No Parallel Data) | Front: {avg_front_baseline:.0f}mm"
                                else:
                                    normalized_error = parallel_error if align_side == "left" else -parallel_error
                                    pid_output = alignment_pid.update(normalized_error)
                                    approach_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                    display_text = f"[CORNER] Phase 2: Approach (Wall Lost -> Parallel Straighten) | Front: {avg_front_baseline:.0f}mm | Err: {parallel_error:.0f}mm"
                            else:
                                wall_error = (avg_wall - WALL_FOLLOW_TARGET_MM) if CLOCKWISE_WALL_FOLLOWING else (WALL_FOLLOW_TARGET_MM - avg_wall)
                                pid_output = wall_follow_pid.update(wall_error)
                                approach_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                display_text = f"[CORNER] Phase 2: Approach (Wall Follow) | Front: {avg_front_baseline:.0f}mm | Err: {wall_error:.0f}mm"
                        else:
                            if parallel_error is None:
                                approach_servo_angle = SERVO_CENTER_ANGLE
                                display_text = f"[CORNER] Phase 2: Approach (No Wall Data) | Front: {avg_front_baseline:.0f}mm"
                            else:
                                normalized_error = parallel_error if align_side == "left" else -parallel_error
                                pid_output = alignment_pid.update(normalized_error)
                                approach_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                display_text = f"[CORNER] Phase 2: Approach (No Side-Wall Pts -> Parallel Straighten) | Front: {avg_front_baseline:.0f}mm | Err: {parallel_error:.0f}mm"

                        final_approach_servo = int(round(np.clip(approach_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                        if should_log_state:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] {display_text} | servo={final_approach_servo} | Lane: {lane_number}")
                        send_esp_packet(esp_ser, final_approach_servo, ROBOT_CRUISE_SPEED)

                elif current_robot_state == RobotState.CORNER_ACTIVE_PIVOT:
                    elapsed_pivot = time.monotonic() - pivot_phase_start_time
                    yaw_delta_signed = current_yaw - baseline_start_yaw

                    if turn_direction == "RIGHT":
                        target_degrees = TURN_TARGET_RIGHT_DEGREES
                        pivot_complete = yaw_delta_signed <= -TURN_TARGET_RIGHT_DEGREES
                    else:
                        target_degrees = TURN_TARGET_LEFT_DEGREES
                        pivot_complete = yaw_delta_signed >= TURN_TARGET_LEFT_DEGREES

                    display_text = f"[CORNER] Phase 3: Gyro Pivot | Yaw: {yaw_delta_signed:+.1f}° / target {target_degrees}° ({turn_direction}) | Lane: {lane_number}"
                    pivot_timed_out = elapsed_pivot >= CORNER_PIVOT_SAFETY_TIMEOUT

                    if pivot_complete or pivot_timed_out:
                        if pivot_timed_out and not pivot_complete:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] WARNING: Pivot safety timeout hit before target yaw reached (only {yaw_delta_signed:+.1f}° / target {target_degrees}°). Yaw: {current_yaw:+.1f}°. Check gyro data.")
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Pivot complete ({yaw_delta_signed:+.1f}°). Braking...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Resetting chassis orientation...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        if lane_number == 1:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Lane 1 -> post-pivot align (backward creep).")
                            post_pivot_align_start_time = time.monotonic()
                            post_pivot_align_no_wall_start_time = 0.0
                            current_robot_state = RobotState.CORNER_POST_PIVOT_ALIGN
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        else:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Lane {lane_number} -> skipping ALL backward movement, forward-aligning directly.")
                            turn_count += 1
                            gyro_straight_pid.reset()
                            wall_follow_pid.reset()
                            alignment_pid.reset()
                            corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
                            align_return_state = RobotState.LIDAR_WALL_FOLLOWING
                            align_phase_start_time = time.monotonic()
                            align_no_wall_start_time = 0.0
                            current_robot_state = RobotState.WALL_ALIGN_CORRECTION
                            turn_direction = None
                            locked_lane_reference_mm = None
                            lane_number = None
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                    else:
                        final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                        send_esp_packet(esp_ser, final_servo, CORNER_PIVOT_SPEED)
                        if should_log_state:
                            print(f"[{current_robot_state}] [PIVOT] Yaw: {yaw_delta_signed:+.1f}° / target {target_degrees}° | Elapsed: {elapsed_pivot:.2f}s")

                elif current_robot_state == RobotState.CORNER_ARC_ACTIVE_PIVOT:
                    elapsed_arc = time.monotonic() - pivot_phase_start_time
                    yaw_delta_signed = current_yaw - baseline_start_yaw
                    arc_complete = yaw_delta_signed <= -TURN_TARGET_RIGHT_DEGREES
                    arc_timed_out = elapsed_arc >= CORNER_ARC_PIVOT_SAFETY_TIMEOUT

                    with tof_data_lock:
                        rear_distance = latest_tof_distance_mm

                    rear_tof_clear_view_hit = (
                        rear_distance is not None
                        and rear_distance <= CORNER_ARC_CLEAR_VIEW_TOF_MM
                    )

                    display_text = f"[CORNER] Arc Reverse Pivot | Yaw: {yaw_delta_signed:+.1f}° / target -{TURN_TARGET_RIGHT_DEGREES}° | Rear: {rear_distance if rear_distance is not None else float('nan'):.0f}mm"
                    if should_log_state:
                        print(f"[{current_robot_state}] [ARC PIVOT] Yaw: {yaw_delta_signed:+.1f}° | Rear: {rear_distance} | Elapsed: {elapsed_arc:.2f}s")

                    if arc_complete or arc_timed_out or rear_tof_clear_view_hit:
                        if arc_timed_out and not arc_complete:
                            print(f"[{current_robot_state}] [ARC PIVOT] WARNING: Arc safety timeout hit before target yaw reached (only {yaw_delta_signed:+.1f}°). Check gyro data.")
                        if rear_tof_clear_view_hit:
                            print(f"[{current_robot_state}] [ARC PIVOT] Rear ToF clear-view distance reached ({rear_distance:.0f}mm <= {CORNER_ARC_CLEAR_VIEW_TOF_MM:.0f}mm). Ending arc early for forward visibility.")
                        print(f"[{current_robot_state}] [ARC PIVOT] Arc complete ({yaw_delta_signed:+.1f}°). Hard braking chassis...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [ARC PIVOT] Resetting chassis orientation before post-pivot align...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        post_pivot_align_start_time = time.monotonic()
                        post_pivot_align_no_wall_start_time = 0.0
                        current_robot_state = RobotState.CORNER_POST_PIVOT_ALIGN
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                    else:
                        arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                        send_esp_packet(esp_ser, arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)

                elif current_robot_state == RobotState.CORNER_POST_PIVOT_ALIGN:
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                    parallel_error = get_wall_parallel_error(scan_data, align_side)
                    elapsed_post_pivot_align = time.monotonic() - post_pivot_align_start_time

                    if parallel_error is None:
                        if post_pivot_align_no_wall_start_time == 0.0:
                            post_pivot_align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - post_pivot_align_no_wall_start_time
                    else:
                        post_pivot_align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    hard_timeout = elapsed_post_pivot_align >= WALL_ALIGN_SAFETY_TIMEOUT
                    no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                    display_text = (
                        f"[CORNER] Phase 3.5: Post-Pivot Align | Side: {align_side.title()} | Err: {parallel_error:.1f}mm"
                        if parallel_error is not None else
                        f"[CORNER] Phase 3.5: Post-Pivot Align | Side: {align_side.title()} | Err: N/A"
                    )

                    if should_log_state:
                        print(
                            f"[{current_robot_state}] [POST-PIVOT ALIGN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                            f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                            f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_post_pivot_align:.2f}s"
                        )

                    if is_aligned or hard_timeout or no_wall_timeout:
                        reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                        print(f"[{current_robot_state}] [POST-PIVOT ALIGN] Exit ({reason}). Lane 1 -> backward phase.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)
                        alignment_pid.reset()
                        backward_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                    else:
                        if parallel_error is None:
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -WALL_ALIGN_CREEP_SPEED)
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = -alignment_pid.update(normalized_error)
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            send_esp_packet(esp_ser, final_servo_angle, -WALL_ALIGN_CREEP_SPEED)

                elif current_robot_state == RobotState.CORNER_ALIGN_BACKWARD:
                    elapsed_back = time.monotonic() - backward_phase_start_time

                    with tof_data_lock:
                        rear_distance = latest_tof_distance_mm

                    tof_reached = (
                        rear_distance is not None
                        and rear_distance <= (CORNER_BACKWARD_TOF_TARGET_MM + CORNER_BACKWARD_TOF_TOLERANCE_MM)
                    )
                    hard_timeout_backward = elapsed_back >= CORNER_BACKWARD_DURATION

                    display_text = (
                        f"[CORNER] Phase 4: Backing Away | Rear: {rear_distance:.0f}mm / {CORNER_BACKWARD_TOF_TARGET_MM:.0f}mm"
                        if rear_distance is not None else
                        f"[CORNER] Phase 4: Backing Away | Rear: N/A | {elapsed_back:.1f}s / {CORNER_BACKWARD_DURATION}s"
                    )
                    if should_log_state:
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Phase 4: Reversing | Rear: {rear_distance} | Elapsed: {elapsed_back:.2f}s | Yaw: {current_yaw:+.1f}°")

                    if tof_reached or hard_timeout_backward:
                        if hard_timeout_backward and not tof_reached:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] WARNING: Backward safety timeout hit before ToF target reached (rear={rear_distance}). Check ToF sensor.")
                        else:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] ToF target reached (rear={rear_distance:.0f}mm). Hard braking applied.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(0.3)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Cleaning up spatial registers before resuming wall-follow...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        turn_count += 1
                        gyro_straight_pid.reset()
                        wall_follow_pid.reset()
                        alignment_pid.reset()
                        corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC

                        align_return_state = RobotState.LIDAR_WALL_FOLLOWING
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        current_robot_state = RobotState.WALL_ALIGN_CORRECTION
                        turn_direction = None
                        locked_lane_reference_mm = None
                        lane_number = None
                    else:
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)

                if DEBUG_UI_OVERLAYS:
                    loop_duration = time.monotonic() - loop_start_time
                    fps = 1.0 / loop_duration if loop_duration > 0 else 0
                    cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    if STREAM_VIDEO:
                        with output_frame_lock:
                            output_frame = processed_frame.copy()

                time.sleep(0.02)
                continue

            # ====================================================
            # PRIORITY LEVEL 2: RADAR FRONTAL TRIPWIRE CHECK (corner signature)
            # ====================================================
            is_corner_signature = False
            if (not in_cooldown
                and avg_front_baseline <= CORNER_SIGNATURE_FRONT_TRIGGER_MM
                and ((avg_left < 950.0 and avg_right > 2000.0)
                     or (avg_right < 900.0 and avg_left > 1800.0))):
                is_corner_signature = True

            if is_corner_signature:
                print(f"\n[CORNER INTERSECTION] Front Wall at: {avg_front_baseline:.1f}mm. left:{avg_left}-- right:{avg_right} Stopping chassis... (signature)")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                time.sleep(CORNER_SIGNATURE_STOP_DELAY_SEC)
                wall_follow_pid.reset()

                if avg_left < avg_right:
                    turn_direction = "RIGHT"
                    if turn_count == 0:
                        CLOCKWISE_WALL_FOLLOWING = True
                        print("[LAYOUT LOCKDOWN] Track direction set to: CLOCKWISE (CW)")
                else:
                    turn_direction = "LEFT"
                    if turn_count == 0:
                        CLOCKWISE_WALL_FOLLOWING = False
                        print("[LAYOUT LOCKDOWN] Track direction set to: COUNTER-CLOCKWISE (CCW)")

                locked_lane_reference_mm = avg_left if CLOCKWISE_WALL_FOLLOWING else avg_right

                if locked_lane_reference_mm > LANE_REFERENCE_ARC_THRESHOLD_MM:
                    lane_number = 1
                    use_reverse_arc = (turn_direction == "RIGHT")
                    if turn_direction != "RIGHT":
                        print("[LANE DETECT] Lane 1 detected on a LEFT turn -- arc geometry not "
                              "tuned for this side yet, falling back to pivot turn.")
                else:
                    lane_number = "2/3"
                    use_reverse_arc = False

                print(f"[LANE DETECT] Locked {'left' if CLOCKWISE_WALL_FOLLOWING else 'right'} wall dist: "
                      f"{locked_lane_reference_mm:.0f}mm (threshold {LANE_REFERENCE_ARC_THRESHOLD_MM:.0f}mm) -> "
                      f"Lane {lane_number} -> {'ARC turn' if use_reverse_arc else 'PIVOT turn'}")

                print(f"[PRE-TURN ACTUATION] Entering pre-turn wall straighten before {turn_direction} turn approach...")
                print("[ACTION] Flushing scrub vibration error -> Resetting Gyro Yaw to 0°...")
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                time.sleep(0.1)
                
                current_yaw = 0.0
                baseline_start_yaw = 0.0
                pre_align_start_time = time.monotonic()
                pre_align_no_wall_start_time = 0.0
                current_robot_state = RobotState.CORNER_PRE_ALIGN
                
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                continue

            # =====================================================================
            # FIX -- PRIORITY LEVEL 2.5: REAL FRONT EMERGENCY BRAKE
            # -----------------------------------------------------------------------
            # This is the actual fix for "rubbing into the wall". Previously the only
            # front-distance STOP logic lived inside calculate_steering_error(), whose
            # 9999.0 return value was computed but never read anywhere -- the log
            # would print "Commanding STOP" while the robot kept driving. This check
            # runs every loop, ahead of side-panic and vision-avoidance, using
            # avg_front_baseline (already computed above) against a real threshold,
            # and actually commands a stop + brief reverse.
            # =====================================================================
            if fixed_front_distance < FRONT_EMERGENCY_STOP_MM:
                if not front_emergency_active:
                    # Trigger fires ONCE per approach -- brake, then reverse for a fixed
                    # duration, then hand control back to normal driving regardless of
                    # what the distance reads by then (no more indefinite hold/wait).
                    print(f"[FRONT E-STOP] Obstacle/wall at {avg_front_baseline:.1f}mm (< {FRONT_EMERGENCY_STOP_MM:.0f}mm). Reversing {FRONT_EMERGENCY_REVERSE_SEC}s...")
                    front_emergency_active = True
                    front_emergency_start_time = time.monotonic()
                    send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)

                current_robot_state = RobotState.FRONT_EMERGENCY_STOP
                elapsed_emergency = time.monotonic() - front_emergency_start_time
                display_text = f"MODE: FRONT E-STOP | Front: {avg_front_baseline:.0f}mm | {elapsed_emergency:.1f}s / {FRONT_EMERGENCY_REVERSE_SEC}s"

                if elapsed_emergency < FRONT_EMERGENCY_REVERSE_SEC:
                    send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -FRONT_EMERGENCY_REVERSE_SPEED)

                    if DEBUG_UI_OVERLAYS:
                        cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        if STREAM_VIDEO:
                            with output_frame_lock:
                                output_frame = processed_frame.copy()

                    time.sleep(0.02)
                    continue
                else:
                    # 1s of reversing done -- release control back to normal driving,
                    # don't wait around for the distance to clear on its own.
                    print(f"[FRONT E-STOP] Reverse complete. Resuming normal driving.")
                    front_emergency_active = False
                    # fall through -- no `continue` here, execution proceeds to the
                    # rest of the loop (side-avoidance / vision / wall-align) this frame
            else:
                front_emergency_active = False

            # =====================================================================
            # FIX -- PRIORITY LEVEL 3: PROXIMITY SIDE-WALL AVOIDANCE (proportional)
            # -----------------------------------------------------------------------
            # Replaces the old fixed-180mm-trigger / fixed-15-degree response with a
            # correction that ramps up smoothly starting from LIDAR_SIDE_WARN_DISTANCE_MM
            # (currently 300mm) up to full strength at LIDAR_SIDE_PANIC_DISTANCE_MM
            # (150mm), using the actual closest point in each side sector instead of a
            # simple "any point under threshold" test. This reacts earlier and harder
            # the closer the wall actually is, instead of one flat nudge that only ever
            # fires once the robot is already very close.
            # =====================================================================
            right_side_zone = [scan_data[a] for a in range(40, 70) if a in scan_data and scan_data[a] > 0]
            left_side_zone = [scan_data[a] for a in range(-70, -40) if a in scan_data and scan_data[a] > 0]
            right_min_dist = min(right_side_zone) if right_side_zone else None
            left_min_dist = min(left_side_zone) if left_side_zone else None

            # NEW: count how many rays in each zone actually fall inside the WARN band --
            # a single ray reading close is easy to be sensor noise; require several
            # before trusting it as a genuine close-wall detection.
            right_warn_count = sum(1 for d in right_side_zone if d < LIDAR_SIDE_WARN_DISTANCE_MM)
            left_warn_count = sum(1 for d in left_side_zone if d < LIDAR_SIDE_WARN_DISTANCE_MM)

            right_min_dist_filtered = right_min_dist if right_warn_count >= LIDAR_SIDE_MIN_TRIGGER_POINTS else None
            left_min_dist_filtered = left_min_dist if left_warn_count >= LIDAR_SIDE_MIN_TRIGGER_POINTS else None

            right_offset = compute_side_avoidance_offset(right_min_dist_filtered)
            left_offset = compute_side_avoidance_offset(left_min_dist_filtered)
            print(f"right: {right_offset} (n={right_warn_count}) left: {left_offset} (n={left_warn_count})")
            print(f"right: {right_offset} left: {left_offset}")
            


            if right_offset > 0.0 or left_offset > 0.0:
                # If both sides are simultaneously encroaching (narrow gap), steer away
                # from whichever is currently closer/stronger rather than fighting itself.
                
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - right_offset if right_offset >= left_offset else SERVO_CENTER_ANGLE + left_offset
                robot_speed_current = LIDAR_SIDE_AVOIDANCE_SPEED
                display_text = "MODE: Side Avoid (during post-avoid window)" if post_avoid_active else "MODE: Side Avoid"
                # NOTE: post_avoid_active is intentionally left untouched here --
                # a brief safety correction no longer permanently cancels the straight-coast timer.
            
            # ====================================================
            # PRIORITY LEVEL 4: COMPUTER VISION PILLAR AVOIDANCE
            # ====================================================
            else:
                obstacle_detected_this_frame = logic_label in ["red_obstacle", "obstacle"]

                if obstacle_detected_this_frame:
                    obstacle_avoidance_active = True
                    obstacle_miss_streak = 0
                    was_avoiding_obstacle = True
                    post_avoid_active = False 
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    last_avoid_logic_label = logic_label

                    if logic_label == "red_obstacle":
                        servo_adjust = -vision_angle * STEERING_GAIN_RED
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust + RED_CLEARANCE_OFFSET
                        display_text = f"MODE: Red Avoid | Steer: {int(target_servo_angle)}°"
                    else:
                        servo_adjust = -vision_angle * STEERING_GAIN_GREEN
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                        display_text = f"MODE: Green Avoid | Steer: {int(target_servo_angle)}°"

                    last_avoid_servo_angle = target_servo_angle

                elif obstacle_avoidance_active and obstacle_miss_streak < OBSTACLE_MISS_EXIT_FRAMES:
                    obstacle_miss_streak += 1
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    target_servo_angle = last_avoid_servo_angle
                    display_text = f"MODE: Avoid Coast ({last_avoid_logic_label}) | Miss {obstacle_miss_streak}/{OBSTACLE_MISS_EXIT_FRAMES}"

                else:
                    obstacle_avoidance_active = False
                    obstacle_miss_streak = 0

                    # ====================================================
                    # PRIORITY LEVEL 5: POST-MANEUVER WALL ALIGNMENT (default drive mode)
                    # ====================================================
                    if was_avoiding_obstacle:
                        was_avoiding_obstacle = False
                         # NEW: just finished avoiding -> coast sFtraight for a beat before wall-align takes over
                        post_avoid_straight_start_time = time.monotonic()
                        post_avoid_active = True
                    
                    # NEW: handle the straight-coast phase
                    if post_avoid_active:
                        elapsed_post_avoid = time.monotonic() - post_avoid_straight_start_time
                        if elapsed_post_avoid < POST_AVOID_STRAIGHT_DURATION_SEC:
                            current_robot_state = RobotState.POST_AVOID_STRAIGHT
                            display_text = f"MODE: Post-Avoid Straight | {elapsed_post_avoid:.2f}s / {POST_AVOID_STRAIGHT_DURATION_SEC}s"
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                            if DEBUG_UI_OVERLAYS and STREAM_VIDEO:
                                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                                with output_frame_lock:
                                    output_frame = processed_frame.copy()
                            time.sleep(0.02)
                            continue
                        else:
                            post_avoid_active = False
                            # fall through into WALL_ALIGN_CORRECTION setup below
                        # else: 1s elapsed, fall through into WALL_ALIGN_CORRECTION setup below
                    if current_robot_state != RobotState.WALL_ALIGN_CORRECTION:
                        current_robot_state = RobotState.WALL_ALIGN_CORRECTION
                        align_return_state = RobotState.LIDAR_WALL_FOLLOWING
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        alignment_pid.reset()

                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                    parallel_error = get_wall_parallel_error(scan_data, align_side)
                    elapsed_align = time.monotonic() - align_phase_start_time

                    if parallel_error is None:
                        if align_no_wall_start_time == 0.0:
                            align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - align_no_wall_start_time
                    else:
                        align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    hard_timeout = elapsed_align >= WALL_ALIGN_SAFETY_TIMEOUT
                    no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                    display_text = (
                        f"MODE: Wall Align | Side: {align_side.title()} | Err: {parallel_error:.1f}mm"
                        if parallel_error is not None else
                        f"MODE: Wall Align | Side: {align_side.title()} | Err: N/A"
                    )

                    state_log_frame_counter += 1
                    if state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0:
                        print(
                            f"[{current_robot_state}] [WALL ALIGN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                            f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                            f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_align:.2f}s"
                        )

                    if is_aligned or hard_timeout or no_wall_timeout:
                        reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                        if state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0:
                            print(f"[{current_robot_state}] [WALL ALIGN] Re-centering cycle ({reason}).")
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        last_periodic_yaw_reset_time = time.monotonic()

                    if parallel_error is None:
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                    else:
                        normalized_error = parallel_error if align_side == "left" else -parallel_error
                        pid_output = alignment_pid.update(normalized_error)
                        target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                        final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                        send_esp_packet(esp_ser, final_servo_angle, WALL_ALIGN_CREEP_SPEED)

                    if DEBUG_UI_OVERLAYS:
                        loop_duration = time.monotonic() - loop_start_time
                        fps = 1.0 / loop_duration if loop_duration > 0 else 0
                        cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(processed_frame, f"State: {current_robot_state} | Align Side: {align_side}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(processed_frame, f"FrontPts: {front_count} RearPts: {rear_count} | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        if STREAM_VIDEO:
                            with output_frame_lock:
                                output_frame = processed_frame.copy()

                    time.sleep(0.02)
                    continue

            # 5. Output packets to hardware layers
            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0

            if DEBUG_UI_OVERLAYS:
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                if STREAM_VIDEO:
                    with output_frame_lock:
                        output_frame = processed_frame.copy()

            time.sleep(0.02)

    except Exception as e:
        print(f"[SYSTEM FAILURE] Main runtime error tripped: {e}")
    finally:
        emergency_shutdown_handler(None, None)

# --- FLASK JPEGMOTION WEB SERVER PIPELINES ---
def generate_frames():
    global output_frame, output_frame_lock
    while not global_shutdown_event.is_set():
        if not STREAM_VIDEO:
            time.sleep(0.2)
            continue
            
        local_frame = None
        with output_frame_lock:
            if output_frame is not None:
                local_frame = output_frame.copy()
        
        if local_frame is None:
            time.sleep(0.03)
            continue

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        flag, encoded_image = cv2.imencode(".jpg", local_frame, encode_params)
        if not flag:
            continue
            
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')
        time.sleep(0.03)

@app.route("/")
def index():
    return "<h3>WRO 2026 Live Camera Server Active</h3><img src='/video_feed' width='100%'/>"

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    cv2.setNumThreads(2)

    print("--- Booting WRO 2026 Unified Obstacle Round System ---")
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)