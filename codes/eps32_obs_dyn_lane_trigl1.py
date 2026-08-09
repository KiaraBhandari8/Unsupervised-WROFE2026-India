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
# NOTE: get_wall_parallel_error / get_wall_parallel_sector_stats are assumed to be
# ported into lidar_steering_new.py.
try:
    from image_frame_combine_outer_inner_depth1 import process_frame_for_steering, draw_debug_overlay
    from lidar_steering_new import (
        LidarScanner,
        PIDController,
        get_wall_parallel_error,
    )
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to mount local tracking components: {e}")
    sys.exit(1)

# --- GLOBAL SHUTDOWN SYSTEM TRACKERS ---
global_shutdown_event = threading.Event()  # Master termination trigger event flag
esp_ser = None                              # Global handle for ESP32 serial link
lidar_scanner = None                        # Global handle for LiDAR object
picam2 = None                               # Global handle for camera driver

# --- LIDAR SCAN WINDOWS (reused for tripwire, corner-detect, lane-detect, side auto-pick) ---
FRONT_SCAN_ANGLE_DEG = 15          # Width of front scan cone (+/- 15 degrees)
LEFT_SCAN_ANGLES = range(-105, -75)
RIGHT_SCAN_ANGLES = range(75, 105)
REAR_SCAN_ANGLES = list(range(165, 181)) + list(range(-180, -164))  # stand-in for rear ToF

# --- SIDE PANIC PARAMETERS ---
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180  # 18cm side distance panic limit
LIDAR_LEFT_SIDE_DISTANCE_MM = 180   # 18cm side distance panic limit
LIDAR_SIDE_STEER_MAGNITUDE = 15     # Fixed steering shift magnitude away from side walls

# --- CORNER DETECTION PARAMETERS ---
CORNER_DETECT_FRONT_MM = 1000.0     # Front distance that triggers "corner ahead" candidate check
CORNER_DETECT_NEAR_SIDE_MM = 900.0  # Inner (hugged) side must be closer than this
CORNER_DETECT_FAR_SIDE_MM = 1500.0  # Outer (opposite) side must be farther than this
CORNER_DETECT_STOP_DURATION = 0.5   # Seconds to stop + average readings before confirming
CORNER_SECOND_STOP_FRONT_MM = 550.0 # Front distance that triggers the actual pivot
CORNER_DETECTION_COOLDOWN_SEC = 8.0 # Safety guard so we don't re-trigger immediately after a turn

# --- LANE DETECTION PARAMETER ---
LANE_SIDE_THRESHOLD_MM = 600.0      # > this on the hugged side => lane 1 (needs backward); else lane 2/3 (skip backward)

# --- BACKWARD PHASE PARAMETERS (LiDAR rear-angle substitute for ToF) ---
CORNER_BACKWARD_TARGET_MM = 150.0
CORNER_BACKWARD_TOLERANCE_MM = 15.0
CORNER_BACKWARD_TIMEOUT_SEC = 3.0

# Hardware Servo Limits
LIDAR_SERVO_MIN_ANGLE = 10
LIDAR_SERVO_MAX_ANGLE = 170

# --- GLOBAL BUFFER LOCKS AND REGISTERS ---
output_frame = None
output_frame_lock = threading.Lock()

latest_lidar_data = {}
lidar_data_lock = threading.Lock()

latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

app = Flask(__name__)

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- CONTROL DESIGN CONSTANTS (8-BIT EXECUTION LAYER) ---
SERVO_CENTER_ANGLE = 95       # Absolute mechanical steering straight alignment midpoint
ROBOT_CRUISE_SPEED = 160      # Operational forward driving speed sent to ESP32 (0-255)
ROBOT_MANEUVER_SPEED = 150    # Slowdown velocity used across complex evasion arcs / backward
CORNER_PIVOT_SPEED = 100      # Speed used during the in-place gyro pivot

# --- INDEPENDENT VISION CALIBRATION PARAMETERS ---
STEERING_GAIN_GREEN = 0.1     # Baseline multiplier that keeps Green working perfectly
STEERING_GAIN_RED = 0.14      # INCREASE THIS to make the steering more aggressive for Red
RED_CLEARANCE_OFFSET = 8      # Static angular nudge (in degrees) to push the chassis wider right

# Gyro Turning Constants (still needed for the in-place pivot itself -- NOT for straight-line driving)
TURN_TARGET_DEGREES = 80.0
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0
CORNER_PIVOT_SAFETY_TIMEOUT = 3.0   # Safety guard so a stuck/noisy gyro can't spin forever

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
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"           # default cruising: parallel (triangle) wall-following, used before AND after the first corner
    VISION_OBSTACLE_AVOIDANCE = "VISION_OBSTACLE_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    CORNER_DETECT_STOP = "CORNER_DETECT_STOP"               # 0.5s stop + average to confirm corner + decide direction/lane
    CORNER_APPROACH_ALIGN = "CORNER_APPROACH_ALIGN"          # drive to the wall using parallel follow until front <= 550mm
    CORNER_ACTIVE_PIVOT = "CORNER_ACTIVE_PIVOT"              # gyro-yaw in-place pivot
    CORNER_ALIGN_BACKWARD = "CORNER_ALIGN_BACKWARD"          # conditional reverse, only when lane_needs_backward is True
    LAP_TERMINATION = "LAP_TERMINATION"
    STOP = "STOP"

current_robot_state = RobotState.INITIALIZING
current_yaw = 0.0
CLOCKWISE_WALL_FOLLOWING = None   # Unknown until the first corner is confirmed


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
        except Exception:
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


# --- SHARED LIDAR SECTOR HELPERS ---
def compute_front_left_right(scan_data):
    """Single place computing front/left/right averages -- reused by tripwire, corner-detect,
    lane-detect, and the pre-first-corner auto-side-pick logic."""
    front_pts = [scan_data[a] for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
                 if a in scan_data and scan_data[a] > 0]
    left_pts = [scan_data[a] for a in LEFT_SCAN_ANGLES if a in scan_data and scan_data[a] > 0]
    right_pts = [scan_data[a] for a in RIGHT_SCAN_ANGLES if a in scan_data and scan_data[a] > 0]

    avg_front = sum(front_pts) / len(front_pts) if front_pts else 2000.0
    avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
    avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
    return avg_front, avg_left, avg_right


def get_rear_distance(scan_data):
    """LiDAR-based stand-in for a rear ToF sensor -- this file has no ToF hardware."""
    pts = [scan_data[a] for a in REAR_SCAN_ANGLES if a in scan_data and scan_data[a] > 0]
    return sum(pts) / len(pts) if pts else None


def compute_parallel_steering(scan_data, side, pid):
    """Triangle-method parallel wall following. Returns (target_servo_angle, display_text)."""
    parallel_error = get_wall_parallel_error(scan_data, side)
    if parallel_error is None:
        # No wall detected on the followed side -- no gyro fallback,
        # so just hold the servo centered until a wall reappears.
        return SERVO_CENTER_ANGLE, f"MODE: Parallel Follow ({side}) | No wall - straight"

    normalized_error = parallel_error if side == "left" else -parallel_error
    pid_output = pid.update(normalized_error)
    target_servo_angle = SERVO_CENTER_ANGLE - pid_output
    return target_servo_angle, f"MODE: Parallel Follow ({side}) | Err: {parallel_error:.0f}mm"


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


# --- CAMERA ACQUISITION BACKGROUND TASK ---
def camera_acquisition_thread_func(picam2_instance, stop_event, processing_size, hsv_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("[SYSTEM] Camera thread active. Processing dual-resize frame array.")
    try:
        while not stop_event.is_set() and not global_shutdown_event.is_set():
            captured_frame_rgb = picam2_instance.capture_array()

            processing_frame_rgb = cv2.resize(captured_frame_rgb, processing_size, interpolation=cv2.INTER_AREA)
            frame_bgr = cv2.cvtColor(processing_frame_rgb, cv2.COLOR_RGB2BGR)

            hsv_source_frame = cv2.resize(captured_frame_rgb, hsv_processing_size, interpolation=cv2.INTER_AREA)
            hsv_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_RGB2HSV)

            with camera_frame_lock:
                latest_processed_frames['rgb'] = processing_frame_rgb
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['hsv'] = hsv_frame
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

    # Initialize Controller Mathematics Loop
    # NOTE: gyro_straight_pid removed entirely -- no gyro-based straight-line driving
    # remains anywhere in this file. Gyro/yaw is only used during the in-place pivot
    # to know when the turn is complete.
    parallel_pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.05, setpoint=0)  # Placeholder gains -- bench-tune before unattended use

    # Set Initial Behavioral States
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
    turn_count = 0
    baseline_start_yaw = 0.0
    turn_direction = None
    lane_needs_backward = False

    # Corner sequence bookkeeping
    corner_detect_samples = []
    corner_detect_stop_start_time = 0.0
    pivot_phase_start_time = 0.0
    backward_phase_start_time = 0.0
    corner_cooldown_end_time = 0.0

    # Blue Line Crossing Telemetry Registers
    blue_count = 0
    prev_blue_state = False
    blue_cooldown_end_time = 0.0

    print(f"[SYSTEM] Calibration complete. Initial State: {current_robot_state}")

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

            with camera_frame_lock:
                if not latest_processed_frames:
                    time.sleep(0.01)
                    continue
                frame_bgr = latest_processed_frames['bgr'].copy()
                hsv = latest_processed_frames['hsv'].copy()

            processed_frame = frame_bgr.copy()

            scan_data = {}
            if lidar_scanner:
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            avg_front, avg_left, avg_right = compute_front_left_right(scan_data)

            # Used by draw_debug_overlay() in BOTH debug blocks below, so the ROI
            # boxes stay visually consistent with whatever the vision module would
            # actually use if it ran this frame (near-field vs far-field ROI).
            is_near_field_mode = avg_front < 1100.0

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
            # PRIORITY LEVEL 1: ACTIVE CORNERING SEQUENCE
            # (CORNER_DETECT_STOP -> CORNER_APPROACH_ALIGN -> CORNER_ACTIVE_PIVOT -> [CORNER_ALIGN_BACKWARD])
            # ====================================================
            if current_robot_state in [RobotState.CORNER_DETECT_STOP, RobotState.CORNER_APPROACH_ALIGN,
                                        RobotState.CORNER_ACTIVE_PIVOT, RobotState.CORNER_ALIGN_BACKWARD]:

                display_text = ""

                # --- PHASE 1: Stop + average readings to confirm corner + decide direction + decide lane ---
                if current_robot_state == RobotState.CORNER_DETECT_STOP:
                    send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                    corner_detect_samples.append((avg_front, avg_left, avg_right))
                    elapsed_stop = time.monotonic() - corner_detect_stop_start_time
                    display_text = f"[CORNER] Confirming... {elapsed_stop:.2f}/{CORNER_DETECT_STOP_DURATION}s"

                    if elapsed_stop >= CORNER_DETECT_STOP_DURATION:
                        fronts = [s[0] for s in corner_detect_samples]
                        lefts = [s[1] for s in corner_detect_samples]
                        rights = [s[2] for s in corner_detect_samples]
                        mean_front = sum(fronts) / len(fronts)
                        mean_left = sum(lefts) / len(lefts)
                        mean_right = sum(rights) / len(rights)

                        if mean_left < mean_right:
                            turn_direction = "RIGHT"  # CW
                            if turn_count == 0:
                                CLOCKWISE_WALL_FOLLOWING = True
                                print("[LAYOUT LOCKDOWN] Track direction set to: CLOCKWISE (CW)")
                            lane_needs_backward = mean_left > LANE_SIDE_THRESHOLD_MM
                        else:
                            turn_direction = "LEFT"  # CCW
                            if turn_count == 0:
                                CLOCKWISE_WALL_FOLLOWING = False
                                print("[LAYOUT LOCKDOWN] Track direction set to: COUNTER-CLOCKWISE (CCW)")
                            lane_needs_backward = mean_right > LANE_SIDE_THRESHOLD_MM

                        print(f"[CORNER CONFIRMED] Dir={turn_direction} | AvgFront={mean_front:.0f}mm "
                              f"AvgLeft={mean_left:.0f}mm AvgRight={mean_right:.0f}mm | "
                              f"Lane needs backward: {lane_needs_backward}")

                        corner_detect_samples = []
                        current_robot_state = RobotState.CORNER_APPROACH_ALIGN
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)

                # --- PHASE 2: Drive to the wall using parallel (triangle) following until front <= 550mm ---
                elif current_robot_state == RobotState.CORNER_APPROACH_ALIGN:
                    follow_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    approach_servo_angle, align_text = compute_parallel_steering(scan_data, follow_side, parallel_pid)
                    display_text = f"[CORNER] Approach Align | {align_text} | Front:{avg_front:.0f}mm"

                    if avg_front <= CORNER_SECOND_STOP_FRONT_MM:
                        print(f"[CORNER] Second stop reached (front={avg_front:.0f}mm). Starting pivot...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(0.3)

                        final_servo_angle_lock = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                        send_esp_packet(esp_ser, final_servo_angle_lock, 0)
                        time.sleep(0.4)

                        print("[ACTION] Flushing scrub vibration error -> Resetting Gyro Yaw to 0deg...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)

                        current_yaw = 0.0
                        baseline_start_yaw = 0.0
                        pivot_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ACTIVE_PIVOT
                        send_esp_packet(esp_ser, final_servo_angle_lock, ROBOT_CRUISE_SPEED)
                    else:
                        send_esp_packet(esp_ser, approach_servo_angle, ROBOT_CRUISE_SPEED)

                # --- PHASE 3: In-place gyro pivot ---
                elif current_robot_state == RobotState.CORNER_ACTIVE_PIVOT:
                    yaw_delta = current_yaw - baseline_start_yaw
                    target_angle = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                    elapsed_pivot = time.monotonic() - pivot_phase_start_time
                    pivot_timed_out = elapsed_pivot >= CORNER_PIVOT_SAFETY_TIMEOUT

                    display_text = f"[CORNER] Pivot | Delta:{yaw_delta:.1f}/{TURN_TARGET_DEGREES}deg"

                    if abs(yaw_delta) >= TURN_TARGET_DEGREES or pivot_timed_out:
                        if pivot_timed_out and abs(yaw_delta) < TURN_TARGET_DEGREES:
                            print(f"[CORNER] WARNING: Pivot safety timeout hit before target yaw reached "
                                  f"(only {yaw_delta:+.1f}deg). Check gyro data.")
                        print(f"\n==========================================================")
                        print(f"[TARGET MET] Turn complete at total delta: {yaw_delta:.1f}deg")
                        print(f"==========================================================")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(0.4)

                        print("[ACTION] Clearing registers -> Setting Gyro baseline to 0deg...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.2)
                        current_yaw = 0.0
                        parallel_pid.reset()

                        if lane_needs_backward:
                            print("[CORNER] Lane 1 detected -> executing backward phase...")
                            backward_phase_start_time = time.monotonic()
                            current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                        else:
                            print("[CORNER] Lane 2/3 detected -> skipping backward phase.")
                            turn_count += 1
                            corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
                            turn_direction = None
                            current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                    else:
                        send_esp_packet(esp_ser, target_angle, CORNER_PIVOT_SPEED)

                # --- PHASE 4 (conditional): Reverse using LiDAR rear-angle distance as ToF substitute ---
                elif current_robot_state == RobotState.CORNER_ALIGN_BACKWARD:
                    rear_dist = get_rear_distance(scan_data)
                    elapsed_back = time.monotonic() - backward_phase_start_time
                    reached = rear_dist is not None and rear_dist <= (CORNER_BACKWARD_TARGET_MM + CORNER_BACKWARD_TOLERANCE_MM)
                    timed_out = elapsed_back >= CORNER_BACKWARD_TIMEOUT_SEC

                    display_text = (f"[CORNER] Backward | Rear:{rear_dist:.0f}mm"
                                     if rear_dist is not None else "[CORNER] Backward | Rear: N/A")

                    if reached or timed_out:
                        if timed_out and not reached:
                            print(f"[CORNER] WARNING: Backward safety timeout hit before target reached "
                                  f"(rear={rear_dist}). Check rear LiDAR coverage.")
                        else:
                            print(f"[CORNER] Backward target reached (rear={rear_dist:.0f}mm). Braking.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(0.3)

                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        turn_count += 1
                        corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
                        turn_direction = None
                        current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                    else:
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)

                # --- DEBUG BLOCK 1 of 2: fires for every cornering sub-state ---
                if DEBUG_UI_OVERLAYS:
                    # FIX: draw the ROI boxes unconditionally here too, so they don't
                    # vanish while the robot is in any cornering phase (previously this
                    # block only drew text, never the ROI rectangles/target lines).
                    processed_frame = draw_debug_overlay(processed_frame, use_outer_roi_and_bottom_point=is_near_field_mode)

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
            # PRIORITY LEVEL 2: CORNER SIGNATURE TRIPWIRE
            # ====================================================
            in_cooldown = time.monotonic() < corner_cooldown_end_time

            corner_signature_cw = (avg_front < CORNER_DETECT_FRONT_MM
                                    and avg_left < CORNER_DETECT_NEAR_SIDE_MM
                                    and avg_right > CORNER_DETECT_FAR_SIDE_MM)
            corner_signature_ccw = (avg_front < CORNER_DETECT_FRONT_MM
                                     and avg_right < CORNER_DETECT_NEAR_SIDE_MM
                                     and avg_left > CORNER_DETECT_FAR_SIDE_MM)

            if not in_cooldown and (corner_signature_cw or corner_signature_ccw):
                print(f"\n[CORNER DETECTED] Front={avg_front:.0f}mm Left={avg_left:.0f}mm "
                      f"Right={avg_right:.0f}mm -> stopping {CORNER_DETECT_STOP_DURATION}s to confirm...")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                corner_detect_samples = [(avg_front, avg_left, avg_right)]
                corner_detect_stop_start_time = time.monotonic()
                current_robot_state = RobotState.CORNER_DETECT_STOP
                continue

            # ====================================================
            # PRIORITY LEVEL 3: PROXIMITY CRITICAL WALL OVERRIDES
            # ====================================================
            right_side_panic = [scan_data[a] for a in range(40, 76) if a in scan_data and 0 < scan_data[a] < LIDAR_RIGHT_SIDE_DISTANCE_MM]
            left_side_panic = [scan_data[a] for a in range(-75, -39) if a in scan_data and 0 < scan_data[a] < LIDAR_LEFT_SIDE_DISTANCE_MM]

            if right_side_panic:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Right Wall Close"
            elif left_side_panic:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Left Wall Close"

            # ====================================================
            # PRIORITY LEVEL 4: COMPUTER VISION PILLAR AVOIDANCE
            # ====================================================
            else:
                processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                    frame_bgr, use_outer_roi_and_bottom_point=is_near_field_mode
                )
                vision_angle = -1 * vision_angle

                if logic_label in ["red_obstacle", "obstacle"]:
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED

                    if logic_label == "red_obstacle":
                        servo_adjust = -vision_angle * STEERING_GAIN_RED
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust + RED_CLEARANCE_OFFSET
                        display_text = f"MODE: Red Avoid | Steer: {int(target_servo_angle)}deg"
                    else:
                        servo_adjust = -vision_angle * STEERING_GAIN_GREEN
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                        display_text = f"MODE: Green Avoid | Steer: {int(target_servo_angle)}deg"

                # ====================================================
                # PRIORITY LEVEL 5: DEFAULT DRIVING -- PARALLEL (TRIANGLE) WALL FOLLOWING ONLY
                # No gyro-straight anywhere. Before the first corner (direction unknown),
                # auto-hug whichever side currently has the nearer wall.
                # ====================================================
                else:
                    robot_speed_current = ROBOT_CRUISE_SPEED
                    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING

                    if CLOCKWISE_WALL_FOLLOWING is not None:
                        follow_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    else:
                        follow_side = "left" if avg_left <= avg_right else "right"

                    target_servo_angle, display_text = compute_parallel_steering(scan_data, follow_side, parallel_pid)

                    # FIX: process_frame_for_steering() only draws the ROI overlay when
                    # it actually finds an obstacle branch to run through internally --
                    # when we land here (no obstacle, plain wall-following), processed_frame
                    # returned above is still the raw camera frame with NO overlay drawn.
                    # Draw it explicitly so the boxes don't disappear during normal cruising.
                    processed_frame = draw_debug_overlay(processed_frame, use_outer_roi_and_bottom_point=is_near_field_mode)

            # 5. Output packets to hardware layers
            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

            # Frame serving calculations
            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0

            # --- DEBUG BLOCK 2 of 2: fires for side-panic, vision, and default states ---
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

        flag, encoded_image = cv2.imencode(".jpg", local_frame)
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
    print("--- Booting WRO 2026 Unified Obstacle Round System (Modified Corner Logic) ---")
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)