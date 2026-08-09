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
import signal

# --- IMPORT CUSTOM VISION, LIDAR, AND FGM EXTENSIONS ---
try:
    from image_frame_combine_outer_inner_depth1 import process_frame_for_steering, draw_debug_overlay
    from lidar_steering_new import LidarScanner
    from follow_the_gap import compute_fgm_steering
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to mount local tracking components: {e}")
    sys.exit(1)

# --- GLOBAL SHUTDOWN SYSTEM TRACKERS ---
global_shutdown_event = threading.Event()
esp_ser = None
lidar_scanner = None
picam2 = None

# --- LIDAR SCAN WINDOWS ---
# Kept only for the lightweight corner-crossing BOOKKEEPING tripwire (turn
# counting / lap termination) and the rear-angle stand-in -- NOT used for
# steering anymore. All steering now comes from compute_fgm_steering().
FRONT_SCAN_ANGLE_DEG = 15
LEFT_SCAN_ANGLES = range(-105, -75)
RIGHT_SCAN_ANGLES = range(75, 105)

# --- CORNER BOOKKEEPING TRIPWIRE (counting only -- does NOT stop/steer the robot) ---
# This replaces the old CORNER_DETECT_STOP -> ... -> CORNER_ALIGN_BACKWARD
# sequence entirely. It exists only to (a) know which wall to bias FGM's
# progress term toward and (b) count corners for the 12-turn lap
# termination. It never sends a servo/speed command and never blocks the
# loop -- FGM is steering every single tick regardless of this tripwire's
# state.
CORNER_DETECT_FRONT_MM = 1000.0
CORNER_DETECT_NEAR_SIDE_MM = 900.0
CORNER_DETECT_FAR_SIDE_MM = 1500.0
CORNER_DETECTION_COOLDOWN_SEC = 3.0  # shorter than before -- FGM doesn't need a long recovery window

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
SERVO_CENTER_ANGLE = 95
ROBOT_CRUISE_SPEED = 160
ROBOT_MANEUVER_SPEED = 150   # used when FGM steer angle is large / obstacle close
ROBOT_CAUTION_SPEED = 110    # used in the rare no-gap-found fallback tick

# --- FGM -> SERVO MAPPING ---
# steer_angle_deg from compute_fgm_steering(): 0 = straight, negative = left,
# positive = right (matches existing LiDAR convention). Bench-tune the gain
# first -- it is the single most important number in this file.
FGM_SERVO_GAIN = 1.0
FGM_STEER_CLIP_DEG = 40.0   # max +/- steer angle FGM is allowed to command
MANEUVER_STEER_THRESHOLD_DEG = 15.0   # beyond this, drop to maneuver speed
MANEUVER_MIN_DIST_MM = 500.0          # closer than this, drop to maneuver speed

# --- PROGRESS BIAS (mild nudge toward the wall-following side, used only
# as a tie-breaker inside FGM's gap scoring -- see PROGRESS_BIAS_WEIGHT in
# follow_the_gap.py, which keeps this from ever overriding safety) ---
FGM_PROGRESS_BIAS_DEG = 12.0

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


class RobotState:
    INITIALIZING = "INITIALIZING"
    FGM_NAVIGATION = "FGM_NAVIGATION"   # single steady-state: FGM drives everywhere, always
    FGM_FALLBACK = "FGM_FALLBACK"       # rare: no gap found this tick, caution creep
    LAP_TERMINATION = "LAP_TERMINATION"
    STOP = "STOP"


current_robot_state = RobotState.INITIALIZING
current_yaw = 0.0
CLOCKWISE_WALL_FOLLOWING = None   # unknown until the first corner crossing is logged
last_fgm_steer_angle = 0.0        # fed back into compute_fgm_steering() for smoothing


def send_esp_packet(ser_port, steering, speed):
    if ser_port and ser_port.is_open and not global_shutdown_event.is_set():
        try:
            packet = f"STR:{steering},SPD:{speed}\n"
            ser_port.write(packet.encode('utf-8'))
        except Exception:
            pass


def emergency_shutdown_handler(signum, frame):
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


signal.signal(signal.SIGINT, emergency_shutdown_handler)
signal.signal(signal.SIGQUIT, emergency_shutdown_handler)


def filter_blue_objects(hsv_frame):
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    return cv2.dilate(mask, kernel, iterations=2)


def detect_color_binary(mask, threshold=4000):
    return cv2.countNonZero(mask) > threshold


def compute_front_left_right(scan_data):
    """Retained only for the corner-crossing bookkeeping tripwire below --
    no longer used for steering (FGM reads the raw scan_data directly)."""
    front_pts = [scan_data[a] for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
                 if a in scan_data and scan_data[a] > 0]
    left_pts = [scan_data[a] for a in LEFT_SCAN_ANGLES if a in scan_data and scan_data[a] > 0]
    right_pts = [scan_data[a] for a in RIGHT_SCAN_ANGLES if a in scan_data and scan_data[a] > 0]

    avg_front = sum(front_pts) / len(front_pts) if front_pts else 2000.0
    avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
    avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
    return avg_front, avg_left, avg_right


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


def robot_control_loop():
    global output_frame, output_frame_lock, current_robot_state, latest_processed_frames, camera_frame_lock
    global CLOCKWISE_WALL_FOLLOWING, current_yaw, esp_ser, lidar_scanner, picam2, last_fgm_steer_angle

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] High-speed serial connection established with ESP32 execution layer.")
    except Exception as e:
        print(f"[FATAL] Serial bridge initialization failed on {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

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

    try:
        lidar_scanner = LidarScanner(port='/dev/ttyUSB0', baudrate=230400)
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        print("[INFO] LiDAR scanner pipeline mounted safely.")
    except Exception as e:
        print(f"[WARN] LiDAR interface offline: {e}. FGM cannot run without LiDAR -- halting.")
        lidar_scanner = None

    current_robot_state = RobotState.FGM_NAVIGATION
    turn_count = 0
    corner_cooldown_end_time = 0.0

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
            # CORNER-CROSSING BOOKKEEPING TRIPWIRE (counting only)
            # No stop, no pivot, no state change to steering. FGM below
            # runs unconditionally every tick regardless of this.
            # ====================================================
            in_cooldown = time.monotonic() < corner_cooldown_end_time
            corner_signature_cw = (avg_front < CORNER_DETECT_FRONT_MM
                                    and avg_left < CORNER_DETECT_NEAR_SIDE_MM
                                    and avg_right > CORNER_DETECT_FAR_SIDE_MM)
            corner_signature_ccw = (avg_front < CORNER_DETECT_FRONT_MM
                                     and avg_right < CORNER_DETECT_NEAR_SIDE_MM
                                     and avg_left > CORNER_DETECT_FAR_SIDE_MM)

            if not in_cooldown and (corner_signature_cw or corner_signature_ccw):
                if CLOCKWISE_WALL_FOLLOWING is None:
                    CLOCKWISE_WALL_FOLLOWING = corner_signature_cw
                    print(f"[LAYOUT LOCKDOWN] Track direction set to: "
                          f"{'CLOCKWISE (CW)' if CLOCKWISE_WALL_FOLLOWING else 'COUNTER-CLOCKWISE (CCW)'}")
                turn_count += 1
                corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
                print(f"[CORNER LOGGED] Turn {turn_count}/12 (Front={avg_front:.0f}mm "
                      f"Left={avg_left:.0f}mm Right={avg_right:.0f}mm)")

            # ====================================================
            # VISION: run every tick -- FGM needs to know whether a
            # red/green obstacle is in view and exactly where, every
            # single frame, not just when LiDAR proximity triggers it.
            # ====================================================
            processed_frame, _vision_steer_unused, _mask, logic_label, vision_obstacle_info = \
                process_frame_for_steering(frame_bgr, use_outer_roi_and_bottom_point=is_near_field_mode)

            if logic_label not in ("red_obstacle", "obstacle"):
                vision_obstacle_info = None  # line_centering / corner_avoid / none are not real obstacles
            elif vision_obstacle_info is not None:
                # Required by _apply_vision_mask()'s mandatory pass-direction
                # enforcement -- green forces left, red forces right.
                vision_obstacle_info["logic_label"] = logic_label

            # ====================================================
            # PROGRESS BIAS: mild nudge toward the wall-following side,
            # only used as a gap-selection tie-breaker inside FGM --
            # never overrides safety (see PROGRESS_BIAS_WEIGHT).
            # ====================================================
            if CLOCKWISE_WALL_FOLLOWING is None:
                bias_angle_deg = 0.0
            elif CLOCKWISE_WALL_FOLLOWING:
                bias_angle_deg = FGM_PROGRESS_BIAS_DEG    # hug/lean right-ish through the loop
            else:
                bias_angle_deg = -FGM_PROGRESS_BIAS_DEG   # hug/lean left-ish through the loop

            # ====================================================
            # SINGLE STEERING SOURCE: Follow-The-Gap, every tick,
            # every part of the track. No corner state machine, no
            # LiDAR side-panic override, no priority chain.
            # ====================================================
            steer_angle_deg, fgm_debug = compute_fgm_steering(
                scan_data,
                vision_obstacle_info=vision_obstacle_info,
                bias_angle_deg=bias_angle_deg,
                prev_angle_deg=last_fgm_steer_angle,
            )
            last_fgm_steer_angle = steer_angle_deg

            if fgm_debug.get("fallback"):
                current_robot_state = RobotState.FGM_FALLBACK
                robot_speed_current = ROBOT_CAUTION_SPEED
                display_text = f"MODE: FGM Fallback (no gap) | Steer:{steer_angle_deg:.1f}deg"
            else:
                current_robot_state = RobotState.FGM_NAVIGATION
                close_obstacle = fgm_debug["bubble_min_dist"] < MANEUVER_MIN_DIST_MM
                sharp_turn = abs(steer_angle_deg) > MANEUVER_STEER_THRESHOLD_DEG
                robot_speed_current = ROBOT_MANEUVER_SPEED if (close_obstacle or sharp_turn) else ROBOT_CRUISE_SPEED
                display_text = (f"MODE: FGM | Steer:{steer_angle_deg:.1f}deg | "
                                f"GapW:{fgm_debug['chosen_gap']['width_deg']:.0f}deg | "
                                f"MinDist:{fgm_debug['bubble_min_dist']:.0f}mm")

            steer_angle_deg = float(np.clip(steer_angle_deg, -FGM_STEER_CLIP_DEG, FGM_STEER_CLIP_DEG))
            target_servo_angle = SERVO_CENTER_ANGLE + (steer_angle_deg * FGM_SERVO_GAIN)
            final_servo_angle = int(round(np.clip(
                target_servo_angle,
                SERVO_CENTER_ANGLE - FGM_STEER_CLIP_DEG,
                SERVO_CENTER_ANGLE + FGM_STEER_CLIP_DEG
            )))
            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

            # --- Debug overlay ---
            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0

            if DEBUG_UI_OVERLAYS:
                processed_frame = draw_debug_overlay(processed_frame, use_outer_roi_and_bottom_point=is_near_field_mode)
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}/12", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if STREAM_VIDEO:
                    with output_frame_lock:
                        output_frame = processed_frame.copy()

            time.sleep(0.02)

    except Exception as e:
        print(f"[SYSTEM FAILURE] Main runtime error tripped: {e}")
    finally:
        emergency_shutdown_handler(None, None)


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
    return "<h3>WRO 2026 Live Camera Server Active (FGM Navigation)</h3><img src='/video_feed' width='100%'/>"


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    print("--- Booting WRO 2026 Unified Obstacle Round System (Follow-The-Gap Navigation) ---")
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)