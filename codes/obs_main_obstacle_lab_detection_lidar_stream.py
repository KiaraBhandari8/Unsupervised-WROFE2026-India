import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response, request, jsonify
import threading
import time
import os
import json
import signal

try:
    import serial
except ImportError:
    serial = None

# Import custom functions
try:
    from lidar_steering4sept import LidarScanner, PIDController, calculate_steering_error
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit()

# --- Global Shutdown Event ---
shutdown_event = threading.Event()

# --- Global Buffers & Locks ---
lidar_data_lock = threading.Lock()
latest_lidar_data = {}

camera_frame_lock = threading.Lock()
latest_processed_frames = {}

# Dictionary to hold the final rendered streams for the Flask web UI
display_lock = threading.Lock()
display_frames = {
    "original": None,
    "lidar": None,
    "track": None,
    "red": None,
    "green": None,
    "orange": None,
    "blue": None,
    "white": None
}

# Real-time telemetry dictionary for dashboard UI text blocks
telemetry_lock = threading.Lock()
lidar_telemetry = {"front": "--", "left": "--", "right": "--"}

# --- POWER / CPU BUDGET ---
CONTROL_LOOP_MAX_HZ = 30.0      
STARTUP_STAGE_DELAY_SEC = 2.0   

# --- Turn Counter & State Variables ---
turn_counter = 0
max_line_crossings = 12
OUT_PARKING_MANEUVER = False
START_PAUSE_DURATION = 5
DELAY_BETWEEN_TURNS = 7

# --- CONTROL CONSTANTS ---
SERVO_CENTER_ANGLE = 102
STEERING_GAIN = 0.1
ROBOT_MANEUVER_SPEED = 0.75
ROBOT_CRUISE_SPEED = 0.8

# --- CAMERA CONFIGURATION ---
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2   
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2  
LAB_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   
LAB_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  

# ===================== LAB COLOUR DETECTION =====================
PRESETS_FILE = "/home/pi8/wrofe2025/vision_presets.json"

DEFAULT_PRESETS = {
    "red":    {"l_min": 0,   "l_max": 255, "a_min": 155, "a_max": 255, "b_min": 100, "b_max": 145},
    "blue":   {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 140, "b_min": 0,   "b_max": 120},
    "green":  {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 115, "b_min": 130, "b_max": 200},
    "orange": {"l_min": 0,   "l_max": 255, "a_min": 135, "a_max": 255, "b_min": 150, "b_max": 255},
    "white":  {"l_min": 65,  "l_max": 255, "a_min": 110, "a_max": 145, "b_min": 110, "b_max": 145},
}

STREAM_COLORS = ["red", "green", "orange", "blue", "white"]

def load_presets():
    presets = {k: dict(v) for k, v in DEFAULT_PRESETS.items()}
    try:
        with open(PRESETS_FILE) as f:
            loaded = json.load(f)
        for color in STREAM_COLORS:
            if color in loaded:
                presets[color] = loaded[color]
        print(f"[LAB] Loaded presets from {PRESETS_FILE}")
    except Exception as e:
        print(f"[LAB] Could not load {PRESETS_FILE} ({e}); using defaults.")
    return presets

COLOR_PRESETS = load_presets()
presets_lock = threading.Lock()

LAB_MIN_CONTOUR_AREA = 900 
LAB_MIN_WIDTH = 30 
LAB_ROI_TOP_FRAC = 0.35          
COLOR_STEER_MAGNITUDE = 22       

# --- OBSTACLE AVOIDANCE CONSTANTS ---
AVOID_STANDOFF_MM = 250          
AVOID_CLEARANCE_MM = 100         
AVOID_PASS_CX_FRAC = 0.78        
REALIGN_DURATION_SEC = 0.8       
AVOID_WIDTH_TRIGGER = 50         
AVOID_DIST_GAIN_REF_MM = 350.0   

# --- DRIVABLE-AREA GATING ---
WHITE_FLOOR_BAND_PX = 24         
WHITE_FLOOR_MIN_RATIO = 0.30     
TRACK_MASK_CLOSE_PX = 25         
TRACK_MIN_AREA_FRAC = 0.02       
TRACK_BASE_DILATE_PX = 12        
MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)

def build_track_mask(white_mask):
    h, w = white_mask.shape[:2]
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TRACK_MASK_CLOSE_PX, TRACK_MASK_CLOSE_PX))
    closed = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, k, iterations=1)

    num, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    if num <= 1:
        return np.zeros_like(white_mask)

    min_area = TRACK_MIN_AREA_FRAC * (h * w)
    best_label, best_area = 0, 0
    for lbl in range(1, num):
        area = stats[lbl, cv2.CC_STAT_AREA]
        if area < min_area:
            continue
        top = stats[lbl, cv2.CC_STAT_TOP]
        height = stats[lbl, cv2.CC_STAT_HEIGHT]
        reaches_bottom = (top + height) >= (h - 2)
        if reaches_bottom and area > best_area:
            best_label, best_area = lbl, area

    if best_label == 0:
        return np.zeros_like(white_mask)

    track = np.where(labels == best_label, 255, 0).astype(np.uint8)
    if TRACK_BASE_DILATE_PX > 0:
        dk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TRACK_BASE_DILATE_PX, TRACK_BASE_DILATE_PX))
        track = cv2.dilate(track, dk, iterations=1)
    return track

def lab_mask(lab_frame, color):
    with presets_lock:
        p = dict(COLOR_PRESETS[color])
    mask = cv2.inRange(
        lab_frame,
        np.array([p["l_min"], p["a_min"], p["b_min"]]),
        np.array([p["l_max"], p["a_max"], p["b_max"]]),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)
    return mask

def _on_drivable_area(track_mask, bbox):
    x1, y1, x2, y2 = bbox
    h, w = track_mask.shape[:2]
    if cv2.countNonZero(track_mask) == 0:
        return False

    by1 = min(y2, h)
    by2 = min(y2 + WHITE_FLOOR_BAND_PX, h)
    bx1 = max(0, x1)
    bx2 = min(x2, w)
    
    if (by2 - by1) < 2 or (bx2 - bx1) < 2:
        cx = int(np.clip((x1 + x2) // 2, 0, w - 1))
        cy = int(np.clip(y2 - 1, 0, h - 1))
        return track_mask[cy, cx] != 0
    band = track_mask[by1:by2, bx1:bx2]
    ratio = cv2.countNonZero(band) / band.size
    return ratio >= WHITE_FLOOR_MIN_RATIO

def detect_lab_pillars(lab_frame, track_mask):
    frame_h = lab_frame.shape[0]
    roi_top = int(frame_h * LAB_ROI_TOP_FRAC)
    dets = []
    for cls in ("red", "green"):
        mask = lab_mask(lab_frame, cls)
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in conts:
            ar = cv2.contourArea(c)
            if ar > LAB_MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                if w > LAB_MIN_WIDTH:
                    bbox = [x, y, x + w, y + h]
                    if (y + h) < roi_top:
                        continue
                    if not _on_drivable_area(track_mask, bbox):
                        continue  
                    dets.append({
                        'class': cls,
                        'area': float(ar),
                        'bbox': bbox,
                        'width': w,
                        'cx': x + w // 2,
                    })
    return dets

avoid_state = "IDLE"        
avoid_color = None          
realign_end_time = 0.0

def _front_distance_mm(scan_data):
    if not scan_data:
        return None
    fronts = [d for a, d in scan_data.items() if -5 <= a <= 5 and d > 0]
    return float(np.median(fronts)) if fronts else None

def _avoid_servo_angle(color, front_dist, cx=None, lab_w=None):
    mag = COLOR_STEER_MAGNITUDE
    if front_dist and front_dist > 0:
        mag = COLOR_STEER_MAGNITUDE * max(0.6, min(1.8, AVOID_DIST_GAIN_REF_MM / front_dist))
    if cx is not None and lab_w is not None and lab_w > 0:
        if color == 'green':
            side_factor = max(0.0, 1.0 - 2.0 * cx / lab_w)
            mag *= (1.0 + side_factor)
        else:
            side_factor = max(0.0, 2.0 * cx / lab_w - 1.0)
            mag *= (1.0 + side_factor)
    return SERVO_CENTER_ANGLE - mag if color == 'green' else SERVO_CENTER_ANGLE + mag

def manage_color_avoidance(dets, scan_data, lab_w):
    global avoid_state, avoid_color, realign_end_time
    now = time.time()

    objs = [d for d in dets if d['class'] in ('red', 'green')]
    target = max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None
    front_dist = _front_distance_mm(scan_data)

    if avoid_state == "REALIGN":
        if now < realign_end_time:
            return True, SERVO_CENTER_ANGLE, f"realign_{avoid_color}"
        avoid_state, avoid_color = "IDLE", None

    if avoid_state == "TURNING":
        color = avoid_color
        passed = False
        if target is None or target['class'] != color:
            passed = True  
        else:
            cx = target['cx']
            if color == 'green' and cx > lab_w * AVOID_PASS_CX_FRAC:
                passed = True   
            elif color == 'red' and cx < lab_w * (1.0 - AVOID_PASS_CX_FRAC):
                passed = True   
        if passed:
            avoid_state = "REALIGN"
            realign_end_time = now + REALIGN_DURATION_SEC
            return True, SERVO_CENTER_ANGLE, f"realign_{color}"
        cx = target['cx'] if target else None
        return True, _avoid_servo_angle(color, front_dist, cx, lab_w), f"avoid_{color}"

    if target is not None:
        width_trigger = target['width'] >= AVOID_WIDTH_TRIGGER
        dist_trigger = (front_dist is not None and front_dist <= (AVOID_STANDOFF_MM + AVOID_CLEARANCE_MM))
        if width_trigger or dist_trigger:
            avoid_color = target['class']
            avoid_state = "TURNING"
            return True, _avoid_servo_angle(avoid_color, front_dist, target['cx'], lab_w), f"avoid_{avoid_color}"

    return False, None, "none"

def filter_blue_objects_lab(lab_frame):
    return lab_mask(lab_frame, 'blue')

def detect_color_binary(mask, threshold=4000):
    return cv2.countNonZero(mask) > threshold

def draw_detections(frame, dets, scale_x=1.0, scale_y=1.0):
    for d in dets:
        x1, y1, x2, y2 = d['bbox']
        x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
        y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
        clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
        cv2.putText(frame, d['class'], (x1, y1 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)

# --- ESP32 SERIAL ---
SERVO_CENTER = 95
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- LIDAR CONTROL CONSTANTS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 150
CLOCKWISE_WALL_FOLLOWING = True
LIDAR_PID_KP = 0.12
LIDAR_PID_KI = 0.002
LIDAR_PID_KD = 0.05
LIDAR_SERVO_MIN_ANGLE = 77
LIDAR_SERVO_MAX_ANGLE = 127
LIDAR_STEERING_SCALE_FACTOR = 0.25

LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 40
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180
LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -40
LIDAR_LEFT_SIDE_DISTANCE_MM = 180
LIDAR_SIDE_STEER_MAGNITUDE = 20

STREAM_VIDEO = True
STREAM_WIDTH = 480          
STREAM_MAX_FPS = 12         
STREAM_JPEG_QUALITY = 65    
LINE_CHECK_EVERY_N_LOOPS = 2

class RobotState:
    IMMINENT_COLLISION_AVOIDANCE = "IMMINENT_COLLISION_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    RED_AVOIDANCE = "RED_AVOIDANCE"
    GREEN_AVOIDANCE = "GREEN_AVOIDANCE"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    STOP = "STOP"
    INITIALIZING = "INITIALIZING"
    FALLBACK_STRAIGHT = "FALLBACK_STRAIGHT"

current_robot_state = RobotState.INITIALIZING

ser = None
ESP32_OK = False
if serial is not None:
    try:
        ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.1)
        time.sleep(2)
        ESP32_OK = True
        print(f"[SERIAL] ESP32 on {PI_TO_ESP_PORT}")
    except Exception as e:
        print(f"[SERIAL] Fail: {e}")

def cmd(angle, speed):
    if ser and ESP32_OK:
        packet = f"STR:{angle},SPD:{int(speed * 255)}\n"
        ser.write(packet.encode())
        ser.flush()

def stop_robot():
    if ser and ESP32_OK:
        ser.write(f"STR:{SERVO_CENTER},SPD:0\n".encode())
        ser.flush()

def map_lidar_steering_angle(center_angle, pid_output, clockwise=True):
    adjusted_output = -1 * pid_output * LIDAR_STEERING_SCALE_FACTOR
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    return max(LIDAR_SERVO_MIN_ANGLE, min(angle, LIDAR_SERVO_MAX_ANGLE))

def check_lidar_side_alerts(scan_data):
    if not scan_data: return None
    for angle, distance in scan_data.items():
        if LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_RIGHT_SIDE_DISTANCE_MM:
            return "RIGHT"
    for angle, distance in scan_data.items():
        if LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_LEFT_SIDE_DISTANCE_MM:
            return "LEFT"
    return None

def get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=2, speed=ROBOT_MANEUVER_SPEED):
    global CLOCKWISE_WALL_FOLLOWING
    end_time = time.time() + duration_sec
    if not scan_data:
        CLOCKWISE_WALL_FOLLOWING = True
    else:
        left_distances = [dist for angle, dist in scan_data.items() if -90 <= angle <= -40 and dist > 0]
        right_distances = [dist for angle, dist in scan_data.items() if 40 <= angle <= 90 and dist > 0]
        avg_left = np.mean(left_distances) if left_distances else 0
        avg_right = np.mean(right_distances) if right_distances else 0
        CLOCKWISE_WALL_FOLLOWING = False if avg_left > avg_right else True

    direction_multiplier = 1 if CLOCKWISE_WALL_FOLLOWING else -1
    servo_angle = SERVO_CENTER_ANGLE + (direction_multiplier * max_angle_magnitude)
    while time.time() < end_time and not shutdown_event.is_set():
        cmd(servo_angle, speed)
        time.sleep(0.05)
    stop_robot()
    time.sleep(0.5)

def check_imminent_collision_and_get_escape_route(scan_data):
    if not scan_data: return None
    is_collision_imminent = False
    for angle, distance in scan_data.items():
        if -10 <= angle <= 10 and 0 < distance < 100:
            is_collision_imminent = True
            break
    if not is_collision_imminent: return None
    left_distances = [d for a, d in scan_data.items() if -90 <= a < 0 and d > 0]
    right_distances = [d for a, d in scan_data.items() if 0 < a <= 90 and d > 0]
    return "LEFT" if np.mean(left_distances or [0]) > np.mean(right_distances or [0]) else "RIGHT"

# --- 2D LiDAR Top-Down Map Generator ---
def generate_lidar_visual_map(scan_data):
    canvas_size = 400
    map_frame = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
    center = canvas_size // 2
    cv2.circle(map_frame, (center, center), 50, (40, 40, 40), 1)
    cv2.circle(map_frame, (center, center), 100, (70, 70, 70), 1)
    
    if not scan_data:
        return map_frame

    for angle_deg, distance_mm in scan_data.items():
        if distance_mm <= 0: continue
        scale_factor = 0.05
        dist_px = distance_mm * scale_factor
        angle_rad = np.radians(angle_deg)
        x = int(center + dist_px * np.sin(angle_rad))
        y = int(center - dist_px * np.cos(angle_rad))
        
        if 0 <= x < canvas_size and 0 <= y < canvas_size:
            color = (0, 255, 255) if -5 <= angle_deg <= 5 else (0, 200, 0)
            cv2.circle(map_frame, (x, y), 2, color, -1)
            
    cv2.line(map_frame, (center - 10, center), (center + 10, center), (0, 0, 255), 1)
    cv2.line(map_frame, (center, center - 10), (center, center + 10), (0, 0, 255), 1)
    return map_frame

# --- LiDAR Data Thread ---
def lidar_acquisition_thread_func(scanner_instance, shutdown_event):
    global latest_lidar_data, lidar_data_lock
    print("LiDAR acquisition thread started.")
    try:
        while not shutdown_event.is_set():
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data
            time.sleep(0.01)
    except Exception as e:
        print(f"LiDAR Acquisition Thread Error: {e}")
    finally:
        print("LiDAR acquisition thread stopping.")

# --- Camera Data Thread ---
def camera_acquisition_thread_func(picam2_instance, shutdown_event, lab_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition and processing thread started.")
    frame_seq = 0
    try:
        while not shutdown_event.is_set():
            yuv420 = picam2_instance.capture_array("lores")
            capture_ts = time.time()
            frame_bgr = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)
            lab_source_frame = cv2.resize(frame_bgr, lab_processing_size, interpolation=cv2.INTER_AREA)
            lab_frame = cv2.cvtColor(lab_source_frame, cv2.COLOR_BGR2LAB)

            frame_seq += 1
            with camera_frame_lock:
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['lab'] = lab_frame
                latest_processed_frames['seq'] = frame_seq
                latest_processed_frames['ts'] = capture_ts

            time.sleep(max(0.005, (1.0 / CAMERA_FRAMERATE) - (time.time() - capture_ts)))
    except Exception as e:
        print(f"Camera Acquisition Thread Error: {e}")
    finally:
        print("Camera acquisition thread stopping.")

# --- Main Robot Control Loop ---
def robot_control_loop(shutdown_event):
    global current_robot_state, latest_processed_frames, latest_lidar_data
    global OUT_PARKING_MANEUVER, turn_counter, CLOCKWISE_WALL_FOLLOWING
    global display_frames, lidar_telemetry

    print("--- Starting Robot Control System (LAB detection) ---")
    print("Robot control thread started.")
    
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        lores={"size": (PROCESSING_WIDTH, PROCESSING_HEIGHT)},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started: main {CAMERA_RESOLUTION} (FOV reference), lores {(PROCESSING_WIDTH, PROCESSING_HEIGHT)} at {CAMERA_FRAMERATE} FPS.")
    
    lab_processing_size = (LAB_PROCESSING_WIDTH, LAB_PROCESSING_HEIGHT)
    camera_thread = threading.Thread(
        target=camera_acquisition_thread_func, 
        args=(picam2, shutdown_event, lab_processing_size)
    )
    camera_thread.daemon = True
    camera_thread.start()

    print(f"[POWER] Camera settled. Waiting {STARTUP_STAGE_DELAY_SEC}s before LiDAR spin-up...")
    time.sleep(STARTUP_STAGE_DELAY_SEC)

    lidar_scanner, lidar_pid, lidar_thread = None, None, None
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner, shutdown_event))
        lidar_thread.daemon = True
        lidar_thread.start()
        lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
        print("LiDAR system initialized successfully.")
    except Exception as e:
        print(f"WARNING: Failed to initialize LiDAR: {e}.")

    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    loop_counter = 0
    blue_count = 0
    prev_blue_state = False
    blue_cooldown_end_time = 0.0
    program_start_time = time.monotonic()
    loop_rate = 1.0 / CONTROL_LOOP_MAX_HZ

    try:
        while not shutdown_event.is_set():
            loop_start = time.time()
            loop_counter += 1

            bgr_frame = None
            lab_frame = None
            with camera_frame_lock:
                if 'bgr' in latest_processed_frames:
                    bgr_frame = latest_processed_frames['bgr'].copy()
                    lab_frame = latest_processed_frames['lab'].copy()

            scan_data = None
            with lidar_data_lock:
                if latest_lidar_data:
                    scan_data = latest_lidar_data.copy()

            if bgr_frame is None or lab_frame is None:
                time.sleep(0.01)
                continue

            # --- Update Telemetry Variables ---
            if scan_data:
                f_dist = _front_distance_mm(scan_data)
                l_dist = np.median([d for a, d in scan_data.items() if -75 <= a <= -40 and d > 0])
                r_dist = np.median([d for a, d in scan_data.items() if 40 <= a <= 75 and d > 0])
                with telemetry_lock:
                    lidar_telemetry["front"] = f"{int(f_dist)}mm" if f_dist else "--"
                    lidar_telemetry["left"] = f"{int(l_dist)}mm" if not np.isnan(l_dist) else "--"
                    lidar_telemetry["right"] = f"{int(r_dist)}mm" if not np.isnan(r_dist) else "--"

            white_mask = lab_mask(lab_frame, 'white')
            track_mask = build_track_mask(white_mask)
            dets = detect_lab_pillars(lab_frame, track_mask)

            # --- Parking Zone Exit ---
            if not OUT_PARKING_MANEUVER:
                if time.monotonic() - program_start_time < START_PAUSE_DURATION:
                    stop_robot()
                    current_robot_state = RobotState.INITIALIZING
                else:
                    get_out_of_parking_lot_maneuver(scan_data)
                    OUT_PARKING_MANEUVER = True
                continue

            # --- Lap Counter (Blue Line) ---
            if loop_counter % LINE_CHECK_EVERY_N_LOOPS == 0:
                blue_mask = filter_blue_objects_lab(lab_frame)
                is_blue = detect_color_binary(blue_mask, threshold=4000)
                now_mono = time.monotonic()
                if is_blue and not prev_blue_state and now_mono > blue_cooldown_end_time:
                    blue_count += 1
                    print(f"[{loop_counter}] Blue:{blue_count}/{max_line_crossings} Time:{time.monotonic()-program_start_time:.1f}s")
                    blue_cooldown_end_time = now_mono + DELAY_BETWEEN_TURNS
                prev_blue_state = is_blue

            # --- Navigation Core Logic ---
            escape_dir = check_imminent_collision_and_get_escape_route(scan_data)
            avoid_angle_label = "none"
            servo_angle = SERVO_CENTER_ANGLE

            if escape_dir:
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                servo_angle = SERVO_CENTER_ANGLE - 25 if escape_dir == "LEFT" else SERVO_CENTER_ANGLE + 25
                cmd(servo_angle, ROBOT_MANEUVER_SPEED)
            else:
                engaged, avoid_angle, avoid_label = manage_color_avoidance(dets, scan_data, LAB_PROCESSING_WIDTH)
                if engaged:
                    current_robot_state = RobotState.GREEN_AVOIDANCE if "green" in avoid_label else RobotState.RED_AVOIDANCE
                    servo_angle = avoid_angle
                    avoid_angle_label = avoid_label
                    cmd(servo_angle, ROBOT_CRUISE_SPEED)
                else:
                    side_alert = check_lidar_side_alerts(scan_data)
                    if side_alert:
                        current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                        servo_angle = (SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE if side_alert == "RIGHT" else SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE)
                        cmd(servo_angle, ROBOT_MANEUVER_SPEED)
                    elif lidar_scanner and scan_data:
                        current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                        try:
                            error = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, CLOCKWISE_WALL_FOLLOWING)
                            pid_output = lidar_pid.update(error)
                            servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, CLOCKWISE_WALL_FOLLOWING)
                            cmd(servo_angle, ROBOT_CRUISE_SPEED)
                        except Exception:
                            cmd(SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                    else:
                        current_robot_state = RobotState.FALLBACK_STRAIGHT
                        cmd(SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)

            # --- Print Loop State Metric Status ---
            if loop_counter % 3 == 0 or current_robot_state != RobotState.LIDAR_WALL_FOLLOWING:
                print(f"[{loop_counter}] {current_robot_state} | Angle:{int(servo_angle)} Speed:{ROBOT_CRUISE_SPEED} Det:LAB({len(dets)}) {avoid_angle_label}")

            # --- Render Visual Frames ---
            if STREAM_VIDEO:
                vis_frame = bgr_frame.copy()
                draw_detections(vis_frame, dets, PROCESSING_WIDTH / LAB_PROCESSING_WIDTH, PROCESSING_HEIGHT / LAB_PROCESSING_HEIGHT)
                lidar_map = generate_lidar_visual_map(scan_data)
                
                with display_lock:
                    display_frames["original"] = vis_frame
                    display_frames["lidar"] = lidar_map
                    display_frames["track"] = track_mask
                    display_frames["red"] = lab_mask(lab_frame, 'red')
                    display_frames["green"] = lab_mask(lab_frame, 'green')
                    display_frames["orange"] = lab_mask(lab_frame, 'orange')
                    display_frames["blue"] = lab_mask(lab_frame, 'blue')
                    display_frames["white"] = white_mask

            if blue_count >= max_line_crossings:
                current_robot_state = RobotState.STOP
                print("[STOP] Final Lap Completed Successfully.")
                stop_robot()
                break

            elapsed = time.time() - loop_start
            time.sleep(max(0.001, loop_rate - elapsed))
            
    finally:
        stop_robot()
        picam2.stop()
        if lidar_scanner: lidar_scanner.disconnect()

# ===================== FLASK SERVER ROUTES =====================
app = Flask(__name__)
app.config["TEMPLATES_AUTO_RELOAD"] = True

def generate_mjpeg_stream(frame_key):
    while not shutdown_event.is_set():
        with display_lock:
            frame = display_frames.get(frame_key)
            if frame is not None:
                frame = frame.copy()
        
        if frame is not None:
            if len(frame.shape) == 2:  
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            frame_res = cv2.resize(frame, (STREAM_WIDTH, int(STREAM_WIDTH * (frame.shape[0]/frame.shape[1]))))
            ret, jpeg = cv2.imencode('.jpg', frame_res, [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_JPEG_QUALITY])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
        time.sleep(1.0 / STREAM_MAX_FPS)

@app.route('/')
def index():
    return render_template('obs_lab.html')

# Endpoint handles color pipelines (/video/original, /video/red, etc.)
@app.route('/video/<channel>')
def stream_channel(channel):
    if channel in display_frames:
        return Response(generate_mjpeg_stream(channel), mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Invalid Stream Channel", 404

# FIXED: Fallback endpoint explicitly maps browser requests directly to the map matrix
@app.route('/get_lidar')
@app.route('/video/lidar')
def stream_lidar():
    return Response(generate_mjpeg_stream('lidar'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
@app.route('/get_stream')
def telemetry():
    with telemetry_lock:
        data = dict(lidar_telemetry)
    data["state"] = current_robot_state
    return jsonify(data)

def sigint_handler(signal, frame):
    print("\n[SHUTDOWN] Signal interrupt intercepted. Exiting safely...")
    shutdown_event.set()

if __name__ == '__main__':
    print("Web server starting. Open http://raspberrypi.local:5000")
    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)

    control_thread = threading.Thread(target=robot_control_loop, args=(shutdown_event,))
    control_thread.daemon = True
    control_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)