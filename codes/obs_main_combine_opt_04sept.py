import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response
import threading
import time
import os
import signal

try:
    import ncnn
    NCNN_OK = True
except ImportError:
    NCNN_OK = False
    print("[WARN] ncnn not available, YOLO disabled")

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

# --- Global Variables ---
output_frame = None
output_frame_lock = threading.Lock()

# Shared LiDAR data buffer and its lock
latest_lidar_data = {}

lidar_data_lock = threading.Lock()

# Shared buffer for the latest camera frame and its lock
latest_camera_frame = None
latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

# Shared YOLO detection results (updated by background thread)
latest_detections = []
latest_det_method = "NONE"
detection_lock = threading.Lock()


app = Flask(__name__)

# Shutdown event for graceful Ctrl+C
shutdown_event = threading.Event()

# Turn Counter Variables
turn_counter = 0
max_turn_count = 12
previous_increment_time = time.time()
START_PAUSE_DURATION = 5
DELAY_BETWEEN_TURNS = 7
OUT_PARKING_MANEUVER = False
straight_detected_time = 0.0
straight_override_duration = 1.5

# --- CONTROL CONSTANTS ---
SERVO_CENTER_ANGLE = 102
STEERING_GAIN = 0.1
ROBOT_MANEUVER_SPEED = 0.65
ROBOT_CRUISE_SPEED = 0.65
ROBOT_SPEED_MAX = 0.65


# --- CAMERA CONFIGURATION ---
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
HSV_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   # This will be 768
HSV_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  # This will be 432


# --- YOLO MODEL SETTINGS ---
YOLO_PARAM = '/home/pi8/wrofe2025/yolo_model/pillar_accurate/model.ncnn.param'
YOLO_BIN = '/home/pi8/wrofe2025/yolo_model/pillar_accurate/model.ncnn.bin'
YOLO_INPUT_SIZE = 640
YOLO_CONF_THRESHOLD = 0.20
YOLO_CLASSES = {0: 'red', 1: 'green'}
YOLO_PIXEL_STEERING_GAIN = 0.4
MIN_STEER_CONF = 0.25

# --- HSV FALLBACK ---
HSV_LOWER_RED1 = np.array([0, 100, 100])
HSV_UPPER_RED1 = np.array([10, 255, 255])
HSV_LOWER_RED2 = np.array([160, 100, 100])
HSV_UPPER_RED2 = np.array([180, 255, 255])
HSV_LOWER_GREEN = np.array([40, 50, 50])
HSV_UPPER_GREEN = np.array([80, 255, 255])
HSV_MIN_CONTOUR_AREA = 150
HSV_MIN_WIDTH = 25

# --- ESP32 SERIAL CONSTANTS ---
SERVO_CENTER = 95
SERVO_MIN = 77
SERVO_MAX = 127
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- LIDAR CONTROL CONSTANTS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 150
CLOCKWISE_WALL_FOLLOWING = True

# PID parameters (tuned for corridor centering with closest-wall distances)
LIDAR_PID_KP = 0.12
LIDAR_PID_KI = 0.002
LIDAR_PID_KD = 0.05

LIDAR_SERVO_MIN_ANGLE = 77
LIDAR_SERVO_MAX_ANGLE = 127
LIDAR_STEERING_SCALE_FACTOR = 0.25

# LiDAR side-check parameters
LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 40
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180
LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -40
LIDAR_LEFT_SIDE_DISTANCE_MM = 180
LIDAR_SIDE_STEER_MAGNITUDE = 20

# --- DEBUGGING AND UI ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

# --- BEHAVIOR STATES ---
class RobotState:
    IMMINENT_COLLISION_AVOIDANCE = "IMMINENT_COLLISION_AVOIDANCE" # <-- NEW STATE
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    RED_AVOIDANCE = "RED_AVOIDANCE"
    GREEN_AVOIDANCE = "GREEN_AVOIDANCE"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    STOP = "STOP"
    INITIALIZING = "INITIALIZING"
    FALLBACK_STRAIGHT = "FALLBACK_STRAIGHT"

current_robot_state = RobotState.INITIALIZING


def filter_blue_objects(hsv_frame):
    """Detects presence of blue using HSV masking."""
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)
    return blue_mask

def filter_orange_objects(hsv_frame):
    """Detects presence of orange using HSV masking."""
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    orange_mask = cv2.erode(orange_mask, kernel, iterations=2)
    orange_mask = cv2.dilate(orange_mask, kernel, iterations=2)
    return orange_mask

def detect_color_binary(mask, threshold=4000):
    """Returns True if color is present above a pixel threshold."""
    return cv2.countNonZero(mask) > threshold

# ===================== ESP32 SERIAL =====================
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
else:
    print("[SERIAL] pyserial not available")

def cmd(angle, speed):
    if ser and ESP32_OK:
        packet = f"STR:{angle},SPD:{int(speed * 255)}\n"
        ser.write(packet.encode())
        ser.flush()

def stop_robot():
    if ser and ESP32_OK:
        ser.write(f"STR:{SERVO_CENTER},SPD:0\n".encode())
        ser.flush()

def startup_test():
    if not (ser and ESP32_OK):
        print("[STARTUP] Skipped - no serial")
        return
    print("[STARTUP] Sweeping servo...")
    for angle in [SERVO_CENTER - 20, SERVO_CENTER, SERVO_CENTER + 20, SERVO_CENTER]:
        ser.write(f"STR:{angle},SPD:0\n".encode())
        ser.flush()
        time.sleep(0.3)
    print("[STARTUP] Motor pulse...")
    cmd(SERVO_CENTER, 0.4)
    time.sleep(0.4)
    stop_robot()
    print("[STARTUP] Done")

# ===================== YOLO DETECTION =====================
yolo_net = None

def load_yolo():
    global yolo_net
    if not NCNN_OK:
        print("[YOLO] ncnn not available")
        return False
    if not os.path.exists(YOLO_PARAM):
        print(f"[YOLO] Model not found: {YOLO_PARAM}")
        return False
    print("[YOLO] Loading model...")
    yolo_net = ncnn.Net()
    yolo_net.opt.num_threads = 4
    yolo_net.load_param(YOLO_PARAM)
    yolo_net.load_model(YOLO_BIN)
    print("[YOLO] Loaded!")
    return True

def detect_yolo(img):
    if yolo_net is None:
        return None
    h, w = img.shape[:2]
    scale = YOLO_INPUT_SIZE / max(h, w)
    nw, nh = int(w * scale), int(h * scale)
    r = cv2.resize(img, (nw, nh))
    p = np.zeros((YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, 3), dtype=np.uint8)
    dw = (YOLO_INPUT_SIZE - nw) // 2
    dh = (YOLO_INPUT_SIZE - nh) // 2
    p[dh:dh+nh, dw:dw+nw] = r
    mat = ncnn.Mat.from_pixels(p, ncnn.Mat.PixelType.PIXEL_BGR2RGB, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE)
    mat.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
    ex = yolo_net.create_extractor()
    ex.set_light_mode(True)
    ex.input("in0", mat)
    _, out = ex.extract("out0")
    output = np.array(out).reshape(6, 8400).T
    class_cols = output[:, 4:]
    max_scores = np.max(class_cols, axis=1)
    mask = max_scores >= YOLO_CONF_THRESHOLD
    if not np.any(mask):
        return []
    filtered = output[mask]
    scores = max_scores[mask]
    class_ids = np.argmax(filtered[:, 4:], axis=1)
    xc = filtered[:, 0]
    yc = filtered[:, 1]
    bw = filtered[:, 2]
    bh = filtered[:, 3]
    x1 = (xc - bw / 2 - dw) / scale
    y1 = (yc - bh / 2 - dh) / scale
    x2 = (xc + bw / 2 - dw) / scale
    y2 = (yc + bh / 2 - dh) / scale
    boxes = [[float(x1[i]), float(y1[i]), float(x2[i] - x1[i]), float(y2[i] - y1[i])] for i in range(len(x1))]
    indices = cv2.dnn.NMSBoxes(boxes, scores.tolist(), YOLO_CONF_THRESHOLD, 0.45)
    dets = []
    for idx in indices.flatten() if len(indices) > 0 else []:
        i = int(idx)
        dets.append({
            'class': YOLO_CLASSES[int(class_ids[i])],
            'conf': float(scores[i]),
            'bbox': [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
            'width': int(x2[i] - x1[i]),
            'cx': int((x1[i] + x2[i]) // 2)
        })
    return dets

def detect_hsv_fallback(hsv):
    red = cv2.inRange(hsv, HSV_LOWER_RED1, HSV_UPPER_RED1) + \
          cv2.inRange(hsv, HSV_LOWER_RED2, HSV_UPPER_RED2)
    green = cv2.inRange(hsv, HSV_LOWER_GREEN, HSV_UPPER_GREEN)
    dets = []
    for mask, cls in [(red, 'red'), (green, 'green')]:
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in conts:
            ar = cv2.contourArea(c)
            if ar > HSV_MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                if w > HSV_MIN_WIDTH:
                    roi = mask[y:y+h, x:x+w]
                    conf = cv2.countNonZero(roi) / (w * h) if w * h > 0 else 0
                    dets.append({'class': cls, 'conf': conf, 'bbox': [x, y, x+w, y+h],
                                 'width': w, 'cx': (x + x + w) // 2})
    return dets

def detect_pillars(frame_bgr):
    """Returns (dets, method) where dets is list of detections or empty list."""
    if yolo_net is not None:
        yolo_dets = detect_yolo(frame_bgr)
        if yolo_dets is not None and len(yolo_dets) > 0:
            return yolo_dets, "YOLO"
    return [], "NONE"

def calculate_steering_angle_from_color_and_lidar(pillar_color, lidar_distance_mm, fw, pillar_cx):
    if not lidar_distance_mm or lidar_distance_mm <= 0:
        target_x = int(fw * 0.85) if pillar_color == 'red' else int(fw * 0.15)
        error = target_x - pillar_cx
        return YOLO_PIXEL_STEERING_GAIN * error
    distance_gain = max(0.2, min(2.0, 1000.0 / lidar_distance_mm))
    target_x = int(fw * 0.85) if pillar_color == 'red' else int(fw * 0.15)
    pixel_error = target_x - pillar_cx
    pixel_steering = YOLO_PIXEL_STEERING_GAIN * pixel_error
    if pillar_color == 'red':
        base_steering = 15.0 * distance_gain
    else:
        base_steering = -15.0 * distance_gain
    return base_steering + (pixel_steering * 0.5)

def compute_vision_from_detections(dets, fw, scan_data=None):
    """Returns (vision_angle, logic_label) from detection list with optional LiDAR integration."""
    reds = [d for d in dets if d['class'] == 'red']
    greens = [d for d in dets if d['class'] == 'green']

    if reds:
        largest = max(reds, key=lambda d: d['width'])
        pX = largest['cx']
        lidar_distance = None
        if scan_data:
            front_distances = [d for a, d in scan_data.items() if -5 <= a <= 5 and d > 0]
            if front_distances:
                lidar_distance = np.median(front_distances)
        vision_angle = calculate_steering_angle_from_color_and_lidar('red', lidar_distance, fw, pX)
        return vision_angle, "red_obstacle"

    if greens:
        largest = max(greens, key=lambda d: d['width'])
        pX = largest['cx']
        lidar_distance = None
        if scan_data:
            front_distances = [d for a, d in scan_data.items() if -5 <= a <= 5 and d > 0]
            if front_distances:
                lidar_distance = np.median(front_distances)
        vision_angle = calculate_steering_angle_from_color_and_lidar('green', lidar_distance, fw, pX)
        return vision_angle, "obstacle"

    return 0, "none"

def draw_detections(frame, dets):
    for d in dets:
        x1, y1, x2, y2 = d['bbox']
        clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
        label = f"{d['class']}:{int(d['conf']*100)}%"
        cv2.putText(frame, label, (x1, y1-3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)

# --- Helper Functions ---
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

def check_front_obstacle_proximity(scan_data, distance_mm=1000):
    if not scan_data: return False
    for angle, dist in scan_data.items():
        if -2 <= angle <= 2 and 0 < dist < distance_mm:
            return True
    return False

def get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=2, speed=ROBOT_MANEUVER_SPEED):
    """
    Analyzes LiDAR data to choose the most open path (left or right),
    sets the global wall-following direction, and executes the turn.
    """
    global CLOCKWISE_WALL_FOLLOWING
    
    start_time = time.time()
    end_time = start_time + duration_sec
    
    if not scan_data:
        print("Parking Maneuver Warning: No LiDAR data. Defaulting to RIGHT turn.")
        CLOCKWISE_WALL_FOLLOWING = True
    else:
        # Define angle ranges for checking open space
        left_distances = [dist for angle, dist in scan_data.items() if -90 <= angle <= -40 and dist > 0]
        right_distances = [dist for angle, dist in scan_data.items() if 40 <= angle <= 90 and dist > 0]
        
        avg_left = np.mean(left_distances) if left_distances else 0
        avg_right = np.mean(right_distances) if right_distances else 0
        
        print(f"Parking Maneuver Analysis: Avg Left Space={avg_left:.0f}mm, Avg Right Space={avg_right:.0f}mm")
        
        # Decide direction based on which side has more open space
        if avg_left > avg_right:
            CLOCKWISE_WALL_FOLLOWING = False # More space on left -> turn left (anti-clockwise)
            print("Decision: Turning LEFT (Anti-Clockwise). Setting wall-following mode.")
        else:
            CLOCKWISE_WALL_FOLLOWING = True # More space on right -> turn right (clockwise)
            print("Decision: Turning RIGHT (Clockwise). Setting wall-following mode.")
    
    # Execute the turn based on the decision
    direction_multiplier = 1 if CLOCKWISE_WALL_FOLLOWING else -1
    servo_angle = SERVO_CENTER_ANGLE + (direction_multiplier * max_angle_magnitude)
    print(f"Executing escape maneuver with Servo Angle: {servo_angle}")

    while time.time() < end_time:
        cmd(servo_angle, speed)
        time.sleep(0.05)

    stop_robot()
    time.sleep(0.5)

# --- NEW FUNCTION FOR IMMINENT COLLISION AVOIDANCE ---
def check_imminent_collision_and_get_escape_route(scan_data):
    """
    Checks for an imminent forward collision and determines the best escape direction.
    Trigger: Any distance < 100mm in the -10 to +10 degree range.
    Logic: Compares average free space on the left vs. right to decide which way to turn.
    Returns: "LEFT", "RIGHT", or None.
    """
    if not scan_data:
        return None

    # 1. Check for the trigger condition (imminent collision)
    is_collision_imminent = False
    for angle, distance in scan_data.items():
        if -10 <= angle <= 10 and 0 < distance < 100:
            is_collision_imminent = True
            break
    
    if not is_collision_imminent:
        return None

    # 2. If triggered, calculate escape route
    left_distances = [d for a, d in scan_data.items() if -90 <= a < 0 and d > 0]
    right_distances = [d for a, d in scan_data.items() if 0 < a <= 90 and d > 0]

    # Calculate average distance, handling cases with no valid readings
    avg_left = np.mean(left_distances) if left_distances else 0
    avg_right = np.mean(right_distances) if right_distances else 0

    # 3. Decide direction based on which side has more open space
    if avg_left > avg_right:
        return "LEFT"  # More space on the left, so turn left
    else:
        return "RIGHT" # More space on the right (or they are equal), so turn right


def check_for_straight_corridor(scan_data, min_dist_mm=1000, max_dist_mm=3500, angle_range=10):
    if not scan_data:
        return False

    left_front_distances = []
    right_front_distances = []
    angle_range_max = angle_range
    for angle, distance in scan_data.items():
        if -1*angle_range_max <= angle < 0 and distance > 0:
            left_front_distances.append(distance)
        elif 0 <= angle <= angle_range_max and distance > 0:
            right_front_distances.append(distance)

    if not left_front_distances or not right_front_distances:
        return False

    avg_left_dist = sum(left_front_distances) / len(left_front_distances)
    avg_right_dist = sum(right_front_distances) / len(right_front_distances)
    
    is_left_in_range = min_dist_mm < avg_left_dist < max_dist_mm
    is_right_in_range = min_dist_mm < avg_right_dist < max_dist_mm

    return is_left_in_range and is_right_in_range

# --- LiDAR Data Acquisition Thread ---
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("LiDAR acquisition thread started.")
    try:
        while True:
            # Check if the main thread is still alive; if not, exit.
            if not any(t.name == 'MainThread' and t.is_alive() for t in threading.enumerate()):
                break
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data
            time.sleep(0.01)
    except Exception as e:
        print(f"LiDAR Acquisition Thread Error: {e}")
    finally:
        print("LiDAR acquisition thread stopping.")

# --- Camera Acquisition Thread ---
def camera_acquisition_thread_func(picam2_instance, stop_event, processing_size, hsv_processing_size): # <-- ADD hsv_processing_size
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition and processing thread started.")
    try:
        while not stop_event.is_set():
            # 1. Capture the single high-resolution frame
            captured_frame_rgb = picam2_instance.capture_array()

            # --- Main Processing Path ---
            # 2. Resize for main logic (e.g., lane following, display)
            processing_frame_rgb = cv2.resize(
                captured_frame_rgb,
                processing_size,
                interpolation=cv2.INTER_AREA
            )
            # 3. Convert to BGR for functions that require it
            frame_bgr = cv2.cvtColor(processing_frame_rgb, cv2.COLOR_RGB2BGR)

            # --- Color Detection Path ---
            # 4. Resize the *original* frame again to the smaller size for HSV
            hsv_source_frame = cv2.resize(
                captured_frame_rgb,
                hsv_processing_size, # <-- Use the new smaller size
                interpolation=cv2.INTER_AREA
            )
            # 5. Convert this smaller frame directly to HSV
            hsv_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_RGB2HSV)

            # 6. Store all three prepared frames in the global dictionary
            with camera_frame_lock:
                latest_processed_frames['rgb'] = processing_frame_rgb
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['hsv'] = hsv_frame # <-- ADD the new HSV frame

    except Exception as e:
        print(f"Camera Acquisition Thread Error: {e}")
    finally:
        print("Camera acquisition thread stopping.")

# --- YOLO Detection Thread ---
def detection_thread_func(stop_event):
    global latest_detections, latest_det_method, detection_lock
    print("YOLO detection thread started.")
    det_counter = 0
    while not stop_event.is_set():
        with camera_frame_lock:
            if 'bgr' not in latest_processed_frames:
                time.sleep(0.01)
                continue
            frame = latest_processed_frames['bgr'].copy()
        dets, method = detect_pillars(frame)
        with detection_lock:
            latest_detections = dets
            latest_det_method = method
        det_counter += 1
        if det_counter % 10 == 0:
            for d in dets:
                print(f"[{method}] {d['class']} conf={d['conf']:.2f} w={d['width']}")
        time.sleep(0.01)
    print("YOLO detection thread stopping.")

# --- Main Robot Control Loop ---
def robot_control_loop(shutdown_event):
    global output_frame, output_frame_lock, current_robot_state, latest_camera_frame, camera_frame_lock, camera_thread_stop_event
    global straight_detected_time, OUT_PARKING_MANEUVER, START_PAUSE_DURATION, previous_increment_time, turn_counter, max_turn_count, DELAY_BETWEEN_TURNS
    global ROBOT_SPEED_MAX, ROBOT_MANEUVER_SPEED, ROBOT_CRUISE_SPEED
    global CLOCKWISE_WALL_FOLLOWING

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started with resolution {CAMERA_RESOLUTION} at {CAMERA_FRAMERATE} FPS.")
    
    time.sleep(1) 
    processing_size = (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    hsv_processing_size = (HSV_PROCESSING_WIDTH, HSV_PROCESSING_HEIGHT) # <-- NEW

    # Pass both sizes to the thread
    camera_acquisition_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, processing_size, hsv_processing_size) # <-- MODIFIED
    )
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    lidar_scanner, lidar_pid, lidar_acquisition_thread = None, None, None
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
        print("LiDAR system initialized successfully.")
    except (IOError, Exception) as e:
        print(f"WARNING: Failed to initialize LiDAR system: {e}.")
        lidar_scanner = None

    yolo_ok = load_yolo()
    startup_test()
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    try:
        first_loop = True
        straight_corridor_detected = False
        # --- NEW: Line crossing counters and states ---
        blue_count = 0
        orange_count = 0
        max_line_crossings = 12
        prev_blue_state = False
        prev_orange_state = False
        blue_cooldown_end_time = 0
        orange_cooldown_end_time = 0
        loop_counter = 0
        program_start_time = time.monotonic()
        out_direction = None
        crossed_12 = False
        crossed_time = 0
        print_timer = 0
        # Start YOLO detection thread
        detection_thread_stop_event = threading.Event()
        detection_thread = threading.Thread(
            target=detection_thread_func,
            args=(detection_thread_stop_event,)
        )
        detection_thread.daemon = True
        detection_thread.start()

        while not shutdown_event.is_set():
            loop_start_time = time.monotonic()
            loop_counter += 1
            
            with camera_frame_lock:
                if not latest_processed_frames: # <-- CHANGE THIS LINE
                    time.sleep(0.01) 
                    # The print statement below is optional but good for debugging startup
                    # print("Skipping loop: Waiting for first processed frame...")
                    continue
                
                # Now that we know the dictionary is not empty, grab the frames
                frame_bgr = latest_processed_frames['bgr'].copy()
                hsv = latest_processed_frames['hsv'].copy()
            
            scan_data = None
            if lidar_scanner:
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            if first_loop and OUT_PARKING_MANEUVER:
                print("Executing parking lot escape maneuver...")
                out_direction = get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=1.25, speed=0.5)
                first_loop = True
                OUT_PARKING_MANEUVER = True
                # break

            if loop_counter % 1 == 0:
                current_time = time.time()
                # Color Detection
                blue_mask = filter_blue_objects(hsv)
                orange_mask = filter_orange_objects(hsv)
                blue_in_view = detect_color_binary(blue_mask)
                orange_in_view = detect_color_binary(orange_mask)

                # Blue Line Binary Logic with Cooldown
                if not blue_in_view and prev_blue_state:
                    if current_time > blue_cooldown_end_time:
                        blue_count += 1
                        print(f"Blue line crossed! Total blue lines: {blue_count}")
                        blue_cooldown_end_time = current_time + 6
                prev_blue_state = blue_in_view

                # # Orange Line Binary Logic with Cooldown
                # if not orange_in_view and prev_orange_state:
                #     if current_time > orange_cooldown_end_time:
                #         orange_count += 1
                #         print(f"Orange line crossed! Total orange lines: {orange_count}")
                #         orange_cooldown_end_time = current_time + 7
                # prev_orange_state = orange_in_view
            
            if loop_counter % 3 == 0 and time.time() - print_timer >= 0.5:
                print_timer = time.time()
                elapsed_time = time.monotonic() - program_start_time
                print(f"[{loop_counter}] Blue:{blue_count}/{max_line_crossings} Time:{elapsed_time:.1f}s")

            # --- NEW: STOPPING LOGIC BASED ON LINE COUNT ---
            if not crossed_12 and blue_count >= max_line_crossings:
                print(f"Max line count ({max_line_crossings}) reached for blue")
                crossed_12 = True
                crossed_time = time.time()
                blue_count = 0
                # break # Exit the main while loop

            if crossed_12 and (time.time() - crossed_time) > 4:
                stop_robot()
                time.sleep(120)
                break # Exit the while loop

            # between_walls = check_between_walls(scan_data, front_distance_min_threshold=1000, front_distance_max_threshold=2000,
            #     front_angle_range=7.5, side_angle_range=7.5, side_distance_threshold=1000, side_distance_tolerance=100)
    
            # if between_walls:
            #     if time.time() - loop_start_time > START_PAUSE_DURATION:
            #         if time.time() - previous_increment_time > DELAY_BETWEEN_TURNS:
            #             turn_counter += 1
            #             previous_increment_time = time.time()
            #             print(f"Turn condition met! Executing turn {turn_counter}/{max_turn_count}.")

            # print(f"--------Current turn count: {turn_counter}/{max_turn_count}--------")

            # if turn_counter > max_turn_count:
            #     print(f"Max turn count ({max_turn_count}) reached, stopping robot.")
            #     # stop_robot()
            #     # time.sleep(60)
            #     # break

            is_near_field_mode = check_front_obstacle_proximity(scan_data, distance_mm=1100)

            # --- Read latest YOLO detections from background thread ---
            with detection_lock:
                detections = latest_detections
                det_method = latest_det_method

            # Fast HSV fallback only when YOLO returns nothing
            if not detections:
                hsv_dets = detect_hsv_fallback(hsv)
                if hsv_dets:
                    scale_x = frame_bgr.shape[1] / hsv.shape[1]
                    scale_y = frame_bgr.shape[0] / hsv.shape[0]
                    for d in hsv_dets:
                        d['bbox'] = [int(d['bbox'][0] * scale_x), int(d['bbox'][1] * scale_y),
                                     int(d['bbox'][2] * scale_x), int(d['bbox'][3] * scale_y)]
                        d['width'] = int(d['width'] * scale_x)
                        d['cx'] = int(d['cx'] * scale_x)
                    detections = hsv_dets
                    det_method = "HSV"

            # Only steer if confidence > 25%
            detections = [d for d in detections if d['conf'] >= MIN_STEER_CONF]

            vision_angle, logic_label = compute_vision_from_detections(detections, frame_bgr.shape[1], scan_data)

            # Build display frame
            processed_frame = frame_bgr.copy()
            draw_detections(processed_frame, detections)
            label_method = f"[{det_method}]"
            
            # Get high-priority maneuver decisions before main logic
            escape_direction = check_imminent_collision_and_get_escape_route(scan_data)
            side_alert_status = check_lidar_side_alerts(scan_data)
            
            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # --- BEHAVIOR ARBITRATION ---
            # PRIORITY 0: IMMINENT COLLISION (always highest)
            if escape_direction == "LEFT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: ESCAPE LEFT!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("Imminent Collision: Escaping LEFT")
            elif escape_direction == "RIGHT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: ESCAPE RIGHT!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("Imminent Collision: Escaping RIGHT")

            # PRIORITY 1: SIDE OBSTACLE (LIDAR)
            elif side_alert_status == "RIGHT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Right LiDAR!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("LiDAR Side: RIGHT")
            elif side_alert_status == "LEFT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Left LiDAR!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("LiDAR Side: LEFT")

            # PRIORITY 2: CAMERA OBSTACLE (YOLO + HSV fallback)
            elif logic_label == "red_obstacle" or logic_label == "obstacle":
                robot_speed_current = ROBOT_MANEUVER_SPEED
                current_robot_state = RobotState.RED_AVOIDANCE if logic_label == 'red_obstacle' else RobotState.GREEN_AVOIDANCE
                servo_adjust = -vision_angle * STEERING_GAIN
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print(f"Vision: {round(vision_angle)} |Servo Adj: {round(servo_adjust)}")
                target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                display_text = f"MODE: {'Red' if logic_label == 'red_obstacle' else 'Cam'}Avoid {det_method} | Steer: {int(round(target_servo_angle))}°"

            # PRIORITY 2.5: NO DETECTION — GO STRAIGHT
            elif logic_label == "none":
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                robot_speed_current = ROBOT_CRUISE_SPEED
                display_text = "MODE: No Pillar | Straight"

            # PRIORITY 3: LIDAR WALL FOLLOWING
            elif lidar_scanner and lidar_pid:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                if scan_data:
                    # Straight corridor override (timer-based, keeps robot straight after entering a straight)
                    if straight_detected_time > 0 and (time.time() - straight_detected_time) < straight_override_duration:
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "MODE: LiDARWF | Straight Ovrd"
                        if time.time() - print_timer >= 0.5:
                            print_timer = time.time()
                            print("LiDAR: Straight override active")
                    elif check_for_straight_corridor(scan_data, min_dist_mm=1000, max_dist_mm=3500, angle_range=10):
                        straight_detected_time = time.time()
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "MODE: LiDARWF | Straight"
                        if time.time() - print_timer >= 0.5:
                            print_timer = time.time()
                            print("LiDAR: Straight corridor detected")
                    else:
                        straight_detected_time = 0.0
                        lidar_error = calculate_steering_error(
                            scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM,
                            clockwise=CLOCKWISE_WALL_FOLLOWING
                        )
                        if lidar_error == 9999.0:
                            stop_robot()
                            current_robot_state = RobotState.STOP
                            display_text = "MODE: STOP"
                            time.sleep(0.1)
                            continue
                        pid_output = lidar_pid.update(lidar_error)
                        target_servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=CLOCKWISE_WALL_FOLLOWING)
                        display_text = f"MODE: LiDARWF | Steer: {round(target_servo_angle)}° | Err: {lidar_error:.0f}mm"
                else:
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE
                    display_text = "MODE: Fallback (No LiDAR)"

            # PRIORITY 4: FALLBACK
            else:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                display_text = f"MODE: Fallback | Logic: {logic_label}"

            # APPLY ROBOT MOTION
            final_angle = SERVO_CENTER_ANGLE
            if current_robot_state != RobotState.STOP:
                # Determine clipping limits
                if check_front_obstacle_proximity(scan_data, distance_mm=150):
                    min_angle_limit = LIDAR_SERVO_MIN_ANGLE - 5
                    max_angle_limit = LIDAR_SERVO_MAX_ANGLE + 5
                else:
                    min_angle_limit = LIDAR_SERVO_MIN_ANGLE
                    max_angle_limit = LIDAR_SERVO_MAX_ANGLE

                final_angle = int(round(np.clip(target_servo_angle, min_angle_limit, max_angle_limit)))

                # Slow down on sharp turns, maintain cruise when near center
                deviation = abs(final_angle - SERVO_CENTER_ANGLE)
                if deviation > 15:
                    turn_speed = ROBOT_CRUISE_SPEED * 0.75
                    cmd(final_angle, turn_speed)
                else:
                    cmd(final_angle, robot_speed_current)
            else:
                stop_robot()

            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0

            # Throttled summary (~2 Hz)
            if time.time() - print_timer >= 0.5:
                print_timer = time.time()
                print(f"[{loop_counter}] {current_robot_state} | Angle:{final_angle} Speed:{robot_speed_current:.2f} FPS:{int(fps)} Det:{det_method}({len(detections)}) {logic_label}")
            # UI OVERLAYS
            if DEBUG_UI_OVERLAYS:
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(processed_frame, f"FPS: {int(fps)}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"Blue: {blue_count}/{max_line_crossings}, Orange: {orange_count}/{max_line_crossings}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
            if STREAM_VIDEO:
                with output_frame_lock:
                    output_frame = processed_frame.copy()

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down gracefully...")

    finally:
        print("Control loop ending. Cleaning up resources...")
        shutdown_event.set()
        camera_thread_stop_event.set()
        detection_thread_stop_event.set()
        if camera_acquisition_thread and camera_acquisition_thread.is_alive():
            camera_acquisition_thread.join(timeout=2)
        if detection_thread and detection_thread.is_alive():
            detection_thread.join(timeout=2)
        stop_robot()
        if lidar_scanner:
            print("Disconnecting LiDAR...")
            lidar_scanner.disconnect()
        try:
            picam2.stop()
        except:
            pass
        print("All resources released.")


# --- Flask Streaming Functions ---
def generate_frames():
    global output_frame, output_frame_lock
    while True:
        if not STREAM_VIDEO:
            time.sleep(0.5)
            continue
        with output_frame_lock:
            if output_frame is None:
                time.sleep(0.01)
                continue
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

# --- Main Execution Block ---
if __name__ == '__main__':
    print("--- Starting Robot Control System ---")
    stop_robot()
    time.sleep(0.5)

    control_thread = threading.Thread(target=robot_control_loop, args=(shutdown_event,))
    control_thread.start()
    print("Robot control thread started.")

    def handle_sigint(sig, frame):
        print("\nSIGINT received. Shutting down...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        hostname = os.uname()[1]
        print(f"Web server starting. Open http://{hostname}.local:5000 or http://<your_pi_ip>:5000")
    except AttributeError:
         import socket
         hostname = socket.gethostname()
         ip_address = socket.gethostbyname(hostname)
         print(f"Web server starting. Open http://{ip_address}:5000")

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass

    shutdown_event.set()
    print("Waiting for control thread to stop...")
    control_thread.join(timeout=5)
    stop_robot()
    print("Main application exiting.")