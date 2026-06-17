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

# Shared YOLO detection results (updated by background thread).
# 'ts' is the CAPTURE timestamp of the frame the detections came from, so
# (now - ts) measures the full camera->inference->use latency.
# 'seq' is the camera frame sequence number that was processed.
latest_detection_packet = {'dets': [], 'method': 'NONE', 'ts': 0.0, 'seq': -1}
detection_lock = threading.Lock()

# --- POWER / CPU BUDGET (3A-friendly profile) ---
# The Pi 5's 5V rail sags under sustained 4-core load (measured 5.08V idle
# -> 4.91V with just a synthetic CPU stress); below ~4.8V the PMIC browns
# out and the Pi shuts off while the ESP32 keeps driving the motors.
# These values bound sustained current draw so the robot survives on a
# 5V/3A battery supply. On the official 27W (5V/5A) supply you can raise:
#   YOLO_THREADS = 3, YOLO_MAX_RATE_HZ = 6-8, CONTROL_LOOP_MAX_HZ = 50.
YOLO_THREADS = 2                # ncnn CPU threads (2 = 3A-safe, 3 = needs 5A)
YOLO_MAX_RATE_HZ = 4.0          # max inferences per second
CONTROL_LOOP_MAX_HZ = 30.0      # max control loop iterations per second
STARTUP_STAGE_DELAY_SEC = 2.0   # pause between subsystem bring-ups so the
                                # camera, LiDAR spin-up (USB inrush) and YOLO
                                # model load never stack on one current peak

# Detections older than this are considered stale and ignored for steering.
# Must stay comfortably above (1/YOLO_MAX_RATE_HZ + inference time) or every
# packet reads as stale and HSV fallback runs on every loop. At 4 Hz with
# ~0.3-0.4s inference on 2 threads, worst-case age at consumption is ~0.7s.
# Watch the "age:" value in the status print: if it constantly shows STALE,
# raise this or raise YOLO_MAX_RATE_HZ (needs the bigger supply).
DETECTION_MAX_AGE_SEC = 0.7


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
# main stays at the full-FOV binned sensor mode (wide angle preserved).
# lores is a hardware-scaled copy of the SAME full-FOV image, produced by the
# ISP at zero CPU cost. All processing reads lores; the full-res main stream
# is never copied into Python.
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2   # 1152 (lores stream size)
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2  # 648
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
# Set True only while practicing/tuning. Streaming costs a frame copy,
# overlay drawing and JPEG encode per frame -- and in competition rounds
# Wi-Fi must be off anyway (rule 11.10), so the stream is unusable there.
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = False

# Servo sweep + motor pulse on boot. Useful on the bench; skip in rounds
# (it burns ~2s of round time after the start button is pressed).
STARTUP_TEST_ENABLED = False

# Run blue/orange line masking every Nth control loop (it is only used for
# lap counting, which has a 6 s cooldown anyway -- no need to pay the
# morphology cost on every single iteration).
LINE_CHECK_EVERY_N_LOOPS = 2

# Pre-built kernel for the colour masks (was rebuilt on every call before)
MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)

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
    # Single fused open (erode->dilate) instead of separate calls
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)
    return blue_mask

def filter_orange_objects(hsv_frame):
    """Detects presence of orange using HSV masking."""
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)
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
    # Fewer threads = lower peak current draw and a core left free for the
    # camera/control/serial work. See the power budget block at the top.
    yolo_net.opt.num_threads = YOLO_THREADS
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
def camera_acquisition_thread_func(picam2_instance, stop_event, hsv_processing_size):
    """
    Captures the hardware-scaled lores stream (full wide-angle FOV, already
    downscaled by the ISP) and prepares the BGR + HSV frames. The full-res
    main stream is never pulled into Python, removing the 9 MB/frame copy
    and both CPU-side cv2.resize calls from full resolution.

    Each published frame set carries a monotonically increasing 'seq' and a
    capture timestamp 'ts' so consumers can detect new frames and measure age.
    """
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition and processing thread started.")
    frame_seq = 0
    try:
        while not stop_event.is_set():
            # 1. Capture the lores stream (YUV420 planar, processing size)
            yuv420 = picam2_instance.capture_array("lores")
            capture_ts = time.time()

            # 2. Convert YUV420 -> BGR (cheap at lores size)
            frame_bgr = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)

            # 3. Smaller HSV frame for line/colour masking
            hsv_source_frame = cv2.resize(
                frame_bgr,
                hsv_processing_size,
                interpolation=cv2.INTER_AREA
            )
            hsv_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_BGR2HSV)

            # 4. Publish the prepared frames with seq + timestamp
            frame_seq += 1
            with camera_frame_lock:
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['hsv'] = hsv_frame
                latest_processed_frames['seq'] = frame_seq
                latest_processed_frames['ts'] = capture_ts

    except Exception as e:
        print(f"Camera Acquisition Thread Error: {e}")
    finally:
        print("Camera acquisition thread stopping.")

# --- YOLO Detection Thread ---
def detection_thread_func(stop_event):
    """
    Runs YOLO only when a NEW camera frame is available (frame-freshness
    check via the camera 'seq' counter) instead of re-inferring on the same
    frame in a tight loop. Publishes results together with the source
    frame's capture timestamp so the control loop can reject stale results.
    """
    global latest_detection_packet
    print("YOLO detection thread started.")
    det_counter = 0
    last_processed_seq = -1
    min_inference_interval = 1.0 / YOLO_MAX_RATE_HZ
    last_inference_time = 0.0
    while not stop_event.is_set():
        # Rate cap: sustained back-to-back inference browns out the 5V rail.
        wait = min_inference_interval - (time.monotonic() - last_inference_time)
        if wait > 0:
            time.sleep(wait)

        frame = None
        with camera_frame_lock:
            seq = latest_processed_frames.get('seq', -1)
            if seq != last_processed_seq and 'bgr' in latest_processed_frames:
                frame = latest_processed_frames['bgr'].copy()
                frame_ts = latest_processed_frames['ts']
        if frame is None:
            # No new frame yet -- don't burn CPU re-running inference
            time.sleep(0.005)
            continue
        last_processed_seq = seq

        last_inference_time = time.monotonic()
        dets, method = detect_pillars(frame)
        with detection_lock:
            latest_detection_packet = {'dets': dets, 'method': method,
                                       'ts': frame_ts, 'seq': seq}
        det_counter += 1
        if det_counter % 10 == 0:
            for d in dets:
                print(f"[{method}] {d['class']} conf={d['conf']:.2f} w={d['width']}")
    print("YOLO detection thread stopping.")

# --- Main Robot Control Loop ---
def robot_control_loop(shutdown_event):
    global output_frame, output_frame_lock, current_robot_state, latest_camera_frame, camera_frame_lock, camera_thread_stop_event
    global straight_detected_time, OUT_PARKING_MANEUVER, START_PAUSE_DURATION, previous_increment_time, turn_counter, max_turn_count, DELAY_BETWEEN_TURNS
    global ROBOT_SPEED_MAX, ROBOT_MANEUVER_SPEED, ROBOT_CRUISE_SPEED
    global CLOCKWISE_WALL_FOLLOWING

    picam2 = Picamera2()
    # main keeps the full-FOV (binned) sensor mode -> wide angle preserved.
    # lores is the same image hardware-scaled to processing size by the ISP.
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        lores={"size": (PROCESSING_WIDTH, PROCESSING_HEIGHT)},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started: main {CAMERA_RESOLUTION} (FOV reference), "
          f"lores {(PROCESSING_WIDTH, PROCESSING_HEIGHT)} at {CAMERA_FRAMERATE} FPS.")

    time.sleep(1)
    hsv_processing_size = (HSV_PROCESSING_WIDTH, HSV_PROCESSING_HEIGHT)

    camera_acquisition_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, hsv_processing_size)
    )
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    # STAGED STARTUP: let the camera's current draw settle before the LiDAR
    # motor spin-up (a USB inrush of several hundred mA) hits the same rail.
    print(f"[POWER] Camera settled. Waiting {STARTUP_STAGE_DELAY_SEC}s before LiDAR spin-up...")
    time.sleep(STARTUP_STAGE_DELAY_SEC)

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

    # STAGED STARTUP: LiDAR is spinning; pause again before the YOLO model
    # load, which briefly loads all cores while weights are parsed.
    print(f"[POWER] LiDAR up. Waiting {STARTUP_STAGE_DELAY_SEC}s before YOLO model load...")
    time.sleep(STARTUP_STAGE_DELAY_SEC)

    yolo_ok = load_yolo()
    if STARTUP_TEST_ENABLED:
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
        # Start YOLO detection thread only if the model actually loaded --
        # otherwise the thread just burns CPU copying frames for nothing
        # and the HSV fallback handles detection anyway.
        detection_thread_stop_event = threading.Event()
        detection_thread = None
        if yolo_ok:
            detection_thread = threading.Thread(
                target=detection_thread_func,
                args=(detection_thread_stop_event,)
            )
            detection_thread.daemon = True
            detection_thread.start()
        else:
            print("[YOLO] Not loaded -- running on HSV fallback only.")

        while not shutdown_event.is_set():
            loop_start_time = time.monotonic()
            loop_counter += 1

            with camera_frame_lock:
                if 'bgr' not in latest_processed_frames:
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

            # Line counting only every Nth loop -- the masks are purely for
            # lap counting (6 s cooldown), so per-loop morphology is wasted CPU.
            if loop_counter % LINE_CHECK_EVERY_N_LOOPS == 0:
                current_time = time.time()
                # Color Detection
                blue_mask = filter_blue_objects(hsv)
                blue_in_view = detect_color_binary(blue_mask)

                # Blue Line Binary Logic with Cooldown
                if not blue_in_view and prev_blue_state:
                    if current_time > blue_cooldown_end_time:
                        blue_count += 1
                        print(f"Blue line crossed! Total blue lines: {blue_count}")
                        blue_cooldown_end_time = current_time + 6
                prev_blue_state = blue_in_view

                # # Orange Line Binary Logic with Cooldown
                # orange_mask = filter_orange_objects(hsv)
                # orange_in_view = detect_color_binary(orange_mask)
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

            is_near_field_mode = check_front_obstacle_proximity(scan_data, distance_mm=1100)

            # --- Read latest YOLO detections from background thread ---
            with detection_lock:
                det_packet = latest_detection_packet
            detections = det_packet['dets']
            det_method = det_packet['method']

            # Reject stale detections: the timestamp is the source frame's
            # capture time, so this bounds total camera->inference->steer
            # latency. Stale YOLO results fall through to the HSV fallback
            # below (which runs on the CURRENT frame).
            detection_age = time.time() - det_packet['ts'] if det_packet['ts'] > 0 else 999.0
            if detections and detection_age > DETECTION_MAX_AGE_SEC:
                detections = []
                det_method = "STALE"

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

            # NOTE: the old "PRIORITY 2.5: no detection -> go straight" branch
            # was removed. logic_label == "none" made it unconditionally true
            # whenever no pillar was visible, so the LiDAR wall-following
            # branch below was unreachable dead code and the robot drove
            # open track blind. With it gone, no-pillar frames fall through
            # to PID corridor centering. RE-TEST PID GAINS ON THE FIELD --
            # wall following now actually runs for the first time.

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
                print(f"[{loop_counter}] {current_robot_state} | Angle:{final_angle} Speed:{robot_speed_current:.2f} FPS:{int(fps)} Det:{det_method}({len(detections)}) age:{detection_age:.2f}s {logic_label}")

            # Build the display frame ONLY when streaming -- the copy,
            # detection boxes and text overlays are pure overhead otherwise.
            if STREAM_VIDEO:
                processed_frame = frame_bgr.copy()
                draw_detections(processed_frame, detections)
                if DEBUG_UI_OVERLAYS:
                    cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(processed_frame, f"FPS: {int(fps)}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    cv2.putText(processed_frame, f"Blue: {blue_count}/{max_line_crossings}, Orange: {orange_count}/{max_line_crossings}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                with output_frame_lock:
                    output_frame = processed_frame

            # Pace the control loop: new frames only arrive at 30 fps, so
            # spinning faster just burns power. 50 Hz is still well above
            # the camera/LiDAR data rate.
            pace_remaining = (1.0 / CONTROL_LOOP_MAX_HZ) - (time.monotonic() - loop_start_time)
            if pace_remaining > 0:
                time.sleep(pace_remaining)

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
        # Grab a reference under the lock, encode OUTSIDE the lock.
        # The control loop rebinds output_frame (never mutates in place),
        # so holding a reference after releasing the lock is safe. This
        # stops JPEG encoding from blocking the control loop's publish.
        with output_frame_lock:
            frame_to_encode = output_frame
        if frame_to_encode is None:
            time.sleep(0.01)
            continue
        (flag, encoded_image) = cv2.imencode(".jpg", frame_to_encode)
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
