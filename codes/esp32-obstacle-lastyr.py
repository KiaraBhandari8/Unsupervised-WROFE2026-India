import cv2
import sys
import numpy as np
import serial
import struct
import math
import time
import os
import signal
import threading
import ydlidar        # Explicitly imported to solve the initialization error
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response

# --- CHECK FOR NCNN ACCELERATION ---
try:
    import ncnn
    NCNN_OK = True
except ImportError:
    NCNN_OK = False
    print("[WARN] ncnn not available, YOLO disabled")

# --- GLOBAL BUFFERS & LOCKS ---
output_frame = None
output_frame_lock = threading.Lock()

latest_lidar_data = {}
lidar_data_lock = threading.Lock()

latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()
shutdown_event = threading.Event()

app = Flask(__name__)

# --- DIRECTION & OVERRIDE CONFIGURATION TUNING ---
# BUG FIX 2: If your robot steers INTO walls instead of AWAY from them, flip this to False
CLOCKWISE_WALL_FOLLOWING = True 

# BUG FIX 2: Set to False to prevent the straight corridor checker from locking your wheels straight
ENABLE_STRAIGHT_OVERRIDE = False  

# --- TURN COUNTER & RACE CONTROL ---
turn_counter = 0
max_turn_count = 12
previous_increment_time = time.time()
START_PAUSE_DURATION = 5
DELAY_BETWEEN_TURNS = 7
OUT_PARKING_MANEUVER = False
straight_detected_time = 0.0
straight_override_duration = 1.5

# --- CONTROL SPEED CONSTANTS ---
SERVO_CENTER_ANGLE = 97
STEERING_GAIN = 0.1
ROBOT_MANEUVER_SPEED = 0.65
ROBOT_CRUISE_SPEED = 0.65
ROBOT_SPEED_MAX = 0.65

# --- CAMERA DIMENSION RESOLUTIONS ---
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
HSV_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   # 768
HSV_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  # 432

# --- YOLO DETECTION LAYER CONFIGURATIONS ---
YOLO_PARAM = '/home/pi8/yolo_model/pillar_640_v2/model.ncnn.param'
YOLO_BIN = '/home/pi8/yolo_model/pillar_640_v2/model.ncnn.bin'
YOLO_INPUT_SIZE = 640
YOLO_CONF_THRESHOLD = 0.55
YOLO_CLASSES = {0: 'red', 1: 'green'}
YOLO_PIXEL_STEERING_GAIN = 0.4
pi
# --- HSV PILLAR RECOGNITION FALLBACKS ---
HSV_LOWER_RED1 = np.array([0, 100, 100])
HSV_UPPER_RED1 = np.array([10, 255, 255])
HSV_LOWER_RED2 = np.array([160, 100, 100])
HSV_UPPER_RED2 = np.array([180, 255, 255])
HSV_LOWER_GREEN = np.array([40, 50, 50])
HSV_UPPER_GREEN = np.array([80, 255, 255])
HSV_MIN_CONTOUR_AREA = 150
HSV_MIN_WIDTH = 25

# --- HARDWARE CONNECTIVITY SERIAL CONSTANTS ---
SERVO_CENTER = 97
SERVO_MIN = 77
SERVO_MAX = 120
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- LIDAR CORRIDOR CENTERING CONSTANTS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 150

LIDAR_PID_KP = 0.12
LIDAR_PID_KI = 0.002
LIDAR_PID_KD = 0.05

LIDAR_SERVO_MIN_ANGLE = 77
LIDAR_SERVO_MAX_ANGLE = 120
LIDAR_STEERING_SCALE_FACTOR = 0.25

LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 40
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180
LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -40
LIDAR_LEFT_SIDE_DISTANCE_MM = 180
LIDAR_SIDE_STEER_MAGNITUDE = 20

STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

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

# --- INTEGRATED DRIVER: LIDAR ENGINE ---
class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.laser = None
        self.scan_data = {}
        self.MIN_ANGLE = -180.0
        self.MAX_ANGLE = 180.0
        self.MIN_RANGE = 0.02
        self.MAX_RANGE = 16.0

    def connect(self):
        try:
            ydlidar.os_init()
            self.laser = ydlidar.CYdLidar()
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.MAX_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.MIN_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.MAX_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.MIN_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
            if not self.laser.initialize():
                raise IOError(f"LiDAR connection failed: {self.laser.DescribeError()}")
            if not self.laser.turnOn():
                raise IOError(f"Failed to turn on YDLIDAR: {self.laser.DescribeError()}")
            print(f"LiDAR: Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            raise IOError(f"LiDAR connection failed: {e}")

    def disconnect(self):
        if self.laser:
            print("LiDAR: Disconnecting...")
            self.laser.turnOff()
            self.laser.disconnecting()
            self.laser = None
            print("LiDAR: Disconnected.")

    def get_scan_data(self):
        if not self.laser: return None
        self.scan_data = {}
        scan = ydlidar.LaserScan()
        try:
            if self.laser.doProcessSimple(scan):
                for p in scan.points:
                    if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                        angle_degrees = round(math.degrees(p.angle))
                        self.scan_data[angle_degrees] = p.range * 1000.0
                return self.scan_data
            return None
        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None

# --- INTEGRATED DRIVER: PID ENGINE ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: return self.prev_error
        P = self.Kp * current_error
        self.integral += current_error * dt
        I = self.Ki * self.integral
        derivative = (current_error - self.prev_error) / dt
        D = self.Kd * derivative
        self.prev_error = current_error
        self.last_time = current_time
        return P + I + D

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

# --- HARDWARE TELEMETRY COM LINK ---
ser = None
ESP32_OK = False

try:
    ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.1)
    time.sleep(2)
    ESP32_OK = True
    print(f"[SERIAL] ESP32 Channel Open on: {PI_TO_ESP_PORT}")
except Exception as e:
    print(f"[SERIAL] Interface Initialization Failure: {e}")

def cmd(angle, speed):
    if ser and ESP32_OK:
        packet = f"STR:{int(angle)},SPD:{int(speed * 255)}\n"
        ser.write(packet.encode())
        ser.flush()

def stop_robot():
    if ser and ESP32_OK:
        ser.write(f"STR:{SERVO_CENTER},SPD:0\n".encode())
        ser.flush()

def startup_test():
    if not (ser and ESP32_OK):
        print("[STARTUP] Skipped hardware validation sweep - no connection.")
        return
    print("[STARTUP] Initializing mechanical servo sweep validation...")
    for angle in [SERVO_CENTER - 20, SERVO_CENTER, SERVO_CENTER + 20, SERVO_CENTER]:
        ser.write(f"STR:{angle},SPD:0\n".encode())
        ser.flush()
        time.sleep(0.3)
    print("[STARTUP] Executing motor pulse test...")
    cmd(SERVO_CENTER, 0.4)
    time.sleep(0.4)
    stop_robot()
    print("[STARTUP] Hardware Validation Complete.")

# --- INTEGRATED SPATIAL MATH MATRIX PROCESSING ---
def calculate_steering_error(scan_data, target_distance_mm=500, safety_distance_mm=150, clockwise=True):
    front_angles_degrees = range(-5, 6)

    if clockwise:
        right_wall_angles_degrees = range(35, 81)
        left_wall_angles_degrees = range(-90, -30)
    else:
        right_wall_angles_degrees = range(30, 85)
        left_wall_angles_degrees = range(-80, -34)

    front_distances = [scan_data[angle] for angle in front_angles_degrees if angle in scan_data and scan_data[angle] > 0]
    for dist in front_distances:
        if dist < safety_distance_mm:
            print(f"LiDAR: Obstacle at {dist:.0f}mm. STOP.")
            return 9999.0

    right_raw = [d for a, d in scan_data.items() if a in right_wall_angles_degrees and d is not None and 0 < d < 3000]
    left_raw = [d for a, d in scan_data.items() if a in left_wall_angles_degrees and d is not None and 0 < d < 3000]

    right_raw.sort()
    left_raw.sort()
    num_values = 8
    closest_right = right_raw[:num_values] if len(right_raw) >= num_values else right_raw
    closest_left = left_raw[:num_values] if len(left_raw) >= num_values else left_raw

    wall_right = np.median(closest_right) if closest_right else None
    wall_left = np.median(closest_left) if closest_left else None

    if wall_right is not None or wall_left is not None:
        r_str = f"{wall_right:.0f}" if wall_right is not None else "N/A"
        l_str = f"{wall_left:.0f}" if wall_left is not None else "N/A"
        print(f"LiDAR: Right wall: {r_str}mm, Left wall: {l_str}mm")

    if wall_right is not None and wall_left is not None:
        error = wall_right - wall_left
    elif wall_right is not None:
        error = wall_right - target_distance_mm
    elif wall_left is not None:
        error = target_distance_mm - wall_left
    else:
        error = 0.0

    return error

# --- LAP COUNT VISUAL SEGMENTATION FILTERS ---
def filter_blue_objects(hsv_frame):
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=2)
    return cv2.dilate(blue_mask, kernel, iterations=2)

def filter_orange_objects(hsv_frame):
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    orange_mask = cv2.erode(orange_mask, kernel, iterations=2)
    return cv2.dilate(orange_mask, kernel, iterations=2)

def detect_color_binary(mask, threshold=4000):
    return cv2.countNonZero(mask) > threshold

# ===================== YOLO PILLAR INFERENCE ENGINE =====================
yolo_net = None

def load_yolo():
    global yolo_net
    if not NCNN_OK: return False
    if not os.path.exists(YOLO_PARAM): return False
    print("[YOLO] Initializing model layers into memory...")
    yolo_net = ncnn.Net()
    yolo_net.opt.num_threads = 4
    yolo_net.load_param(YOLO_PARAM)
    yolo_net.load_model(YOLO_BIN)
    print("[YOLO] Inference layer operational.")
    return True

def detect_yolo(img):
    if yolo_net is None: return None
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
    output = np.array(out).T
    if output.shape[0] == 6: output = output.T
    class_cols = output[:, 4:]
    max_scores = np.max(class_cols, axis=1)
    scale_inv = w / (YOLO_INPUT_SIZE - 2 * dw)
    dets = []
    for i in np.where(max_scores >= YOLO_CONF_THRESHOLD)[0]:
        conf = float(max_scores[i])
        cls_idx = int(np.argmax(class_cols[i]))
        cls = YOLO_CLASSES[cls_idx]
        x1 = int((output[i, 0] - output[i, 2]/2 - dw) * scale_inv)
        y1 = int((output[i, 1] - output[i, 3]/2 - dh) * scale_inv)
        x2 = int((output[i, 0] + output[i, 2]/2 - dw) * scale_inv)
        y2 = int((output[i, 1] + output[i, 3]/2 - dh) * scale_inv)
        dets.append({'class': cls, 'conf': conf, 'bbox': [x1, y1, x2, y2],
                     'width': x2 - x1, 'cx': (x1 + x2) // 2})
    if len(dets) > 1:
        keep = cv2.dnn.NMSBoxes([d['bbox'] for d in dets], [d['conf'] for d in dets], 0.1, 0.5)
        if len(keep) > 0: dets = [dets[i] for i in keep.flatten()]
    return dets

def detect_hsv_fallback(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, HSV_LOWER_RED1, HSV_UPPER_RED1) + cv2.inRange(hsv, HSV_LOWER_RED2, HSV_UPPER_RED2)
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
    if yolo_net is not None:
        yolo_dets = detect_yolo(frame_bgr)
        if yolo_dets is not None and len(yolo_dets) > 0: return yolo_dets, "YOLO"
    hsv_dets = detect_hsv_fallback(frame_bgr)
    if hsv_dets: return hsv_dets, "HSV"
    return [], "NONE"

def compute_vision_from_detections(dets, fw):
    reds = [d for d in dets if d['class'] == 'red']
    greens = [d for d in dets if d['class'] == 'green']
    if reds:
        largest = max(reds, key=lambda d: d['width'])
        pX = largest['cx']
        target_x = int(fw * 0.85)
        return YOLO_PIXEL_STEERING_GAIN * (target_x - pX), "red_obstacle"
    if greens:
        largest = max(greens, key=lambda d: d['width'])
        pX = largest['cx']
        target_x = int(fw * 0.15)
        return YOLO_PIXEL_STEERING_GAIN * (target_x - pX), "obstacle"
    return 0, "none"

def map_steering_displacement(frame, dets):
    for d in dets:
        x1, y1, x2, y2 = d['bbox']
        clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
        label = f"{d['class']}:{int(d['conf']*100)}%"
        cv2.putText(frame, label, (x1, y1-3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)

# --- RECOVERY CONTROL LOGIC HELPERS ---
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
        if -2 <= angle <= 2 and 0 < dist < distance_mm: return True
    return False

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
        if avg_left > avg_right:
            CLOCKWISE_WALL_FOLLOWING = False
        else:
            CLOCKWISE_WALL_FOLLOWING = True
    direction_multiplier = 1 if CLOCKWISE_WALL_FOLLOWING else -1
    servo_angle = SERVO_CENTER_ANGLE + (direction_multiplier * max_angle_magnitude)
    while time.time() < end_time:
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
    avg_left = np.mean(left_distances) if left_distances else 0
    avg_right = np.mean(right_distances) if right_distances else 0
    return "LEFT" if avg_left > avg_right else "RIGHT"

def check_for_straight_corridor(scan_data, min_dist_mm=1000, max_dist_mm=3500, angle_range=10):
    if not scan_data: return False
    left_front_distances = [d for a, d in scan_data.items() if -angle_range <= a < 0 and d > 0]
    right_front_distances = [d for a, d in scan_data.items() if 0 <= a <= angle_range and d > 0]
    if not left_front_distances or not right_front_distances: return False
    is_left_in_range = min_dist_mm < (sum(left_front_distances)/len(left_front_distances)) < max_dist_mm
    is_right_in_range = min_dist_mm < (sum(right_front_distances)/len(right_front_distances)) < max_dist_mm
    return is_left_in_range and is_right_in_range

# --- ASYNCHRONOUS DAEMON PACKET THREADS ---
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("[LIDAR LOOP] Thread engine online.")
    try:
        while True:
            if not any(t.name == 'MainThread' and t.is_alive() for t in threading.enumerate()): break
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock: latest_lidar_data = data
            time.sleep(0.01)
    except Exception as e:
        print(f"[LIDAR SYSTEM CRASH] {e}")

def camera_acquisition_thread_func(picam2_instance, stop_event, processing_size, hsv_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("[VISION LOOP] Multi-stage processing matrix initialized.")
    try:
        while not stop_event.is_set():
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
        print(f"[VISION SYSTEM CRASH] {e}")

# --- ROBOT DECISION MACHINE LOOP ---
def robot_control_loop(shutdown_event):
    global output_frame, output_frame_lock, current_robot_state, camera_thread_stop_event
    global straight_detected_time, OUT_PARKING_MANEUVER, turn_counter, max_turn_count
    global ROBOT_SPEED_MAX, ROBOT_MANEUVER_SPEED, ROBOT_CRUISE_SPEED, CLOCKWISE_WALL_FOLLOWING

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION}, transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE}, buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(1)
    
    camera_acquisition_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, (PROCESSING_WIDTH, PROCESSING_HEIGHT), (HSV_PROCESSING_WIDTH, HSV_PROCESSING_HEIGHT))
    )
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    lidar_scanner, lidar_pid = None, None
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_thread.daemon = True
        lidar_thread.start()
        lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
    except Exception as e:
        print(f"[WARN] Continuing in headless visual path mode. LiDAR initialization failure: {e}")

    load_yolo()
    startup_test()
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT

    try:
        first_loop = True
        blue_count, orange_count, max_line_crossings = 0, 0, 12
        prev_blue_state, prev_orange_state = False, False
        blue_cooldown_end_time, orange_cooldown_end_time = 0.0, 0.0
        loop_counter = 0
        program_start_time = time.monotonic()
        crossed_12 = False
        crossed_time = 0.0
        last_det_time = 0.0
        detections = []
        det_method = "NONE"

        while not shutdown_event.is_set():
            loop_start_time = time.monotonic()
            loop_counter += 1
            
            with camera_frame_lock:
                if not latest_processed_frames:
                    time.sleep(0.01)
                    continue
                frame_bgr = latest_processed_frames['bgr'].copy()
                hsv = latest_processed_frames['hsv'].copy()
            
            scan_data = None
            if lidar_scanner:
                with lidar_data_lock: scan_data = latest_lidar_data.copy()

            if first_loop and OUT_PARKING_MANEUVER:
                get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=1.25, speed=0.5)
                first_loop = False

            # Evaluate Lap Progress via Base Markers
            current_time = time.time()
            blue_mask = filter_blue_objects(hsv)
            blue_in_view = detect_color_binary(blue_mask)
            if not blue_in_view and prev_blue_state:
                if current_time > blue_cooldown_end_time:
                    blue_count += 1
                    print(f"[LAP COUNTER] Blue Boundary Crossed! Total: {blue_count}")
                    blue_cooldown_end_time = current_time + 6.0
            prev_blue_state = blue_in_view

            if not crossed_12 and blue_count >= max_line_crossings:
                crossed_12 = True
                crossed_time = time.time()
                blue_count = 0

            if crossed_12 and (time.time() - crossed_time) > 4.0:
                stop_robot()
                time.sleep(120)
                break

            # Object Detection Segment
            t_now = time.time()
            if t_now - last_det_time > 0.15:
                detections, det_method = detect_pillars(frame_bgr)
                last_det_time = t_now

            vision_angle, logic_label = compute_vision_from_detections(detections, frame_bgr.shape[1])
            processed_frame = frame_bgr.copy()
            map_steering_displacement(processed_frame, detections)
            
            escape_direction = check_imminent_collision_and_get_escape_route(scan_data)
            side_alert_status = check_lidar_side_alerts(scan_data)
            
            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # ====================================================
            # ARBITRATION TREE: BEHAVIOR STATES ENGINE
            # ====================================================
            if escape_direction == "LEFT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "CRITICAL: SLAM LEFT ESCAPE!"
            elif escape_direction == "RIGHT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "CRITICAL: SLAM RIGHT ESCAPE!"

            elif side_alert_status == "RIGHT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "OVERRIDE: FLANK WALL RIGHT"
            elif side_alert_status == "LEFT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "OVERRIDE: FLANK WALL LEFT"

            elif logic_label == "red_obstacle" or logic_label == "obstacle":
                robot_speed_current = ROBOT_MANEUVER_SPEED
                current_robot_state = RobotState.RED_AVOIDANCE if logic_label == 'red_obstacle' else RobotState.GREEN_AVOIDANCE
                servo_adjust = -vision_angle * STEERING_GAIN
                target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                display_text = f"CAM EVASION: {'Red' if logic_label == 'red_obstacle' else 'Green'} Obstacle"

            elif lidar_scanner and lidar_pid:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                if scan_data:
                    # BUG FIX 2: Check whether the hardcoded straight alignment override flag is active
                    if ENABLE_STRAIGHT_OVERRIDE and straight_detected_time > 0 and (time.time() - straight_detected_time) < straight_override_duration:
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "CORRIDOR: FIXED LOCK STRAIGHT"
                    elif ENABLE_STRAIGHT_OVERRIDE and check_for_straight_corridor(scan_data, min_dist_mm=1000, max_dist_mm=3500, angle_range=10):
                        straight_detected_time = time.time()
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "CORRIDOR: OPEN RUNWAY VISIBLE"
                    else:
                        straight_detected_time = 0.0
                        lidar_error = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM, clockwise=CLOCKWISE_WALL_FOLLOWING)
                        if lidar_error == 9999.0:
                            stop_robot()
                            current_robot_state = RobotState.STOP
                            continue
                        pid_output = lidar_pid.update(lidar_error)
                        target_servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=CLOCKWISE_WALL_FOLLOWING)
                        display_text = f"CORRIDOR: CENTERING | Dev: {lidar_error:.0f}mm"
                else:
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE

            else:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE

            # APPLY PHYSICAL DISPATCH COMMAND MATRICES
            if current_robot_state != RobotState.STOP:
                min_angle_limit = LIDAR_SERVO_MIN_ANGLE - 5 if check_front_obstacle_proximity(scan_data, distance_mm=150) else LIDAR_SERVO_MIN_ANGLE
                max_angle_limit = LIDAR_SERVO_MAX_ANGLE + 5 if check_front_obstacle_proximity(scan_data, distance_mm=150) else LIDAR_SERVO_MAX_ANGLE
                final_angle = int(round(np.clip(target_servo_angle, min_angle_limit, max_angle_limit)))

                deviation = abs(final_angle - SERVO_CENTER_ANGLE)
                if deviation > 15:
                    cmd(final_angle, ROBOT_CRUISE_SPEED * 0.75)
                else:
                    cmd(final_angle, robot_speed_current)
            else:
                stop_robot()

            # BUILD DASHBOARD DIAGNOSTICS STREAM OVERLAYS
            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0
            if DEBUG_UI_OVERLAYS:
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(processed_frame, f"FPS: {int(fps)}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"Lines - Blue: {blue_count}/{max_line_crossings}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
            if STREAM_VIDEO:
                with output_frame_lock: output_frame = processed_frame.copy()

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Interrupted manually.")
    finally:
        shutdown_event.set()
        camera_thread_stop_event.set()
        stop_robot()
        if lidar_scanner: lidar_scanner.disconnect()
        try: picam2.stop()
        except: pass

# --- BUG FIX 1: FLASK TELEMETRY STREAM SERVICES ---
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
            flag, encoded_image = cv2.imencode(".jpg", output_frame)
            if not flag: continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

@app.route("/")
def index(): 
    # Returns an HTML dashboard wrapping your video feed automatically
    return """
    <html>
      <head>
        <title>WRO 2026 Telemetry Stream Dashboard</title>
      </head>
      <body style="background-color: #0f0f11; color: #e2e8f0; font-family: sans-serif; text-align: center; padding-top: 30px;">
        <h1 style="color: #38bdf8; margin-bottom: 5px;">WRO 2026 Autonomous Navigation Dashboard</h1>
        <p style="color: #94a3b8; font-size: 14px; margin-bottom: 20px;">Distributed Real-Time Robot Telemetry Stream Layer</p>
        <div style="display: inline-block; background-color: #1e1e24; padding: 15px; border-radius: 12px; box-shadow: 0 10px 15px -3px rgba(0,0,0,0.5);">
          <img src="/video_feed" style="border: 3px solid #334155; border-radius: 8px; width: 768px; height: 432px;"/>
        </div>
        <p style="color: #64748b; font-size: 11px; margin-top: 15px;">MakerWorks Lab System Engine Link Active</p>
      </body>
    </html>
    """

@app.route("/video_feed")
def video_feed(): return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    stop_robot()
    time.sleep(0.5)
    control_thread = threading.Thread(target=robot_control_loop, args=(shutdown_event,))
    control_thread.start()

    def handle_sigint(sig, frame): shutdown_event.set()
    signal.signal(signal.SIGINT, handle_sigint)

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt: pass
    shutdown_event.set()
    control_thread.join(timeout=5)
    stop_robot()