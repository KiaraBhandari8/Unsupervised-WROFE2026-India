#!/usr/bin/env python3
"""Robot Control with YOLO Pillar Detection - Integrated Version"""

import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, Response
import threading
import time
import os
import ncnn

# Import your custom functions
try:
    from image_frame_combine_outer_inner_depth_sept06 import process_frame_for_steering
    from robot_motion import adjust_servo_angle, robot_forward_speed, robot_stop, robot_forward, motor_standby, init_servo
    from lidar_steering4sept import LidarScanner, PIDController, calculate_steering_error
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit()

# --- YOLO MODEL CONFIG ---
YOLO_PARAM = '/home/pi8/wrofe2025/yolo_model/finetune_100/model.ncnn.param'
YOLO_BIN = '/home/pi8/wrofe2025/yolo_model/finetune_100/model.ncnn.bin'
YOLO_INPUT_SIZE = 640
YOLO_CONF_THRESHOLD = 0.25
YOLO_CLASSES = {0: 'red', 1: 'green'}

# Initialize YOLO model globally
yolo_net = None

def load_yolo_model():
    """Load YOLO NCNN model"""
    global yolo_net
    print("Loading YOLO model...")
    yolo_net = ncnn.Net()
    yolo_net.opt.use_vulkan_compute = False
    yolo_net.opt.num_threads = 4
    yolo_net.load_param(YOLO_PARAM)
    yolo_net.load_model(YOLO_BIN)
    print("YOLO model loaded!")

def yolo_detect(img):
    """Run YOLO detection on image"""
    if yolo_net is None:
        return []
    
    h, w = img.shape[:2]
    scale = YOLO_INPUT_SIZE / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    padded = np.zeros((YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, 3), dtype=np.uint8)
    dw, dh = (YOLO_INPUT_SIZE - new_w) // 2, (YOLO_INPUT_SIZE - new_h) // 2
    padded[dh:dh + new_h, dw:dw + new_w] = resized
    
    mat = ncnn.Mat.from_pixels(padded, ncnn.Mat.PixelType.PIXEL_BGR2RGB, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE)
    mat.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
    ex = yolo_net.create_extractor()
    ex.set_light_mode(True)
    _, output = ex.extract('out0')
    output = np.array(output).reshape(6, 8400).T
    
    class_scores = output[:, 4:]
    max_scores = np.max(class_scores, axis=1)
    scale_inv = w / (YOLO_INPUT_SIZE - 2 * dw)
    
    dets = []
    for i in np.where(max_scores >= YOLO_CONF_THRESHOLD)[0]:
        conf = float(max_scores[i])
        cls_idx = int(np.argmax(class_scores[i]))
        cls = YOLO_CLASSES[cls_idx]
        cx = output[i, 0] * YOLO_INPUT_SIZE
        cy = output[i, 1] * YOLO_INPUT_SIZE
        bw = output[i, 2] * YOLO_INPUT_SIZE
        bh = output[i, 3] * YOLO_INPUT_SIZE
        x1 = int((cx - bw/2 - dw) * scale_inv)
        y1 = int((cy - bh/2 - dh) * scale_inv)
        x2 = int((cx + bw/2 - dw) * scale_inv)
        y2 = int((cy + bh/2 - dh) * scale_inv)
        dets.append({'class': cls, 'confidence': conf, 'bbox': [x1, y1, x2, y2], 'center_x': (x1 + x2) // 2})
    return dets


if __name__ == '__main__':
    print("--- Starting Robot Control System ---")
    
    init_servo()
    
    robot_stop()
    motor_standby()
    time.sleep(0.5)

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

# YOLO detection results
latest_yolo_detections = []
yolo_detections_lock = threading.Lock()

app = Flask(__name__)

# Turn Counter Variables
turn_counter = 0
max_turn_count = 12
previous_increment_time = time.time()
START_PAUSE_DURATION = 5
DELAY_BETWEEN_TURNS = 7
OUT_PARKING_MANEUVER = False
straight_detected_time = time.time()

# --- CONTROL CONSTANTS ---
SERVO_CENTER_ANGLE = 97
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
HSV_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3
HSV_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3

# --- LIDAR CONTROL CONSTANTS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 50
CLOCKWISE_WALL_FOLLOWING = True

if CLOCKWISE_WALL_FOLLOWING:
    LIDAR_PID_KP = 0.15
    LIDAR_PID_KI = 0.001
    LIDAR_PID_KD = 0.03
else:
    LIDAR_PID_KP = 0.2
    LIDAR_PID_KI = 0.001
    LIDAR_PID_KD = 0.05

LIDAR_SERVO_MIN_ANGLE = 70
LIDAR_SERVO_MAX_ANGLE = 120
LIDAR_STEERING_SCALE_FACTOR = 0.2

LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 40
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180
LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -40
LIDAR_LEFT_SIDE_DISTANCE_MM = 180
LIDAR_SIDE_STEER_MAGNITUDE = 15

# --- DEBUGGING AND UI ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

# --- BEHAVIOR STATES ---
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


# --- Helper Functions ---
def map_lidar_steering_angle(center_angle, pid_output, clockwise=True):
    adjusted_output = -1 * pid_output * LIDAR_STEERING_SCALE_FACTOR
    print(f"PID Output: {round(pid_output)} | Adjusted Output: {round(adjusted_output)}")
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
    """Analyzes LiDAR data to choose the most open path (left or right)"""
    global CLOCKWISE_WALL_FOLLOWING
    
    start_time = time.time()
    end_time = start_time + duration_sec
    
    if not scan_data:
        print("Parking Maneuver Warning: No LiDAR data. Defaulting to RIGHT turn.")
        CLOCKWISE_WALL_FOLLOWING = True
    else:
        left_distances = [dist for angle, dist in scan_data.items() if -90 <= angle <= -40 and dist > 0]
        right_distances = [dist for angle, dist in scan_data.items() if 40 <= angle <= 90 and dist > 0]
        
        avg_left = np.mean(left_distances) if left_distances else 0
        avg_right = np.mean(right_distances) if right_distances else 0
        
        print(f"Parking Maneuver Analysis: Avg Left Space={avg_left:.0f}mm, Avg Right Space={avg_right:.0f}mm")
        
        if avg_left > avg_right:
            CLOCKWISE_WALL_FOLLOWING = False
            print("Decision: Turning LEFT (Anti-Clockwise). Setting wall-following mode.")
        else:
            CLOCKWISE_WALL_FOLLOWING = True
            print("Decision: Turning RIGHT (Clockwise). Setting wall-following mode.")
    
    direction_multiplier = 1 if CLOCKWISE_WALL_FOLLOWING else -1
    servo_angle = SERVO_CENTER_ANGLE + (direction_multiplier * max_angle_magnitude)
    print(f"Executing escape maneuver with Servo Angle: {servo_angle}")

    while time.time() < end_time:
        adjust_servo_angle(servo_angle)
        robot_forward_speed(speed)
        time.sleep(0.05)

    robot_stop()
    adjust_servo_angle(SERVO_CENTER_ANGLE)
    time.sleep(0.5)


def check_imminent_collision_and_get_escape_route(scan_data):
    """Checks for an imminent forward collision and determines the best escape direction."""
    if not scan_data:
        return None

    is_collision_imminent = False
    for angle, distance in scan_data.items():
        if -10 <= angle <= 10 and 0 < distance < 100:
            is_collision_imminent = True
            break
    
    if not is_collision_imminent:
        return None

    left_distances = [d for a, d in scan_data.items() if -90 <= a < 0 and d > 0]
    right_distances = [d for a, d in scan_data.items() if 0 < a <= 90 and d > 0]

    avg_left = np.mean(left_distances) if left_distances else 0
    avg_right = np.mean(right_distances) if right_distances else 0

    if avg_left > avg_right:
        return "LEFT"
    else:
        return "RIGHT"


def check_for_straight_corridor(scan_data, min_dist_mm=1750, max_dist_mm=2750, angle_range=10):
    if not scan_data:
        return False

    left_front_distances = []
    right_front_distances = []
    for angle, distance in scan_data.items():
        if -angle_range <= angle < 0 and distance > 0:
            left_front_distances.append(distance)
        elif 0 <= angle <= angle_range and distance > 0:
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
def camera_acquisition_thread_func(picam2_instance, stop_event, processing_size, hsv_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition and processing thread started.")
    try:
        while not stop_event.is_set():
            captured_frame_rgb = picam2_instance.capture_array()

            processing_frame_rgb = cv2.resize(
                captured_frame_rgb,
                processing_size,
                interpolation=cv2.INTER_AREA
            )
            frame_bgr = cv2.cvtColor(processing_frame_rgb, cv2.COLOR_RGB2BGR)

            hsv_source_frame = cv2.resize(
                captured_frame_rgb,
                hsv_processing_size,
                interpolation=cv2.INTER_AREA
            )
            hsv_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_RGB2HSV)

            with camera_frame_lock:
                latest_processed_frames['rgb'] = processing_frame_rgb
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['hsv'] = hsv_frame

    except Exception as e:
        print(f"Camera Acquisition Thread Error: {e}")
    finally:
        print("Camera acquisition thread stopping.")


# --- YOLO Detection Thread ---
def yolo_detection_thread_func(stop_event):
    global latest_yolo_detections, yolo_detections_lock
    print("YOLO detection thread started.")
    try:
        while not stop_event.is_set():
            with camera_frame_lock:
                if 'bgr' in latest_processed_frames:
                    frame_bgr = latest_processed_frames['bgr'].copy()
                else:
                    time.sleep(0.01)
                    continue
            
            detections = yolo_detect(frame_bgr)
            
            with yolo_detections_lock:
                latest_yolo_detections = detections
            
            time.sleep(0.1)  # ~10 FPS
    except Exception as e:
        print(f"YOLO Detection Thread Error: {e}")
    finally:
        print("YOLO detection thread stopping.")


# --- Main Robot Control Loop ---
def robot_control_loop():
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
    hsv_processing_size = (HSV_PROCESSING_WIDTH, HSV_PROCESSING_HEIGHT)

    camera_acquisition_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, processing_size, hsv_processing_size)
    )
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    # Load YOLO model and start detection thread
    load_yolo_model()
    yolo_detection_thread = threading.Thread(
        target=yolo_detection_thread_func,
        args=(camera_thread_stop_event,)
    )
    yolo_detection_thread.daemon = True
    yolo_detection_thread.start()

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

    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    try:
        first_loop = True
        straight_corridor_detected = False
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

        while True:
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
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            if first_loop and OUT_PARKING_MANEUVER:
                print("Executing parking lot escape maneuver...")
                out_direction = get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=1.25, speed=0.5)
                first_loop = True
                OUT_PARKING_MANEUVER = True

            if loop_counter % 1 == 0:
                current_time = time.time()
                
                # Blue/Orange line detection
                blue_mask = filter_blue_objects(hsv)
                orange_mask = filter_orange_objects(hsv)
                blue_in_view = detect_color_binary(blue_mask)
                orange_in_view = detect_color_binary(orange_mask)

                if not blue_in_view and prev_blue_state:
                    if current_time > blue_cooldown_end_time:
                        blue_count += 1
                        print(f"Blue line crossed! Total blue lines: {blue_count}")
                        blue_cooldown_end_time = current_time + 6
                prev_blue_state = blue_in_view
            
            if loop_counter % 3 == 0:
                elapsed_time = time.monotonic() - program_start_time
                print("-"*75)
                print(f"Line Counts -> Blue: {blue_count}/{max_line_crossings}, Orange: {orange_count}/{max_line_crossings}, Total time elapsed: {elapsed_time:.2f} seconds")
                print("-"*75)

            if not crossed_12 and blue_count >= max_line_crossings:
                print(f"Max line count ({max_line_crossings}) reached for blue")
                crossed_12 = True
                crossed_time = time.time()

            if crossed_12 and (time.time() - crossed_time) > 4:
                robot_stop()
                time.sleep(120)
                break

            is_near_field_mode = check_front_obstacle_proximity(scan_data, distance_mm=1100)
            
            # Use image_frame_combine for lane detection (keep existing functionality)
            processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                frame_bgr,
                use_outer_roi_and_bottom_point=is_near_field_mode
            )
            vision_angle = -1*vision_angle

            if processed_frame is None:
                if STREAM_VIDEO:
                    with output_frame_lock:
                        output_frame = frame_bgr.copy()
                time.sleep(0.01)
                continue
            
            # Get YOLO detections
            with yolo_detections_lock:
                detections = latest_yolo_detections.copy()
            
            # Get high-priority maneuver decisions before main logic
            escape_direction = check_imminent_collision_and_get_escape_route(scan_data)
            side_alert_status = check_lidar_side_alerts(scan_data)
            
            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # --- BEHAVIOR ARBITRATION WITH YOLO ---
            # PRIORITY 0: IMMINENT FORWARD COLLISION OVERRIDE
            if (time.time() - straight_detected_time) < 2:
                straight_corridor_detected = False
                target_servo_angle = SERVO_CENTER_ANGLE
                display_text = "MODE: LiDARWF | Straight Override"
                print("LiDAR Straight Corridor Override Active")
            elif escape_direction == "LEFT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: ESCAPE LEFT!"
                print("Imminent Collision Override: Escaping LEFT")

            elif escape_direction == "RIGHT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: ESCAPE RIGHT!"
                print("Imminent Collision Override: Escaping RIGHT")

            # PRIORITY 1: IMMEDIATE SIDE OBSTACLE (LIDAR OVERRIDE)
            elif side_alert_status == "RIGHT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Right LiDAR!"
                print("LiDAR Side Override: RIGHT")
            
            elif side_alert_status == "LEFT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Left LiDAR!"
                print("LiDAR Side Override: LEFT")

            # PRIORITY 2: YOLO PILLAR DETECTION
            # Red pillar: pass on RIGHT (steer right)
            # Green pillar: pass on LEFT (steer left)
            elif detections:
                frame_width = frame_bgr.shape[1]
                red_near = None
                green_near = None
                
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    # Check if pillar is close (large bounding box)
                    if x2 - x1 > 50:
                        if det['class'] == 'red':
                            red_near = det['center_x']
                        elif det['class'] == 'green':
                            green_near = det['center_x']
                
                if red_near is not None:
                    current_robot_state = RobotState.RED_AVOIDANCE
                    target_servo_angle = SERVO_CENTER_ANGLE + 20
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    display_text = "MODE: RED PILLAR | Go RIGHT"
                    print(f"Red pillar detected! Steering RIGHT to pass on right side")
                    
                elif green_near is not None:
                    current_robot_state = RobotState.GREEN_AVOIDANCE
                    target_servo_angle = SERVO_CENTER_ANGLE - 20
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    display_text = "MODE: GREEN PILLAR | Go LEFT"
                    print(f"Green pillar detected! Steering LEFT to pass on left side")

            # PRIORITY 3: CAMERA-BASED OBSTACLE AVOIDANCE (fallback from image_frame_combine)
            elif logic_label == "red_obstacle" or logic_label == "obstacle":
                robot_speed_current = ROBOT_MANEUVER_SPEED
                current_robot_state = RobotState.RED_AVOIDANCE if logic_label == 'red_obstacle' else RobotState.GREEN_AVOIDANCE
                servo_adjust = -vision_angle * STEERING_GAIN
                print(f"Vision Angle:{round(vision_angle)} |Servo Adjust:{round(servo_adjust)}")
                target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                display_text = f"MODE: {'Red' if logic_label == 'red_obstacle' else 'Cam'}Avoid | Steer: {int(round(target_servo_angle))}°"

            # PRIORITY 4: LIDAR-BASED NAVIGATION (DEFAULT)
            elif (lidar_scanner and lidar_pid):
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                if scan_data:
                    if check_for_straight_corridor(scan_data, min_dist_mm=1750, max_dist_mm=2750, angle_range=10):
                        straight_corridor_detected = True
                        straight_detected_time = time.time()
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "MODE: LiDARWF | Straight Override"
                        print("LiDAR Straight Corridor Override Activated!")
                    else:
                        lidar_error = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM, clockwise=CLOCKWISE_WALL_FOLLOWING)
                        if lidar_error == 9999.0:
                            robot_stop()
                            current_robot_state = RobotState.STOP
                            display_text = "MODE: STOP (LiDAR Obstacle!)"
                            time.sleep(0.1)
                            continue
                        else:
                            pid_output = lidar_pid.update(lidar_error)
                            target_servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, True)
                            display_text = f"MODE: LiDARWF | Steer: {round(target_servo_angle)}° | Err: {lidar_error:.0f}mm"
                else:
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE
                    display_text = "MODE: Fallback (No LiDAR Data)"
            
            # PRIORITY 5: FALLBACK (NO LIDAR SYSTEM)
            else:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                display_text = f"MODE: Fallback | Logic: {logic_label}"

            # APPLY ROBOT MOTION
            if current_robot_state != RobotState.STOP:
                if check_front_obstacle_proximity(scan_data, distance_mm=150):
                    min_angle_limit = LIDAR_SERVO_MIN_ANGLE - 5
                    max_angle_limit = LIDAR_SERVO_MAX_ANGLE + 5
                else:
                    min_angle_limit = LIDAR_SERVO_MIN_ANGLE
                    max_angle_limit = LIDAR_SERVO_MAX_ANGLE
                
                final_angle = int(round(np.clip(target_servo_angle, min_angle_limit, max_angle_limit)))
                
                adjust_servo_angle(final_angle)
                if (final_angle <= 70) or (final_angle >= 110):
                    robot_forward_speed(ROBOT_SPEED_MAX)
                    print(f"Robot Speed : {ROBOT_SPEED_MAX}")
                else:
                    robot_forward_speed(robot_speed_current)
                    print(f"Robot Speed : {robot_speed_current}")
            else:
                robot_stop()

            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0
            print(f"Frames Processed Per Second (FPS): {int(fps)}")
            
            # Draw YOLO detection boxes
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                color = (0, 0, 255) if det['class'] == 'red' else (0, 255, 0)
                cv2.rectangle(processed_frame, (x1, y1), (x2, y2), color, 2)
                label = f"{det['class']}: {det['confidence']:.0%}"
                cv2.putText(processed_frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
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
        camera_thread_stop_event.set()
        camera_acquisition_thread.join()
        robot_stop()
        picam2.stop()
        
        if lidar_scanner:
            print("Disconnecting LiDAR...")
            lidar_scanner.disconnect()
            
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
    return '''<!DOCTYPE html>
<html>
<head>
    <title>Robot Camera with YOLO</title>
    <meta charset="utf-8">
    <style>
        body {
            margin: 0;
            background: #111;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            flex-direction: column;
        }
        img {
            max-width: 100%;
            max-height: 90vh;
        }
        .info {
            color: #0f0;
            font-family: monospace;
            padding: 10px;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="info">Robot Control with YOLO - http://192.168.1.100:5000</div>
    <img src="/video_feed">
</body>
</html>'''

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


# --- Main Execution Block ---
if __name__ == '__main__':
    print("--- Starting Robot Control System with YOLO ---")

    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()
    print("Robot control thread started.")

    try:
        hostname = os.uname()[1]
        print(f"Web server starting. Open http://{hostname}.local:5000 or http://192.168.1.100:5000")
    except AttributeError:
         import socket
         hostname = socket.gethostname()
         ip_address = socket.gethostbyname(hostname)
         print(f"Web server starting. Open http://{ip_address}:5000")

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

    print("Main application exiting.")