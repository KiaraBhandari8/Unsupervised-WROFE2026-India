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

# --- IMPORT CUSTOM VISION AND LIDAR EXTENSIONS ---
try:
    from image_frame_combine_outer_inner_depth_test1 import process_frame_for_steering
    from lidar_steering4sept import LidarScanner, PIDController, calculate_steering_error
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to mount local tracking components: {e}")
    sys.exit(1)

# --- GLOBAL SHUTDOWN SYSTEM TRACKERS ---
global_shutdown_event = threading.Event()  
esp_ser = None                              
lidar_scanner = None                        
picam2 = None                               

# --- LIDAR CONTROL DESIGN PARAMETERS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 200  
WALL_LOSS_THRESHOLD_MM = 1400.0  
CLOCKWISE_WALL_FOLLOWING = True  
WALL_FOLLOW_TARGET_MM = LIDAR_TARGET_DISTANCE_MM  

# Configurations for the close-range side panic state
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180  
LIDAR_LEFT_SIDE_DISTANCE_MM = 180   
LIDAR_SIDE_STEER_MAGNITUDE = 5     

# Hardware Servo Limits
LIDAR_SERVO_MIN_ANGLE = 10
LIDAR_SERVO_MAX_ANGLE = 170
FRONT_SCAN_ANGLE_DEG = 15      

# --- GLOBAL BUFFER LOCKS AND REGISTERS ---
output_frame = None
output_frame_lock = threading.Lock()

latest_lidar_data = {}
lidar_data_lock = threading.Lock()

latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

app = Flask(__name__)

PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- CONTROL ACTUATION LIMITS ---
SERVO_CENTER_ANGLE = 97       
STEERING_ANGLE_CLIP_DEGREES = 40  
ROBOT_SPEED = 0  #180       
ROBOT_CRUISE_SPEED = 0 #180      
ROBOT_MANEUVER_SPEED = 0 #150  

SWERVE_SAFETY_TIMEOUT = 2.0    
CORNER_DETECTION_COOLDOWN_SEC = 4.0

# Independent Vision Calibration Parameters
STEERING_GAIN_GREEN = 0.1     
STEERING_GAIN_RED = 0.14 
RED_CLEARANCE_OFFSET = 8      

# --- CAMERA CONFIGURATION MATRIX ---
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
HSV_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   
HSV_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  

STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

class RobotState:
    INITIALIZING = "INITIALIZING"
    PURE_GYRO_START = "PURE_GYRO_START"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    VISION_OBSTACLE_AVOIDANCE = "VISION_OBSTACLE_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    LAP_TERMINATION = "LAP_TERMINATION"

current_robot_state = RobotState.INITIALIZING
current_yaw = 0.0

# ====================================================
# PURE PYTHON LOCALIZED 2D EUCLIDEAN CLUSTERING ENGINE
# ====================================================
def extract_lidar_clusters(scan_data, min_angle, max_angle, distance_threshold=160.0):
    """Segments raw coordinate Soup arrays into classified spatial obstacle objects."""
    points = []
    for angle, dist in scan_data.items():
        if min_angle <= angle <= max_angle and dist > 0:
            rad = np.radians(angle)
            x = dist * np.sin(rad)
            y = dist * np.cos(rad)
            points.append((x, y, angle, dist))
            
    if not points:
        return []
        
    clusters = []
    unvisited = list(points)
    
    while unvisited:
        current_pt = unvisited.pop(0)
        current_cluster = [current_pt]
        
        i = 0
        while i < len(current_cluster):
            target = current_cluster[i]
            neighbors = []
            for candidate in unvisited:
                d = np.sqrt((target[0] - candidate[0])**2 + (target[1] - candidate[1])**2)
                if d < distance_threshold:
                    neighbors.append(candidate)
            for n in neighbors:
                current_cluster.append(n)
                unvisited.remove(n)
            i += 1
        clusters.append(current_cluster)
    return clusters

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
        except Exception: pass
    if lidar_scanner:
        try: lidar_scanner.disconnect()
        except Exception: pass
    if picam2:
        try: picam2.stop()
        except: pass
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

def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
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
    global CLOCKWISE_WALL_FOLLOWING, current_yaw, esp_ser, lidar_scanner, picam2

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] High-speed serial connection established with ESP32 execution layer.")
    except Exception as e:
        print(f"[FATAL] Serial bridge initialization failed: {e}")
        sys.exit(1)

    print("[INFO] Zeroing orientation tracking profiles for new run...")
    esp_ser.write(b"RST_YAW\n")
    esp_ser.flush()
    time.sleep(0.1)
    current_yaw = 0.0

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION}, controls={"FrameRate": CAMERA_FRAMERATE}, buffer_count=CAMERA_BUFFER_COUNT
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
        print(f"[WARN] LiDAR interface offline: {e}")
        lidar_scanner = None

    gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
    wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)

    current_robot_state = RobotState.PURE_GYRO_START
    turn_count = 0
    avoid_state = "IDLE"       
    avoid_direction = None     
    clearance_end_time = 0.0
    recovery_end_time = 0.0
    swerve_start_time = 0.0    
    corner_cooldown_end_time = 0.0  
    
    blue_count = 0
    prev_blue_state = False
    blue_cooldown_end_time = 0.0
    
    print(f"[SYSTEM] Calibration complete. Initial State: {current_robot_state}")

    try:
        while not global_shutdown_event.is_set():
            loop_start_time = time.monotonic()
            display_text = "MODE: IDLE"  
            now = time.monotonic()

            while esp_ser.in_waiting > 0:
                try:
                    raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line.startswith("YAW:"):
                        current_yaw = float(raw_line.split(":")[1])
                except Exception: pass

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

            front_angles = range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
            front_points = [scan_data[a] for a in front_angles if a in scan_data and scan_data[a] > 0]
            avg_front_baseline = sum(front_points) / len(front_points) if front_points else 2000.0

            # ====================================================
            # 1. LIVE TURN PRE-CALCULATIONS MATRIX
            # ====================================================
            left_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
            right_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
            avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
            avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
            in_cooldown = now < corner_cooldown_end_time

            is_turning_right_opening = (avg_left < 1100.0 and avg_right > 1600.0)
            is_turning_left_opening  = (avg_right < 900.0 and avg_left > 1700.0)
            
            IS_APPROACHING_TURN = (avg_front_baseline <= 1300.0) and (is_turning_right_opening or is_turning_left_opening) and not in_cooldown

            # --- RUN VISION ENGINE WITH CLEAN FIXED CHASSIS BOUNDARIES ---
            is_near_field_mode = avg_front_baseline < 1100.0 
            processed_frame, vision_angle, _, logic_label, vertical_dist_to_bottom = process_frame_for_steering(
                frame_bgr, use_outer_roi_and_bottom_point=is_near_field_mode
            )
            vision_angle = -1 * vision_angle

            # Handle Lap Lines
            current_timestamp = time.time()
            blue_mask = filter_blue_objects(hsv)
            if detect_color_binary(blue_mask, threshold=4000) and not prev_blue_state:
                if current_timestamp > blue_cooldown_end_time:
                    blue_count += 1
                    blue_cooldown_end_time = current_timestamp + 5.0
            prev_blue_state = detect_color_binary(blue_mask, threshold=4000)

            if blue_count >= 12 or turn_count >= 12:
                current_robot_state = RobotState.LAP_TERMINATION
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                break

            # ====================================================
            # 2. PRIORITY EXECUTION TREE
            # ====================================================
            side_alert = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, safety_distance_mm=150, clockwise=CLOCKWISE_WALL_FOLLOWING)
            right_side_panic = [scan_data[a] for a in range(40, 76) if a in scan_data and 0 < scan_data[a] < LIDAR_RIGHT_SIDE_DISTANCE_MM]
            left_side_panic = [scan_data[a] for a in range(-75, -39) if a in scan_data and 0 < scan_data[a] < LIDAR_LEFT_SIDE_DISTANCE_MM]

            # LEVEL A: EVASION TIMERS
            if avoid_state == "CLEARING":
                current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                if now < clearance_end_time:
                    target_servo_angle = SERVO_CENTER_ANGLE + (18 if avoid_direction == "RED" else -18)
                    display_text = f"MODE: Latch Clear Flank | Time: {clearance_end_time - now:.2f}s"
                else:
                    avoid_state = "RECOVERING"
                    recovery_end_time = now + 0.40
                    target_servo_angle = SERVO_CENTER_ANGLE

            elif avoid_state == "RECOVERING":
                current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                robot_speed_current = ROBOT_CRUISE_SPEED
                if now < recovery_end_time:
                    target_servo_angle = SERVO_CENTER_ANGLE + (-10 if avoid_direction == "RED" else 10)
                    display_text = f"MODE: Latch Counter Realign | Time: {recovery_end_time - now:.2f}s"
                else:
                    avoid_state = "IDLE"
                    avoid_direction = None
                    gyro_straight_pid.reset()
                    wall_follow_pid.reset()
                    target_servo_angle = SERVO_CENTER_ANGLE

            # LEVEL B: EMERGENCY SIDE WALL ESCAPES
            elif right_side_panic and avoid_state == "IDLE":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Right Wall Clear"

            elif left_side_panic and avoid_state == "IDLE":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Left Wall Clear"

            # ====================================================
            # LEVEL C: CORNER LIDAR CLUSTERING ENGINES
            # ====================================================
            elif IS_APPROACHING_TURN and avoid_state == "IDLE":
                current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                turn_dir = "RIGHT" if is_turning_right_opening else "LEFT"
                CLOCKWISE_WALL_FOLLOWING = True if turn_dir == "RIGHT" else False
                
                if turn_dir == "RIGHT":
                    # Cluster right corner tracking sector (25 to 85 degrees)
                    clusters = extract_lidar_clusters(scan_data, 25, 85, distance_threshold=150.0)
                    
                    # Search for isolated obstacle profiles separate from continuous outer borders
                    pillar_blob = None
                    for c in clusters:
                        if 2 <= len(c) <= 18:  # Wall segments are long; distinct obstacles are small clusters
                            pillar_blob = c
                            break
                    
                    if pillar_blob:
                        avg_obstacle_angle = sum([p[2] for p in pillar_blob]) / len(pillar_blob)
                        # An obstacle is detected inside our turning corridor: Steer left to pass on the left
                        target_servo_angle = SERVO_CENTER_ANGLE - 25
                        display_text = f"CORNER CLUSTER: Pillar at {avg_obstacle_angle:.0f}° | Evading Left"
                    else:
                        # Clear track corner opening: Execute regular continuous turn sweep
                        target_servo_angle = SERVO_CENTER_ANGLE + 24
                        display_text = "CORNER CLUSTER: Clear Lane | Sweeping Right"
                else:
                    # Cluster left corner tracking sector (-85 to -25 degrees)
                    clusters = extract_lidar_clusters(scan_data, -85, -25, distance_threshold=150.0)
                    pillar_blob = None
                    for c in clusters:
                        if 2 <= len(c) <= 18:
                            pillar_blob = c
                            break
                    
                    if pillar_blob:
                        avg_obstacle_angle = sum([p[2] for p in pillar_blob]) / len(pillar_blob)
                        # Obstacle encountered inside left turn corridor: Steer right to pass on the right
                        target_servo_angle = SERVO_CENTER_ANGLE + 25
                        display_text = f"CORNER CLUSTER: Pillar at {avg_obstacle_angle:.0f}° | Evading Right"
                    else:
                        target_servo_angle = SERVO_CENTER_ANGLE - 24
                        display_text = "CORNER CLUSTER: Clear Lane | Sweeping Left"

                # Check watchdog turn conditions to increment laps continuously
                if abs(current_yaw) > 60.0:
                    turn_count += 1
                    corner_cooldown_end_time = now + CORNER_DETECTION_COOLDOWN_SEC

            # LEVEL D: STRAIGHTAWAY VISION SWERVE
            elif (logic_label in ["red_obstacle", "obstacle"] or avoid_state == "SWERVE"):
                IS_OBSTACLE_NEARBY = vertical_dist_to_bottom < 160.0
                
                if IS_OBSTACLE_NEARBY or avoid_state == "SWERVE":
                    if avoid_state != "SWERVE":
                        avoid_state = "SWERVE"
                        swerve_start_time = now
                    
                    swerve_elapsed = now - swerve_start_time
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED

                    if logic_label == "red_obstacle":
                        avoid_direction = "RED"
                        servo_adjust = -vision_angle * STEERING_GAIN_RED
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust + RED_CLEARANCE_OFFSET
                    else:
                        avoid_direction = "GREEN"
                        servo_adjust = -vision_angle * STEERING_GAIN_GREEN
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust

                    display_text = f"MODE: Straight Swerve {avoid_direction} | Steer: {int(target_servo_angle)}°"

                    if abs(vision_angle) > 35 or swerve_elapsed > SWERVE_SAFETY_TIMEOUT:
                        avoid_state = "CLEARING"
                        clearance_end_time = now + 0.70
                else:
                    logic_label = "none"

            # LEVEL E: BASELINE TRACK CORRIDOR CENTERING
            if logic_label == "none" and avoid_state == "IDLE" and not IS_APPROACHING_TURN:
                robot_speed_current = ROBOT_CRUISE_SPEED

                if turn_count >= 1:
                    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                    if CLOCKWISE_WALL_FOLLOWING:
                        left_follow_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
                        if left_follow_pts:
                            avg_left_wall = sum(left_follow_pts) / len(left_follow_pts)
                            if avg_left_wall > WALL_LOSS_THRESHOLD_MM:
                                target_servo_angle = SERVO_CENTER_ANGLE - gyro_straight_pid.update(0.0 - current_yaw)
                                display_text = "MODE: Gap -> Gyro Straight"
                            else:
                                wall_error = avg_left_wall - WALL_FOLLOW_TARGET_MM
                                target_servo_angle = SERVO_CENTER_ANGLE - wall_follow_pid.update(wall_error)
                                display_text = f"MODE: Track Left | Err: {wall_error:.0f}mm"
                        else:
                            target_servo_angle = SERVO_CENTER_ANGLE - gyro_straight_pid.update(0.0 - current_yaw)
                    else:
                        right_follow_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
                        if right_follow_pts:
                            avg_right_wall = sum(right_follow_pts) / len(right_follow_pts)
                            if avg_right_wall > WALL_LOSS_THRESHOLD_MM:
                                target_servo_angle = SERVO_CENTER_ANGLE - gyro_straight_pid.update(0.0 - current_yaw)
                                display_text = "MODE: Gap -> Gyro Straight"
                            else:
                                wall_error = WALL_FOLLOW_TARGET_MM - avg_right_wall
                                target_servo_angle = SERVO_CENTER_ANGLE - wall_follow_pid.update(wall_error)
                                display_text = f"MODE: Track Right | Err: {wall_error:.0f}mm"
                        else:
                            target_servo_angle = SERVO_CENTER_ANGLE - gyro_straight_pid.update(0.0 - current_yaw)
                else:
                    current_robot_state = RobotState.PURE_GYRO_START
                    target_servo_angle = SERVO_CENTER_ANGLE - gyro_straight_pid.update(0.0 - current_yaw)
                    display_text = f"MODE: Launch Straight | Yaw: {current_yaw:+.1f}°"

            # DISPATCH PACKETS TO PHYSICAL SERVO REGISTERS
            final_servo_angle = int(round(np.clip(
                target_servo_angle,
                SERVO_CENTER_ANGLE - STEERING_ANGLE_CLIP_DEGREES,
                SERVO_CENTER_ANGLE + STEERING_ANGLE_CLIP_DEGREES
            )))
            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

            if DEBUG_UI_OVERLAYS:
                loop_duration = time.monotonic() - loop_start_time
                fps = 1.0 / loop_duration if loop_duration > 0 else 0
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state} | Laps Passed: {turn_count}/12", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"FPS: {int(fps)} | Yaw Vector: {current_yaw:+.1f}°", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if STREAM_VIDEO:
                    with output_frame_lock:
                        output_frame = processed_frame.copy()

            time.sleep(0.02)

    except Exception as e:
        print(f"[SYSTEM FAILURE] Main loop error: {e}")
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
    return "<h3>WRO 2026 Core Clustering Active</h3><img src='/video_feed' width='100%'/>"

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("--- Booting WRO 2026 Continuous Clustering System ---")
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)