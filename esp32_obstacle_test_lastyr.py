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
    from lidar_steering4sept import LidarScanner, PIDController, calculate_steering_error
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
WALL_LOSS_THRESHOLD_MM = 350.0  # Open pocket validation limit to ignore missing walls
CLOCKWISE_WALL_FOLLOWING = True  # Dynamically modified tracking direction flag

# Configurations for the close-range side panic state
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180  # 18cm side distance panic limit
LIDAR_LEFT_SIDE_DISTANCE_MM = 180   # 18cm side distance panic limit
LIDAR_SIDE_STEER_MAGNITUDE = 15     # Fixed steering shift magnitude away from side walls

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

latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

app = Flask(__name__)

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- CONTROL DESIGN CONSTANTS (8-BIT EXECUTION LAYER) ---
SERVO_CENTER_ANGLE = 95       # Absolute mechanical steering straight alignment midpoint
ROBOT_SPEED = 120            # Baseline track speed variable lookup configuration
ROBOT_CRUISE_SPEED = 160      # Operational forward driving speed sent to ESP32 (0-255)
ROBOT_MANEUVER_SPEED = 150    # Slowdown velocity used across complex evasion arcs

# --- NEW: INDEPENDENT VISION CALIBRATION PARAMETERS ---
STEERING_GAIN_GREEN = 0.1     # Baseline multiplier that keeps Green working perfectly
STEERING_GAIN_RED = 0.14      # INCREASE THIS to make the steering more aggressive for Red
RED_CLEARANCE_OFFSET = 8      # Static angular nudge (in degrees) to push the chassis wider right

# Gyro Turning Constants
TURN_TARGET_DEGREES = 80.0    # Braking trigger value to counteract kinetic momentum slip
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0

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
    PURE_GYRO_START = "PURE_GYRO_START"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    VISION_OBSTACLE_AVOIDANCE = "VISION_OBSTACLE_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    SEQUENTIAL_CORNERING = "SEQUENTIAL_CORNERING"
    LAP_TERMINATION = "LAP_TERMINATION"
    STOP = "STOP"

current_robot_state = RobotState.INITIALIZING
current_yaw = 0.0

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

    # Initialize Controller Mathematics Loops
    gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
    wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)

    # Set Initial Behavioral States
    current_robot_state = RobotState.PURE_GYRO_START
    turn_count = 0
    baseline_start_yaw = 0.0
    turn_direction = None
    
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
            if current_robot_state == RobotState.SEQUENTIAL_CORNERING:
                yaw_delta = current_yaw - baseline_start_yaw
                target_angle = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                
                print(f"--> [STATE_CORNERING] Delta: {yaw_delta:.1f}° / {TURN_TARGET_DEGREES}° | Servo: {target_angle}°")
                
                if abs(yaw_delta) >= TURN_TARGET_DEGREES:
                    print("\n==========================================================")
                    print(f"[TARGET MET] Turn complete at total delta: {yaw_delta:.1f}°")
                    print("==========================================================")
                    send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                    time.sleep(0.4)
                    
                    print("[ACTION] Clearing registers -> Setting Gyro baseline to 0°...")
                    esp_ser.write(b"RST_YAW\n")
                    esp_ser.flush()
                    time.sleep(0.2)
                    
                    current_yaw = 0.0
                    gyro_straight_pid.reset()
                    wall_follow_pid.reset()
                    
                    turn_count += 1
                    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                    turn_direction = None
                else:
                    send_esp_packet(esp_ser, target_angle, ROBOT_CRUISE_SPEED)
                
                time.sleep(0.05)
                continue

            # ====================================================
            # PRIORITY LEVEL 2: RADAR FRONTAL TRIPWIRE CHECK
            # ====================================================
            front_angles = range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
            front_points = [scan_data[a] for a in front_angles if a in scan_data and scan_data[a] > 0]
            avg_front_distance = sum(front_points) / len(front_points) if front_points else 2000.0

            if avg_front_distance < FRONT_TURN_TRIGGER_MM:
                print(f"\n[CORNER INTERSECTION] Front Wall at: {avg_front_distance:.1f}mm. Stopping chassis...")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                time.sleep(0.3)
                
                left_scan_angles = range(-80, -44)
                right_scan_angles = range(45, 81)
                
                left_pts = [scan_data[a] for a in left_scan_angles if a in scan_data and scan_data[a] > 0]
                right_pts = [scan_data[a] for a in right_scan_angles if a in scan_data and scan_data[a] > 0]
                
                avg_left_space = sum(left_pts) / len(left_pts) if left_pts else 0.0
                avg_right_space = sum(right_pts) / len(right_pts) if right_pts else 0.0
                
                if avg_left_space < avg_right_space:
                    turn_direction = "RIGHT"
                    final_servo_angle = SERVO_HARD_RIGHT
                    if turn_count == 0:
                        CLOCKWISE_WALL_FOLLOWING = True
                        print("[LAYOUT LOCKDOWN] Track direction set to: CLOCKWISE (CW)")
                else:
                    turn_direction = "LEFT"
                    final_servo_angle = SERVO_HARD_LEFT
                    if turn_count == 0:
                        CLOCKWISE_WALL_FOLLOWING = False
                        print("[LAYOUT LOCKDOWN] Track direction set to: COUNTER-CLOCKWISE (CCW)")
                
                print(f"[PRE-TURN ACTUATION] Pivoting wheels to {final_servo_angle}° while stopped...")
                send_esp_packet(esp_ser, final_servo_angle, 0)
                time.sleep(0.4)
                
                print("[ACTION] Flushing scrub vibration error -> Resetting Gyro Yaw to 0°...")
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                time.sleep(0.1)
                
                current_yaw = 0.0
                baseline_start_yaw = 0.0
                current_robot_state = RobotState.SEQUENTIAL_CORNERING
                
                send_esp_packet(esp_ser, final_servo_angle, ROBOT_CRUISE_SPEED)
                continue

            # ====================================================
            # PRIORITY LEVEL 3: PROXIMITY CRITICAL WALL OVERRIDES
            # ====================================================
            side_alert = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, safety_distance_mm=150, clockwise=CLOCKWISE_WALL_FOLLOWING)
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
                is_near_field_mode = avg_front_distance < 1100.0
                processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                    frame_bgr, use_outer_roi_and_bottom_point=is_near_field_mode
                )
                vision_angle = -1 * vision_angle

                if logic_label in ["red_obstacle", "obstacle"]:
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    
                    # MODIFICATION: Split Red vs Green processing blocks to handle asymmetric steering weights
                    if logic_label == "red_obstacle":
                        servo_adjust = -vision_angle * STEERING_GAIN_RED
                        # Apply the steering scale along with a positive clearance bias to widen right-hand arcs
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust + RED_CLEARANCE_OFFSET
                        display_text = f"MODE: Red Avoid | Steer: {int(target_servo_angle)}°"
                    else:
                        # Standard Green avoidance loop handles left-hand transitions cleanly
                        servo_adjust = -vision_angle * STEERING_GAIN_GREEN
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                        display_text = f"MODE: Green Avoid | Steer: {int(target_servo_angle)}°"
                
                # ====================================================
                # PRIORITY LEVEL 5: TRACK DRIVING NAVIGATION (DEFAULT RUN)
                # ====================================================
                else:
                    robot_speed_current = ROBOT_CRUISE_SPEED
                    
                    if turn_count >= 1 and CLOCKWISE_WALL_FOLLOWING is not None:
                        current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                        if CLOCKWISE_WALL_FOLLOWING:
                            left_follow_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
                            if left_follow_pts:
                                avg_left_wall = sum(left_follow_pts) / len(left_follow_pts)
                                if avg_left_wall > WALL_LOSS_THRESHOLD_MM:
                                    heading_error = 0.0 - current_yaw
                                    pid_output = gyro_straight_pid.update(heading_error)
                                    target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                    display_text = "MODE: Wall Lost Fallback (Gyro Straight)"
                                else:
                                    wall_error = avg_left_wall - WALL_FOLLOW_TARGET_MM
                                    pid_output = wall_follow_pid.update(wall_error)
                                    target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                    display_text = f"MODE: Follow Outer Left | Err: {wall_error:.0f}mm"
                            else:
                                heading_error = 0.0 - current_yaw
                                pid_output = gyro_straight_pid.update(heading_error)
                                target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                        else:
                            right_follow_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
                            if right_follow_pts:
                                avg_right_wall = sum(right_follow_pts) / len(right_follow_pts)
                                if avg_right_wall > WALL_LOSS_THRESHOLD_MM:
                                    heading_error = 0.0 - current_yaw
                                    pid_output = gyro_straight_pid.update(heading_error)
                                    target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                    display_text = "MODE: Wall Lost Fallback (Gyro Straight)"
                                else:
                                    wall_error = WALL_FOLLOW_TARGET_MM - avg_right_wall
                                    pid_output = wall_follow_pid.update(wall_error)
                                    target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                    display_text = f"MODE: Follow Outer Right | Err: {wall_error:.0f}mm"
                            else:
                                heading_error = 0.0 - current_yaw
                                pid_output = gyro_straight_pid.update(heading_error)
                                target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                    else:
                        current_robot_state = RobotState.PURE_GYRO_START
                        heading_error = 0.0 - current_yaw
                        pid_output = gyro_straight_pid.update(heading_error)
                        target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                        display_text = f"MODE: Start Gyro Straight | Yaw: {current_yaw:.1f}°"

            # 5. Output packets to hardware layers
            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

            # Frame serving calculations
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
    print("--- Booting WRO 2026 Unified Obstacle Round System ---")
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)