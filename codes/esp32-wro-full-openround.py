import serial
import math
import time
import ydlidar
import sys
import cv2
import numpy as np
from picamera2 import Picamera2

# --- TUNED RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ROBOT_SPEED = 90           
SERVO_CENTER_ANGLE = 97   

# LiDAR Flank Obstacle Avoidance Margins
LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 50
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 250

LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -50
LIDAR_LEFT_SIDE_DISTANCE_MM = 250

LIDAR_SIDE_STEER_MAGNITUDE = 15

# Servo Physical Clamping Limits
LIDAR_SERVO_MIN_ANGLE = SERVO_CENTER_ANGLE - 25
LIDAR_SERVO_MAX_ANGLE = SERVO_CENTER_ANGLE + 25

# --- Native YDLIDAR Driver Interface ---
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

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

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
            self.laser.initialize()
            self.laser.turnOn()
            print(f"[LIDAR] Driver actively scanning on port: {self.port}")
        except Exception as e:
            print(f"[LIDAR ERROR] Initialization failure: {e}")
            raise IOError()

    def get_scan_data(self):
        if not self.laser: return None
        self.scan_data = {}
        scan = ydlidar.LaserScan()
        if self.laser.doProcessSimple(scan):
            for p in scan.points:
                if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                    angle_deg = round(math.degrees(p.angle))
                    self.scan_data[angle_deg] = p.range * 1000.0
            return self.scan_data
        return None

    def disconnect(self):
        if self.laser:
            self.laser.turnOff()
            self.laser.disconnecting()

# --- Highly Configurable PID Mathematical Processing Core ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
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

# --- Layered Obstacle & Corridor Analysis Engines ---
def check_lidar_side_alerts(scan_data):
    if not scan_data: return None
    for angle, distance in scan_data.items():
        if LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_RIGHT_SIDE_DISTANCE_MM:
            return "RIGHT"
    for angle, distance in scan_data.items():
        if LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_LEFT_SIDE_DISTANCE_MM:
            return "LEFT"
    return None

def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=150):
    front_angles_degrees = [angle for angle in range(-5, 5)]
    right_wall_angles_degrees = [angle for angle in range(30, 91)]
    left_wall_angles_degrees = [angle for angle in range(-90, -29)]

    for angle in front_angles_degrees:
        if angle in scan_data and scan_data[angle] is not None and 0 < scan_data[angle] < safety_distance_mm:
            return 9999.0 

    right_distances = [scan_data[angle] for angle in right_wall_angles_degrees if angle in scan_data and scan_data[angle] > 0]
    left_distances = [scan_data[angle] for angle in left_wall_angles_degrees if angle in scan_data and scan_data[angle] > 0]

    avg_right_distance = sum(right_distances) / len(right_distances) if right_distances else None
    avg_left_distance = sum(left_distances) / len(left_distances) if left_distances else None

    if avg_right_distance is not None and avg_left_distance is not None:
        error = avg_right_distance - avg_left_distance 
    elif avg_right_distance is not None:
        error = avg_right_distance - target_distance_mm
    elif avg_left_distance is not None:
        error = avg_left_distance - target_distance_mm
    else:
        error = 0.0

    return float(error)

def check_front_clearance(scan_data, min_angle, max_angle, target_distance_min=1100, target_distance_max=1500):
    if not scan_data: return False

    front_left_distances = [scan_data[deg] for deg in range(min_angle, 1) if deg in scan_data and scan_data[deg] > 0]
    front_right_distances = [scan_data[deg] for deg in range(0, max_angle + 1) if deg in scan_data and scan_data[deg] > 0]

    if not front_left_distances or not front_right_distances:
        return False

    avg_left = np.mean(front_left_distances)
    avg_right = np.mean(front_right_distances)

    print(f"[STOP AUDIT] Avg Front Left: {avg_left:.1f}mm | Avg Front Right: {avg_right:.1f}mm")

    if target_distance_min < avg_left < target_distance_max:
        if target_distance_min < avg_right < target_distance_max:
            return True
            
    return False

# --- Color Filtering Functions ---
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

def map_steering_angle(center_angle, pid_output, clockwise=True):
    scale_factor = 0.2
    adjusted_output = -1 * (pid_output * scale_factor)
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    return int(max(center_angle - 20, min(angle, center_angle + 20)))

def main():
    print("=== Launching Vision + LiDAR Integrated Open Round Logic ===")
    clockwise_mode = True  
    target_distance_mm = 750
    safety_distance_mm = 150
    max_color_count = 12

    LIDAR_STOP_MIN_ANGLE = -5
    LIDAR_STOP_MAX_ANGLE = 5
    crossed_12 = False

    # Line crossing state track variables
    blue_count = 0
    orange_count = 0
    prev_blue_state = False
    prev_orange_state = False
    blue_cooldown_end_time = 0.0
    orange_cooldown_end_time = 0.0

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=1)
        print("[INFO] Serial bus pipe connected to ESP32 actuator layer.")
    except Exception as e:
        print(f"[FATAL] Cannot establish connection to port {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # Instantiate Camera Module 3 Wide Engine
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (768, 432)})
    picam2.configure(config)
    picam2.start()
    print("[SUCCESS] Camera Module 3 Wide stream thread online.")
    time.sleep(2.0) # Exposure settling delay

    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) if clockwise_mode else PIDController(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)

    try:
        with LidarScanner() as scanner:
            while True:
                current_time = time.time()
                
                # Grab hardware sensor data inputs
                scan_data = scanner.get_scan_data()
                frame = picam2.capture_array()
                
                if not scan_data or frame is None:
                    continue

                # Run Color Extraction Matrices
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

                blue_mask = filter_blue_objects(hsv)
                orange_mask = filter_orange_objects(hsv)
                
                blue_in_view = detect_color_binary(blue_mask)
                orange_in_view = detect_color_binary(orange_mask)

                # Blue Tape Tracking Pipeline
                if not blue_in_view and prev_blue_state:
                    if current_time > blue_cooldown_end_time:
                        blue_count += 1
                        print(f"\n[CAMERA] Blue line cleared! Total Blue Lines: {blue_count}")
                        blue_cooldown_end_time = current_time + 3.0
                prev_blue_state = blue_in_view

                # Orange Tape Tracking Pipeline
                if not orange_in_view and prev_orange_state:
                    if current_time > orange_cooldown_end_time:
                        orange_count += 1
                        print(f"\n[CAMERA] Orange line cleared! Total Orange Lines: {orange_count}")
                        orange_cooldown_end_time = current_time + 3.0
                prev_orange_state = orange_in_view

                print(f"[TELEMETRY] Blue Count: {blue_count} | Orange Count: {orange_count} | Target: {max_color_count}")

                # --- LAYER 1: LAP CLOSURE SWITCH ENGINE ---
                if not crossed_12 and (blue_count >= max_color_count and orange_count >= max_color_count):
                    print("\n[GOAL REACHED] All 12 line crossings completed! Scanning for stopping envelope...")
                    crossed_12 = True
                    blue_count = 0
                    orange_count = 0

                # --- LAYER 2: RACE TERMINATOR ARREST SEQUENCE ---
                if crossed_12:
                    if check_front_clearance(scan_data, LIDAR_STOP_MIN_ANGLE, LIDAR_STOP_MAX_ANGLE, target_distance_min=1100, target_distance_max=1500):
                        if SERVO_CENTER_ANGLE - 5 <= final_servo_angle <= SERVO_CENTER_ANGLE + 5:
                            print("\n[RACE COMPLETE] Straightaway corridor center reached. Brake deployed.")
                            packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
                            esp_ser.write(packet.encode('utf-8'))
                            break # Terminate script execution completely

                # --- LAYER 3: HARDWARE ACTUATION & NAVIGATION ---
                side_alert_status = check_lidar_side_alerts(scan_data)
                
                if side_alert_status == "RIGHT":
                    target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                elif side_alert_status == "LEFT":
                    target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                else: 
                    error = calculate_steering_error(scan_data, target_distance_mm=target_distance_mm, safety_distance_mm=safety_distance_mm)
                    
                    if error == 9999.0:
                        print("[EMERGENCY BRAKE] Target block inside collision zone! Pausing.")
                        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
                        esp_ser.write(packet.encode('utf-8'))
                        time.sleep(0.2)
                        continue
                        
                    pid_output = pid.update(error)
                    target_servo_angle = map_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=clockwise_mode)

                final_servo_angle = int(round(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE)))
                
                # Transmit actuation data packet down to the ESP32 registers
                packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                esp_ser.write(packet.encode('utf-8'))
                
                time.sleep(0.15) # 150ms evaluation frame frequency rate matching last year's code configuration

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard termination flag thrown.")
    finally:
        print("[CLEANUP] Halting motors and dropping camera locks...")
        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
        try:
            esp_ser.write(packet.encode('utf-8'))
            esp_ser.close()
            picam2.close()
        except:
            pass
        print("[CLEANUP] Safety shutdown complete.")

if __name__ == "__main__":
    main()