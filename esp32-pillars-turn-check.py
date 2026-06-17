import serial
import math
import time
import ydlidar
import sys
import cv2
import numpy as np
from picamera2 import Picamera2

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ROBOT_SPEED = 220            # Operational velocity sent to ESP32 (0-255 scaling)
SERVO_CENTER_ANGLE = 90     # Structural mechanical steering center alignment

# HARDCODED OBSTACLE EVASION ANGLES 
# UPDATED: Set explicitly to your exact mechanical target requirements
OBSTACLE_AVOID_RIGHT = 120  # Hard right pivot angle for Red obstacles
OBSTACLE_AVOID_LEFT = 60    # Hard left pivot angle for Green obstacles

# Servo Hardware Clamping Limits 
# UPDATED: Expanded constraints to 60° and 120° so the clip doesn't block your turns
LIDAR_SERVO_MIN_ANGLE = 60
LIDAR_SERVO_MAX_ANGLE = 120

print("[SYSTEM] Initializing pure camera avoidance + dual-wall centering script...")
time.sleep(2)

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
            print(f"[LIDAR] Driver actively streaming data on: {self.port}")
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

# --- PILLARS MULTI-BAND HSV FILTERS ---
def filter_red_obstacle(hsv_frame):
    lower_red1 = np.array([0, 100, 50])
    upper_red1 = np.array([12, 255, 255])
    mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    
    lower_red2 = np.array([165, 100, 50])
    upper_red2 = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    
    combined_mask = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((5, 5), np.uint8)
    combined_mask = cv2.erode(combined_mask, kernel, iterations=2)
    return cv2.dilate(combined_mask, kernel, iterations=2)

def filter_green_obstacle(hsv_frame):
    lower_green = np.array([32, 100, 40])
    upper_green = np.array([95, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    return cv2.dilate(mask, kernel, iterations=2)

def detect_color_binary(mask, threshold=3000):
    return cv2.countNonZero(mask) > threshold

def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=150):
    front_angles_degrees = [angle for angle in range(-5, 5)]
    right_wall_angles_degrees = [angle for angle in range(30, 91)]
    left_wall_angles_degrees = [angle for angle in range(-90, -29)]

    # Emergency Brake Gate
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

def map_steering_angle(center_angle, pid_output, clockwise=True):
    scale_factor = 0.2
    adjusted_output = -1 * (pid_output * scale_factor)
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    # Keeps PID corrections inside a safe +/- 20 degree tracking window around center
    return int(max(center_angle - 20, min(angle, center_angle + 20)))

def main():
    clockwise_mode = True  
    target_distance_mm = 750
    safety_distance_mm = 150

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=1)
        print("[INFO] Hardware UART communications active with ESP32.")
    except Exception as e:
        print(f"[FATAL] Connection failed on serial channel {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # Boot up Camera Module 3 Wide
    picam2 = Picamera2()
    camera_config = picam2.create_video_configuration(main={"size": (768, 432)})
    picam2.configure(camera_config)
    picam2.start()
    print("[SUCCESS] Video stream active.")
    time.sleep(2.0)

    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) if clockwise_mode else PIDController(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)

    try:
        with LidarScanner() as scanner:
            while True:
                scan_data = scanner.get_scan_data()
                frame = picam2.capture_array()
                
                if not scan_data or frame is None:
                    continue

                # Vision Framing Conversions
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

                # Execute Pillar Segmentation
                red_mask = filter_red_obstacle(hsv)
                green_mask = filter_green_obstacle(hsv)
                
                red_detected = detect_color_binary(red_mask)
                green_detected = detect_color_binary(green_mask)

                # ====================================================
                # HIERARCHY TIER 1: ACTIVE CAMERA OBSTACLE AVOIDANCE
                # ====================================================
                if red_detected:
                    target_servo_angle = OBSTACLE_AVOID_RIGHT
                    print(f"[VISUAL MODE] Red Pillar Active -> Steering RIGHT: {target_servo_angle}°")
                
                elif green_detected:
                    target_servo_angle = OBSTACLE_AVOID_LEFT
                    print(f"[VISUAL MODE] Green Pillar Active -> Steering LEFT: {target_servo_angle}°")

                # ====================================================
                # HIERARCHY TIER 2: PURE COMPARATIVE WALL CENTERING
                # ====================================================
                else: 
                    error = calculate_steering_error(scan_data, target_distance_mm=target_distance_mm, safety_distance_mm=safety_distance_mm)
                    
                    if error == 9999.0:
                        print("[SAFETY STOP] Bumper clearance threshold breached! Braking.")
                        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
                        esp_ser.write(packet.encode('utf-8'))
                        time.sleep(0.5)
                        continue
                        
                    pid_output = pid.update(error)
                    target_servo_angle = map_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=clockwise_mode)

                # Clip constraints and transmit packet to ESP32
                final_servo_angle = int(round(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE)))
                packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                esp_ser.write(packet.encode('utf-8'))
                
                time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n[STOP] User manual shutdown issued.")
    finally:
        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
        try:
            esp_ser.write(packet.encode('utf-8'))
            esp_ser.close()
            picam2.close()
        except:
            pass
        print("[SUCCESS] Safety shutdown complete.")

if __name__ == "__main__":
    main()