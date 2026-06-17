import serial
import math
import time
import ydlidar
import sys
import numpy as np

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ROBOT_SPEED = 220            # Your baseline operational speed for the ESP32 (0-255 scaling)
SERVO_CENTER_ANGLE = 90     # Your structural mechanical steering alignment center

# LiDAR Flank Obstacle Avoidance Margins
LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 50
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 250

LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -50
LIDAR_LEFT_SIDE_DISTANCE_MM = 250

LIDAR_SIDE_STEER_MAGNITUDE = 15

# Servo Clamping Margins
LIDAR_SERVO_MIN_ANGLE = SERVO_CENTER_ANGLE - 25
LIDAR_SERVO_MAX_ANGLE = SERVO_CENTER_ANGLE + 25

print("[SYSTEM] Initializing high-level navigation tracking array...")
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
            print(f"[LIDAR] Scanning actively on link: {self.port}")
        except Exception as e:
            print(f"[LIDAR ERROR] Driver instantiation failure: {e}")
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

def map_steering_angle(center_angle, pid_output, clockwise=True):
    scale_factor = 0.2
    adjusted_output = -1 * (pid_output * scale_factor)
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    return int(max(center_angle - 20, min(angle, center_angle + 20)))

def main():
    clockwise_mode = True  
    target_distance_mm = 750
    safety_distance_mm = 150

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=1)
        print("[INFO] Hardware communications established with the ESP32 execution layer.")
    except Exception as e:
        print(f"[FATAL] Connection failed on serial channel {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) if clockwise_mode else PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0)

    try:
        with LidarScanner() as scanner:
            while True:
                scan_data = scanner.get_scan_data()
                if not scan_data:
                    continue

                side_alert_status = check_lidar_side_alerts(scan_data)
                
                if side_alert_status == "RIGHT":
                    target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                elif side_alert_status == "LEFT":
                    target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                else: 
                    error = calculate_steering_error(scan_data, target_distance_mm=target_distance_mm, safety_distance_mm=safety_distance_mm)
                    
                    if error == 9999.0:
                        print("[EMERGENCY] Object encountered in central runway zone! Stopping.")
                        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
                        esp_ser.write(packet.encode('utf-8'))
                        time.sleep(0.5)
                        continue
                        
                    pid_output = pid.update(error)
                    target_servo_angle = map_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=clockwise_mode)

                final_servo_angle = int(round(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE)))
                
                # TRANSMIT PACKET DIRECTLY DOWN TO THE ESP32
                packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                esp_ser.write(packet.encode('utf-8'))
                
                print(f"[RUNNING] Sent -> Servo Angle: {final_servo_angle}° | Speed Value: {ROBOT_SPEED}")
                time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n[STOP] User manual command issued.")
    finally:
        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
        esp_ser.write(packet.encode('utf-8'))
        esp_ser.close()
        print("[SUCCESS] Safety shutdown completed.")

if __name__ == "__main__":
    main()