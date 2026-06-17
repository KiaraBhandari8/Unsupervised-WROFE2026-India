#centering dominates forward straight turn path(gyro turn check)

import serial
import math
import time
import ydlidar
import sys
import numpy as np

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ROBOT_SPEED = 220            # Baseline track speed
SERVO_CENTER_ANGLE = 90     # Mechanical steering center baseline alignment

# LiDAR Flank Override Clearances
LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 50
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 250

LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -50
LIDAR_LEFT_SIDE_DISTANCE_MM = 250

LIDAR_SIDE_STEER_MAGNITUDE = 15

# Global Safety Boundaries
LIDAR_SERVO_MIN_ANGLE = SERVO_CENTER_ANGLE - 25 
LIDAR_SERVO_MAX_ANGLE = SERVO_CENTER_ANGLE + 25 

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

def check_lidar_side_alerts(scan_data):
    if not scan_data: return None
    for angle, distance in scan_data.items():
        if LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_RIGHT_SIDE_DISTANCE_MM:
            return "RIGHT"
    for angle, distance in scan_data.items():
        if LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_LEFT_SIDE_DISTANCE_MM:
            return "LEFT"
    return None

def calculate_steering_error(scan_data, target_distance_mm=750):
    right_wall_angles_degrees = [angle for angle in range(30, 91)]
    left_wall_angles_degrees = [angle for angle in range(-90, -29)]

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
    
    STATE_LANE_FOLLOWING = "LANE_FOLLOWING"
    STATE_CORNERING = "CORNERING"
    active_run_state = STATE_LANE_FOLLOWING
    turn_direction = None
    baseline_start_yaw = 0.0
    current_yaw = 0.0

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] High-speed serial connection complete.")
    except Exception as e:
        print(f"[FATAL] Cannot talk to ESP32: {e}")
        sys.exit(1)

    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0)

    try:
        with LidarScanner() as scanner:
            while True:
                # 1. Asynchronously clear the input buffer to catch the latest Gyro angle from the ESP32
                while esp_ser.in_waiting > 0:
                    try:
                        raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                        if raw_line.startswith("YAW:"):
                            current_yaw = float(raw_line.split(":")[1])
                    except Exception:
                        pass

                scan_data = scanner.get_scan_data()
                if not scan_data:
                    continue

                # ====================================================
                # BRANCH A: ACTIVE CRITICAL MANEUVER STATE MACHINE (DEBUG MODE)
                # ====================================================
                if active_run_state == STATE_CORNERING:
                    yaw_delta = current_yaw - baseline_start_yaw
                    target_angle = 170 if turn_direction == "RIGHT" else 10
                    
                    # LOG TRACE: Print current angular drift status every execution loop
                    print(f"--> [STATE_CORNERING] Current Yaw: {current_yaw:.1f}° | Target Delta: {yaw_delta:.1f}°/90.0° | Sending Servo: {target_angle}°")
                    
                    # Check if the turning rotation target has been met (90 degree shift window completed)
                    if abs(yaw_delta) >= 88.0: 
                        print("\n==========================================================")
                        print(f"[STEP 4] TARGET MET! Absolute turn delta reached: {yaw_delta:.1f}°")
                        print("[STEP 5] Deploying active braking payload down to ESP32...")
                        print("==========================================================")
                        
                        # Force dead stop sequence
                        esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                        time.sleep(0.3)
                        
                        print("[STEP 6] Sending 'RST_YAW' payload to zero out ESP32 gyro registers...")
                        esp_ser.write("RST_YAW\n".encode('utf-8'))
                        time.sleep(0.1)
                        current_yaw = 0.0
                        
                        print("[STEP 7] Turn Routine Succeeded. Transitioning back to LANE_FOLLOWING.\n")
                        active_run_state = STATE_LANE_FOLLOWING
                        turn_direction = None
                    else:
                        # Continue holding execution parameters across processing cycles
                        packet = f"STR:{target_angle},SPD:{ROBOT_SPEED}\n"
                        esp_ser.write(packet.encode('utf-8'))
                    
                    time.sleep(0.05)
                    continue

                # ====================================================
                # BRANCH B: LATCHED TRACK CORRIDOR EVALUATIONS
                # ====================================================
                front_angles = [a for a in range(-10, 11)]
                front_points = [scan_data[a] for a in front_angles if a in scan_data and scan_data[a] > 0]
                avg_front_distance = sum(front_points) / len(front_points) if front_points else 2000.0

                # Check if forward distance breaches the critical 20 cm block line
                if avg_front_distance < 200.0:
                    print("\n==========================================================")
                    print(f"[STEP 1] OBSTACLE TRIGGERED! Forward space dropped to {avg_front_distance:.1f}mm")
                    print("[STEP 1] Freezing chassis movement to execute side analysis...")
                    print("==========================================================")
                    
                    # Deploy safety brakes
                    esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                    time.sleep(0.2)
                    
                    # Analyze side scanning buffers to determine the open space path
                    left_scan_angles = [a for a in range(-80, -44)]
                    right_scan_angles = [a for a in range(45, 81)]
                    
                    left_pts = [scan_data[a] for a in left_scan_angles if a in scan_data and scan_data[a] > 0]
                    right_pts = [scan_data[a] for a in right_scan_angles if a in scan_data and scan_data[a] > 0]
                    
                    avg_left_space = sum(left_pts) / len(left_pts) if left_pts else 0.0
                    avg_right_space = sum(right_pts) / len(right_pts) if right_pts else 0.0
                    
                    print(f"[STEP 2] Scan Outputs -> Avg Left: {avg_left_space:.1f}mm | Avg Right: {avg_right_space:.1f}mm")
                    
                    # Shift state configurations
                    active_run_state = STATE_CORNERING
                    baseline_start_yaw = current_yaw
                    
                    if avg_left_space < avg_right_space:
                        turn_direction = "RIGHT"
                        final_servo_angle = 170
                        print(f"[STEP 3] Action Decision: Turning RIGHT. Locking Servo to {final_servo_angle}°")
                    else:
                        turn_direction = "LEFT"
                        final_servo_angle = 10
                        print(f"[STEP 3] Action Decision: Turning LEFT. Locking Servo to {final_servo_angle}°")
                    print(f"[STEP 3] Baseline Rotation Starting Point Locked at: {baseline_start_yaw:.1f}°")
                    print("==========================================================\n")
                        
                    # Execute immediate packet dispatch to activate movement
                    packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                    esp_ser.write(packet.encode('utf-8'))
                    continue

                # ====================================================
                # BRANCH C: CORE LANE ADJUSTMENTS & FLANK OVERRIDES
                # ====================================================
                side_alert_status = check_lidar_side_alerts(scan_data)
                
                if side_alert_status == "RIGHT":
                    target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                elif side_alert_status == "LEFT":
                    target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                else: 
                    error = calculate_steering_error(scan_data, target_distance_mm=target_distance_mm)
                    pid_output = pid.update(error)
                    target_servo_angle = map_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=clockwise_mode)

                final_servo_angle = int(round(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE)))
                packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                esp_ser.write(packet.encode('utf-8'))
                
                time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n[STOP] Task execution halted manually.")
    finally:
        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
        try:
            esp_ser.write(packet.encode('utf-8'))
            esp_ser.close()
        except:
            pass
        print("[SUCCESS] Safety shutdown complete.")

if __name__ == "__main__":
    main()