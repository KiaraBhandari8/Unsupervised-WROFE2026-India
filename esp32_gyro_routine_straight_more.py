# no centering algorithm, just uses yaw angle 0 to go straight check for turn
# and move forward no outer wall following.

import serial
import math
import time
import ydlidar
import sys
import numpy as np

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ROBOT_SPEED = 120            # Baseline track speed sent to ESP32 (0-255)
SERVO_CENTER_ANGLE = 95      # Absolute mechanical straight midpoint center alignment

# GYRO MANEUVER TUNING
# MODIFICATION: Lowered to 80.0 to account for mechanical momentum overshoot
TURN_TARGET_DEGREES = 80.0  

# Obstacle Sight Threshold Boundaries
FRONT_TURN_TRIGGER_MM = 200.0  # Strict 20cm front trigger boundary
FRONT_SCAN_ANGLE_DEG = 15      # Width of front scan cone (+/- 15°)

# Absolute Steering Turning Targets
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0

# Hardware Limits
LIDAR_SERVO_MIN_ANGLE = 10
LIDAR_SERVO_MAX_ANGLE = 170

print("[SYSTEM] Initializing Sequential Gyro Turn Engine...")
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

def main():
    STATE_LANE_FOLLOWING = "LANE_FOLLOWING"
    STATE_CORNERING = "CORNERING"
    active_run_state = STATE_LANE_FOLLOWING
    turn_direction = None
    baseline_start_yaw = 0.0
    current_yaw = 0.0

    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] High-speed serial link established with ESP32 core.")
    except Exception as e:
        print(f"[FATAL] Connection interface dead on port {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # Gyro heading controller configuration
    gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)

    try:
        with LidarScanner() as scanner:
            while True:
                # Clear UART logs asynchronously to parsing parameters
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
                # STATE 1: ACTIVE CORNERING RUNTIME LOOP
                # ====================================================
                if active_run_state == STATE_CORNERING:
                    yaw_delta = current_yaw - baseline_start_yaw
                    target_angle = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                    
                    print(f"--> [STATE_CORNERING] Delta: {yaw_delta:.1f}° / {TURN_TARGET_DEGREES}° | Turning at Wheel Position: {target_angle}°")
                    
                    # MODIFICATION: Changed static threshold check to the variable TURN_TARGET_DEGREES (80.0)
                    if abs(yaw_delta) >= TURN_TARGET_DEGREES: 
                        print("\n==========================================================")
                        print(f"[TARGET CONFIRMED] Early braking triggered at delta: {yaw_delta:.1f}°")
                        print("[ACTION] Halting motors and centering wheels back to 95° at a stop...")
                        print("==========================================================")
                        
                        # Stop robot completely and return steering to 95° center while dead stopped
                        esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                        time.sleep(0.4)  # Halt completely to let kinetic slide settle down
                        
                        # Clear downstream spatial registers back to 0 orientation baseline
                        print("[ACTION] Dispatching register clear -> Resetting Gyro Yaw to 0°...")
                        esp_ser.write("RST_YAW\n".encode('utf-8'))
                        time.sleep(0.2)  # Delay constraint to let the firmware clear out old buffers
                        
                        current_yaw = 0.0
                        gyro_straight_pid.prev_error = 0 
                        
                        print("[STATE TRANSITION] Path cleared. Resuming forward movement into line tracking.\n")
                        active_run_state = STATE_LANE_FOLLOWING
                        turn_direction = None
                    else:
                        # Drive forward continuously maintaining the full steering wheel lock angle
                        packet = f"STR:{target_angle},SPD:{ROBOT_SPEED}\n"
                        esp_ser.write(packet.encode('utf-8'))
                    
                    time.sleep(0.05)
                    continue

                # ====================================================
                # STATE 2: RADAR FRONTAL RADIAL CONE INSPECT (20 CM LIMIT)
                # ====================================================
                front_angles = [a for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)]
                front_points = [scan_data[a] for a in front_angles if a in scan_data and scan_data[a] > 0]
                avg_front_distance = sum(front_points) / len(front_points) if front_points else 2000.0

                if avg_front_distance < FRONT_TURN_TRIGGER_MM:
                    print("\n==========================================================")
                    print(f"[CRITICAL TRIPWIRE] Distance breach detected at: {avg_front_distance:.1f}mm")
                    print("[ACTION] Bringing chassis to emergency standstill...")
                    print("==========================================================")
                    
                    # Deploy safety brakes completely 
                    esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                    time.sleep(0.3)  # Let the car come to a completely dead physical stop
                    
                    # Run spatial comparison sweep routines
                    left_scan_angles = [a for a in range(-80, -44)]
                    right_scan_angles = [a for a in range(45, 81)]
                    
                    left_pts = [scan_data[a] for a in left_scan_angles if a in scan_data and scan_data[a] > 0]
                    right_pts = [scan_data[a] for a in right_scan_angles if a in scan_data and scan_data[a] > 0]
                    
                    avg_left_space = sum(left_pts) / len(left_pts) if left_pts else 0.0
                    avg_right_space = sum(right_pts) / len(right_pts) if right_pts else 0.0
                    
                    print(f"[SCAN COMPLETED] Left Clearance: {avg_left_space:.1f}mm | Right Clearance: {avg_right_space:.1f}mm")
                    
                    if avg_left_space < avg_right_space:
                        turn_direction = "RIGHT"
                        final_servo_angle = SERVO_HARD_RIGHT
                    else:
                        turn_direction = "LEFT"
                        final_servo_angle = SERVO_HARD_LEFT
                        
                    # Shift steering configuration over to full wheel lock while SPEED stays zero
                    print(f"[PRE-TURN ACTUATION] Pivoting servo to {final_servo_angle}° at a dead stop...")
                    pivot_packet = f"STR:{final_servo_angle},SPD:0\n"
                    esp_ser.write(pivot_packet.encode('utf-8'))
                    time.sleep(0.4)  # Allow mechanical links to lock fully before tracking movement
                    
                    # Reset the yaw register to 0 right here to clear the tire scrub vibration error
                    print("[MODIFICATION] Wiping steering-shake error -> Resetting Gyro Yaw to 0°...")
                    esp_ser.write("RST_YAW\n".encode('utf-8'))
                    time.sleep(0.1)  # Brief sleep to let the ESP32 zero out its global float variable
                    
                    # Update local tracking variables to begin the turn from absolute clean zeroes
                    current_yaw = 0.0
                    baseline_start_yaw = 0.0
                    active_run_state = STATE_CORNERING
                    
                    # Engage motors now that wheels are locked and gyro registers are zeroed out
                    print(f"[PROPULSION ENGAGED] Commencing driving turn at speed: {ROBOT_SPEED}")
                    print("==========================================================\n")
                    drive_packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                    esp_ser.write(drive_packet.encode('utf-8'))
                    continue

                # ====================================================
                # STATE 3: INERTIAL GUIDANCE LOCKWAY LOOP (LANE FOLLOWING)
                # ====================================================
                heading_error = 0.0 - current_yaw
                pid_output = gyro_straight_pid.update(heading_error)
                
                # INVERSE COMPENSATE CORRECTION FIX: Inverted minus assignment applied
                target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                
                final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                packet = f"STR:{final_servo_angle},SPD:{ROBOT_SPEED}\n"
                esp_ser.write(packet.encode('utf-8'))
                
                print(f"[STATE_LANE_FOLLOWING] Yaw: {current_yaw:.2f}° | Err: {heading_error:.2f}° | Output Steering: {final_servo_angle}°")
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[STOP] Script termination execution manual command caught.")
    finally:
        packet = f"STR:{SERVO_CENTER_ANGLE},SPD:0\n"
        try:
            esp_ser.write(packet.encode('utf-8'))
            esp_ser.close()
        except:
            pass
        print("[CLEANUP] Core systems isolated safely.")

if __name__ == "__main__":
    main()