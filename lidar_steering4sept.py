import numpy as np 
import serial
import struct
import math
import time
import ydlidar

# --- LiDAR Scanner Class (No Changes) ---
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
                        self.scan_data[angle_degrees] = p.range * 1000
                return self.scan_data
            return None
        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None

# --- PID Controller Class (No changes) ---
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
        output = P + I + D
        self.prev_error = current_error
        self.last_time = current_time
        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()


# --- MODIFIED STEERING ERROR CALCULATION ---
def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=150, clockwise=True):
    """
    Calculates steering error. The sensing angles are adjusted based on whether
    the robot is performing clockwise (right-turning) or anti-clockwise
    (left-turning) wall following.
    """
    front_angles_degrees = range(-5, 6) # -5 to 5 degrees inclusive

    # Set the left and right wall sensing angles based on the chosen maneuver direction
    if clockwise:
        # Clockwise (right turn priority): wider range on the right
        print("LiDAR: Clockwise mode. Right Range: [30, 110], Left Range: [-90, -30]")
        right_wall_angles_degrees = range(30, 105)  # 30 to 110
        left_wall_angles_degrees = range(-90, -29) # -90 to -30
    else:
        # Anti-Clockwise (left turn priority): wider range on the left
        print("LiDAR: Anti-Clockwise mode. Right Range: [30, 90], Left Range: [-110, -30]")
        right_wall_angles_degrees = range(30, 91)   # 30 to 90
        left_wall_angles_degrees = range(-105, -29)  # -110 to -30

    # --- Immediate Obstacle Detection (Safety) ---
    front_distances = [scan_data[angle] for angle in front_angles_degrees if angle in scan_data and scan_data[angle] > 0]
    for dist in front_distances:
        if dist < safety_distance_mm:
            print(f"LiDAR: WARNING! Obstacle at {dist:.0f}mm. Commanding STOP.")
            return 9999.0 # Signal for immediate stop

    # --- Wall Following Logic ---
    right_distances = [d for a, d in scan_data.items() if a in right_wall_angles_degrees and d is not None and 0 < d < 3000]
    left_distances = [d for a, d in scan_data.items() if a in left_wall_angles_degrees and d is not None and 0 < d < 3000]
    
    right_distances.sort(reverse=True)
    left_distances.sort(reverse=True)
    num_values = 20 # Consider the 20 furthest points to avoid noise from close objects
    top_right_distances = right_distances[:num_values]
    top_left_distances = left_distances[:num_values]

    avg_right_distance = np.mean(top_right_distances) if top_right_distances else None
    avg_left_distance = np.mean(top_left_distances) if top_left_distances else None
    
    if avg_right_distance is not None or avg_left_distance is not None:
        print(f"LiDAR: Avg Right: {avg_right_distance or 'N/A'}, Avg Left: {avg_left_distance or 'N/A'}")

    error = 0.0
    if avg_right_distance is not None and avg_left_distance is not None:
        error = avg_right_distance - avg_left_distance # Balance between two walls
    elif avg_right_distance is not None:
        error = avg_right_distance - target_distance_mm # Keep target distance from right wall
    elif avg_left_distance is not None:
        error = target_distance_mm - avg_left_distance # Keep target distance from left wall (inverted error)
    else:
        error = 0.0 # No walls detected, continue straight
        
    return error