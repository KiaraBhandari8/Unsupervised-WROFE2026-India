import serial
import struct
import math
import time
import ydlidar

# --- LiDAR Scanner Class ---
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

            ret = self.laser.initialize()
            if not ret:
                raise IOError(f"LiDAR connection failed: {self.laser.DescribeError()}")

            ret = self.laser.turnOn()
            if not ret:
                raise IOError(f"Failed to turn on YDLIDAR: {self.laser.DescribeError()}")

            print(f"LiDAR: Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            print(f"LiDAR ERROR: Could not connect to LiDAR: {e}")
            raise IOError(f"LiDAR connection failed: {e}")

    def disconnect(self):
        if self.laser:
            print("LiDAR: Disconnecting...")
            self.laser.turnOff()
            self.laser.disconnecting()
            self.laser = None
            print("LiDAR: Disconnected.")

    def get_scan_data(self):
        if not self.laser:
            return None

        self.scan_data = {}
        scan = ydlidar.LaserScan()

        try:
            r = self.laser.doProcessSimple(scan)
            if r:
                for p in scan.points:
                    if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                        angle_degrees = round(math.degrees(p.angle))
                        distance_mm = p.range * 1000
                        self.scan_data[angle_degrees] = distance_mm
                return self.scan_data
            else:
                return None
        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None


# --- PID Controller Class (No changes needed) ---
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

        if dt <= 0:
            return self.prev_error

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


# --- Steering Error Calculation (Angle Ranges Review) ---
def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=150):
    """
    Calculates the steering error based on LiDAR scan data to keep the robot
    at a target distance from the walls.

    Args:
        scan_data (dict): Dictionary of angle:distance_mm from LiDAR (angles in degrees).
        target_distance_mm (int): The desired distance from the wall in millimeters.
        safety_distance_mm (int): If any point is closer than this, consider it an immediate obstacle.

    Returns:
        float: The steering error. Positive implies turn right, negative implies turn left.
               Returns a large error if an immediate obstacle is detected.
    """

    # Define angular ranges for left, right, and front detection
    # Assuming 0 degrees is directly forward with YDLIDAR T-mini Plus.
    # Angles typically increase counter-clockwise (-180 to 180 degrees).

    # Front sector (for obstacle detection / general clearance)
    # This range is good for typical forward obstacles.
    front_angles_degrees = [angle for angle in range(-5, 5)] # From -15 to +15 degrees

    # Right wall sensing sector
    # If 0 is robot front, 90 is directly right. So, range like 30 to 90 or 45 to 90.
    # The current range (30 to 91) looks reasonable if you want to pick up points directly to the side and slightly forward.
    right_wall_angles_degrees = [angle for angle in range(30,91)]
    #  obs (1,110)

    # Left wall sensing sector
    # If 0 is robot front, -90 is directly left. So, range like -90 to -30 or -90 to -45.
    # The current range (-90 to -29) looks reasonable.
    left_wall_angles_degrees = [angle for angle in range(-90,-29 )]
    #  obs (-90, 0)


    # --- Immediate Obstacle Detection (Safety) ---
    for angle in front_angles_degrees:
        if angle in scan_data and scan_data[angle] is not None and scan_data[angle] < safety_distance_mm and scan_data[angle] > 0:
            print(f"LiDAR: WARNING! Obstacle detected at {angle} deg, {scan_data[angle]:.0f}mm. Commanding full stop/re-evaluation.")
            return 9999.0 # Large error to trigger immediate action (e.g., stop or emergency turn)

    # --- Wall Following Logic ---
    right_distances = [scan_data[angle] for angle in right_wall_angles_degrees if angle in scan_data and scan_data[angle] is not None and scan_data[angle] > 0]
    left_distances = [scan_data[angle] for angle in left_wall_angles_degrees if angle in scan_data and scan_data[angle] is not None and scan_data[angle] > 0]

    avg_right_distance = sum(right_distances) / len(right_distances) if right_distances else None
    avg_left_distance = sum(left_distances) / len(left_distances) if left_distances else None
    print(f"LiDAR: Avg Right Distance: {avg_right_distance:.0f}mm, Avg Left Distance: {avg_left_distance:.0f}mm")

    error = 0.0

    if avg_right_distance is not None and avg_left_distance is not None:
        # Both walls detected, try to center
        # Error: (distance to left wall) - (distance to right wall)
        # If left wall is further (positive error), robot is closer to right, needs to steer LEFT (decrease angle from 90).
        # If right wall is further (negative error), robot is closer to left, needs to steer RIGHT (increase angle from 90).
        # This error definition will need to be handled carefully by the PID and map_steering_angle based on clockwise_mode.
        # My map_steering_angle logic assumes:
        #   - Clockwise (right wall): Positive PID output -> steer right (angle decreases)
        #   - Anticlockwise (left wall): Positive PID output -> steer left (angle increases)
        # Let's adjust `error` calculation to be consistent with the desired steering direction directly.
        
        # If aiming to center, positive error means "robot needs to move more towards left wall"
        # and negative error means "robot needs to move more towards right wall"
        error = avg_right_distance - avg_left_distance # If right is further than left, positive error -> need to move right.
        print(f"LiDAR: Both walls. Left: {avg_left_distance:.0f}, Right: {avg_right_distance:.0f}. Error: {error:.0f}mm (Right-Left)")


    elif avg_right_distance is not None:
        # Only right wall detected, try to maintain target distance from it
        # If avg_right_distance > target_distance_mm, error is positive (too far from right wall).
        # Robot needs to move RIGHT (closer to wall).
        error = avg_right_distance - target_distance_mm
        print(f"LiDAR: Right wall only. Dist: {avg_right_distance:.0f}. Error: {error:.0f}mm (Right-Target)")

    elif avg_left_distance is not None:
        # Only left wall detected, try to maintain target distance from it
        # If avg_left_distance > target_distance_mm, error is positive (too far from left wall).
        # Robot needs to move LEFT (closer to wall).
        # We need the error to have the same sign convention for the PID.
        # So, if too far from left wall (left_dist > target), `left_dist - target` is positive.
        # This implies robot needs to steer LEFT (angle increases).
        # If too close to left wall (left_dist < target), `left_dist - target` is negative.
        # This implies robot needs to steer RIGHT (angle decreases).
        # Let's keep this sign:
        error = avg_left_distance - target_distance_mm
        print(f"LiDAR: Left wall only. Dist: {avg_left_distance:.0f}. Error: {error:.0f}mm (Left-Target)")

    else:
        # No walls detected (e.g., middle of arena, or at a corner entrance)
        print("LiDAR: No walls detected. Maintaining course (error=0).")
        error = 0.0
    
    # IMPORTANT: The sign of the error output by this function
    # must be consistent with how `map_steering_angle` expects it.
    # Let's re-confirm this.
    # For clockwise (right wall following):
    #   - If `error` from `calculate_steering_error` is positive (robot too far from right wall),
    #     PID output will be positive.
    #     `map_steering_angle` uses `center_angle - adjusted_output`.
    #     So, `90 - (positive value)` will decrease the angle, steering RIGHT. This is correct.
    # For anticlockwise (left wall following):
    #   - If `error` from `calculate_steering_error` is positive (robot too far from left wall),
    #     PID output will be positive.
    #     `map_steering_angle` uses `center_angle + adjusted_output`.
    #     So, `90 + (positive value)` will increase the angle, steering LEFT. This is correct.

    # Therefore, the error definitions as currently implemented in calculate_steering_error
    # work correctly with the map_steering_angle logic.

    return error