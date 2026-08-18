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
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
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

def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=150, clockwise=True, verbose=False):
    """
    Calculates steering error. The sensing angles are adjusted based on whether
    the robot is performing clockwise (right-turning) or anti-clockwise
    (left-turning) wall following.

    FIX: this function is called once per main-loop iteration (30-50Hz).
    Debug prints are gated behind `verbose` — pass in the caller's own
    throttled `should_log` flag, not True/False every frame, or you'll
    reintroduce the unthrottled-print FPS drop (~3 FPS) seen previously.

    The safety-stop warning print is deliberately NOT gated by `verbose` -
    it's rare (only fires when something is genuinely too close) and you
    want it visible every time it happens regardless of log throttling.
    """
    front_angles_degrees = range(-5, 6)  # -5 to 5 degrees inclusive

    # Set the left and right wall sensing angles based on maneuver direction.
    # NOTE: these ranges genuinely differ per direction (unlike the earlier
    # version where both branches used the same range and `clockwise` had
    # no real effect).
    if clockwise:
        if verbose:
            print("LiDAR: Clockwise mode. Right Range: [45, 85], Left Range: [-85, -45]")
        right_wall_angles_degrees = range(45, 85)
        left_wall_angles_degrees = range(-85, -45)
    else:
        if verbose:
            print("LiDAR: Anti-Clockwise mode. Right Range: [30, 91], Left Range: [-90, -29]")
        right_wall_angles_degrees = range(30, 91)
        left_wall_angles_degrees = range(-90, -29)

    # --- Immediate Obstacle Detection (Safety) ---
    front_distances = [scan_data[angle] for angle in front_angles_degrees if angle in scan_data and scan_data[angle] > 0]
    for dist in front_distances:
        if dist < safety_distance_mm:
            # Always printed (not gated by verbose) - rare, safety-relevant event.
            print(f"LiDAR: WARNING! Obstacle at {dist:.0f}mm. Commanding STOP.")
            return 9999.0  # Signal for immediate stop

    # --- Wall Following Logic ---
    right_distances = [d for a, d in scan_data.items() if a in right_wall_angles_degrees and d is not None and 0 < d < 3000]
    left_distances = [d for a, d in scan_data.items() if a in left_wall_angles_degrees and d is not None and 0 < d < 3000]

    right_distances.sort(reverse=True)
    left_distances.sort(reverse=True)
    num_values = 20  # Consider the 20 furthest points to avoid noise from close objects
    top_right_distances = right_distances[:num_values]
    top_left_distances = left_distances[:num_values]

    avg_right_distance = np.mean(top_right_distances) if top_right_distances else None
    avg_left_distance = np.mean(top_left_distances) if top_left_distances else None

    if verbose and (avg_right_distance is not None or avg_left_distance is not None):
        print(f"LiDAR: Avg Right: {avg_right_distance or 'N/A'}, Avg Left: {avg_left_distance or 'N/A'}")

    error = 0.0
    if avg_right_distance is not None and avg_left_distance is not None:
        error = avg_right_distance - avg_left_distance  # Balance between two walls
    elif avg_right_distance is not None:
        error = avg_right_distance - target_distance_mm  # Keep target distance from right wall
    elif avg_left_distance is not None:
        error = target_distance_mm - avg_left_distance  # Keep target distance from left wall (inverted error)
    else:
        error = 0.0  # No walls detected, continue straight

    return error



PARALLEL_CHECK_HALF_ANGLE_DEG = 20
PARALLEL_SECTOR_WIDTH_DEG = 15
PARALLEL_MAX_VALID_RANGE_MM = 6000.0
PARALLEL_TOLERANCE_MM = 25.0

# Defaults used when get_wall_parallel_error() is called with a target distance.
# DISTANCE_WEIGHT controls how much of the combined error comes from the
# "how far am I from the target distance" term vs. the "am I angled toward/
# away from the wall" (front-rear) term. 0.0 = pure alignment (old behavior),
# 1.0 = pure distance-hold, 0.5 = even split.
PARALLEL_DEFAULT_TARGET_DISTANCE_MM = 500.0
PARALLEL_DISTANCE_WEIGHT = 0.5


def _parallel_sector_ranges(side):
    half_width = PARALLEL_SECTOR_WIDTH_DEG // 2

    if side == "left":
        front_center = -90 + PARALLEL_CHECK_HALF_ANGLE_DEG
        rear_center = -90 - PARALLEL_CHECK_HALF_ANGLE_DEG
    elif side == "right":
        front_center = 90 - PARALLEL_CHECK_HALF_ANGLE_DEG
        rear_center = 90 + PARALLEL_CHECK_HALF_ANGLE_DEG
    else:
        raise ValueError("side must be 'left' or 'right'")

    front_angles = range(front_center - half_width, front_center + half_width)
    rear_angles = range(rear_center - half_width, rear_center + half_width)
    return front_angles, rear_angles


def _sector_average_and_count(scan_data, angles):
    values = [
        scan_data[angle]
        for angle in angles
        if angle in scan_data and 0 < scan_data[angle] <= PARALLEL_MAX_VALID_RANGE_MM
    ]
    if not values:
        return None, 0
    return sum(values) / len(values), len(values)


def get_wall_parallel_sector_stats(scan_data, side):
    front_angles, rear_angles = _parallel_sector_ranges(side)
    front_avg, front_count = _sector_average_and_count(scan_data, front_angles)
    rear_avg, rear_count = _sector_average_and_count(scan_data, rear_angles)
    return front_avg, rear_avg, front_count, rear_count


def get_wall_parallel_error(scan_data, side, target_distance_mm=None, distance_weight=PARALLEL_DISTANCE_WEIGHT):
    """
    Combined parallel-alignment + distance-hold error using the SAME two
    front/rear sectors (the "two sides of the triangle") that the parallel
    mode already samples -- no full-triangle/wide-angle scan required.

    - alignment_error = front_avg - rear_avg
        Positive => front sector reads farther than rear => robot's nose is
        angled away from the wall (for the "left" side convention used
        elsewhere in this file/the debug harness).
    - distance_error = avg(front_avg, rear_avg) - target_distance_mm
        Positive => robot is farther from the wall than the target distance.

    If target_distance_mm is None, this behaves exactly like the original
    parallel-only function (returns alignment_error only, no distance term).

    If target_distance_mm is given, returns a weighted blend of the two,
    using the same sign convention as alignment_error so callers don't need
    to change how they normalize/consume the result.
    """
    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, side)

    if front_count == 0 or rear_count == 0:
        return None

    alignment_error = front_avg - rear_avg

    if target_distance_mm is None:
        return alignment_error

    avg_distance = (front_avg + rear_avg) / 2.0
    distance_error = avg_distance - target_distance_mm

    distance_weight = max(0.0, min(1.0, distance_weight))
    return (distance_weight * distance_error) + ((1.0 - distance_weight) * alignment_error)