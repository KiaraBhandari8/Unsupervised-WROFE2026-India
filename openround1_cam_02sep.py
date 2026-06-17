import time
import cv2
import numpy as np
from picamera2 import Picamera2
from lidar_steering2 import LidarScanner, PIDController, calculate_steering_error
import robot_motion

ROBOT_SPEED = 0.8
SERVO_CENTER_ANGLE = 97 # Using a more descriptive name for center_angle

# LiDAR side-check override parameters
LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 50
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 250
LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -50
LIDAR_LEFT_SIDE_DISTANCE_MM = 250
LIDAR_SIDE_STEER_MAGNITUDE = 15

# Servo angle limits
LIDAR_SERVO_MIN_ANGLE = SERVO_CENTER_ANGLE - 25
LIDAR_SERVO_MAX_ANGLE = SERVO_CENTER_ANGLE + 25

print("Waiting 5 seconds for hardware to initialize...")
time.sleep(5)

class RobotState:
    INITIALIZING = "INITIALIZING"
    STOP = "STOP"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"

def check_lidar_side_alerts(scan_data):
    """Checks for immediate obstacles on the left or right sides."""
    if not scan_data: return None
    # Check right side first
    for angle, distance in scan_data.items():
        if LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_RIGHT_SIDE_DISTANCE_MM:
            return "RIGHT"
    # Check left side
    for angle, distance in scan_data.items():
        if LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_LEFT_SIDE_DISTANCE_MM:
            return "LEFT"
    return None

def check_front_clearance(scan_data, min_angle, max_angle, target_distance_min=1300, target_distance_max=1700):
    """
    Calculates the average distance of LiDAR scans within a specific angle range.
    Returns True if the average distance is greater than the target distance.
    """
    if not scan_data:
        return False

    front_left_distances = []
    for angle, distance in scan_data.items():
        if angle >= min_angle and angle <=0 and distance > 0:
            front_left_distances.append(distance)

    front_right_distances = []
    for angle, distance in scan_data.items():
        if angle <= max_angle and angle>=0 and distance > 0:
            front_right_distances.append(distance)

    # If no valid readings in the range, we can't confirm clearance
    if not front_left_distances or not front_right_distances:
        return False

    avg_left = np.mean(front_left_distances)
    avg_right = np.mean(front_right_distances)

    print(f"Avg Left: {avg_left:.2f} mm, Avg Right: {avg_right:.2f} mm")

    if avg_left > target_distance_min and avg_left < target_distance_max:
        if avg_right > target_distance_min and avg_right < target_distance_max:
            return True

    return False 

# --- Color Filtering Functions ---
def filter_blue_objects(hsv_frame):
    """Detects presence of blue using HSV masking."""
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)
    return blue_mask

def filter_orange_objects(hsv_frame):
    """Detects presence of orange using HSV masking."""
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    orange_mask = cv2.erode(orange_mask, kernel, iterations=2)
    orange_mask = cv2.dilate(orange_mask, kernel, iterations=2)
    return orange_mask

def detect_color_binary(mask, threshold=4000):
    """Returns True if color is present above a pixel threshold."""
    return cv2.countNonZero(mask) > threshold

def map_steering_angle(center_angle, pid_output, clockwise=True):
    scale_factor = 0.2
    adjusted_output = pid_output * scale_factor
    adjusted_output = -1 * adjusted_output
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    return int(max(center_angle-20, min(angle, center_angle+20)))

def main():
    print("=== Robot PID Navigation & Color Line Detection ===")
    clockwise_mode = True
    center_angle = 95
    target_distance_mm = 750
    safety_distance_mm = 300
    max_color_count = 12
    
    # LiDAR stopping condition parameters
    LIDAR_STOP_MIN_ANGLE = -5
    LIDAR_STOP_MAX_ANGLE = 5
    LIDAR_STOP_DISTANCE_MM = 1600

    # State flag for post-line crossing
    crossed_12 = False

    # PID selection
    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) if clockwise_mode else PIDController(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)

    # Camera setup
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (768, 432)})
    picam2.configure(config)
    picam2.start()

    # Line crossing counters, states, and cooldown timers
    blue_count = 0
    orange_count = 0
    prev_blue_state = False
    prev_orange_state = False
    blue_cooldown_end_time = 0
    orange_cooldown_end_time = 0

    try:
        with LidarScanner() as scanner:
            print("LiDAR and camera active, starting navigation.")
            while True:
                current_time = time.time()

                # Get sensor data
                scan_data = scanner.get_scan_data()
                frame = picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

                # Color Detection
                blue_mask = filter_blue_objects(hsv)
                orange_mask = filter_orange_objects(hsv)
                blue_in_view = detect_color_binary(blue_mask)
                orange_in_view = detect_color_binary(orange_mask)

                # Blue Line Binary Logic with Cooldown
                if not blue_in_view and prev_blue_state:
                    if current_time > blue_cooldown_end_time:
                        blue_count += 1
                        print(f"Blue line crossed! Total blue lines: {blue_count}")
                        blue_cooldown_end_time = current_time + 3
                prev_blue_state = blue_in_view

                # Orange Line Binary Logic with Cooldown
                if not orange_in_view and prev_orange_state:
                    if current_time > orange_cooldown_end_time:
                        orange_count += 1
                        print(f"Orange line crossed! Total orange lines: {orange_count}")
                        orange_cooldown_end_time = current_time + 3
                prev_orange_state = orange_in_view

                print(f"Blue Count: {blue_count}, Orange Count: {orange_count}")

                # --- STOPPING LOGIC ---
                # Step 1: Check if the line count has been reached and set the flag.
                if not crossed_12 and (blue_count >= max_color_count and orange_count >= max_color_count):
                    print("Line crossing target reached. Now looking for a clear path to stop.")
                    crossed_12 = True
                    # Optional: Reset counters to prevent this block from re-triggering
                    blue_count = 0
                    orange_count = 0

                # Step 2: If the flag is set, check the LiDAR condition in every loop.
                if crossed_12:
                    if check_front_clearance(scan_data, LIDAR_STOP_MIN_ANGLE, LIDAR_STOP_MAX_ANGLE, target_distance_min=1100, target_distance_max=1500):
                        if final_servo_angle >= SERVO_CENTER_ANGLE - 5 and final_servo_angle <= SERVO_CENTER_ANGLE + 5:
                            print("Reached the center of the section. Stopping robot.")
                            robot_motion.robot_stop()
                            time.sleep(120)
                            break # Exit the while loop

                # --- NAVIGATION LOGIC ---
                side_alert_status = check_lidar_side_alerts(scan_data)
                
                if side_alert_status == "RIGHT":
                    target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                
                elif side_alert_status == "LEFT":
                    target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE

                else: # LIDAR_WALL_FOLLOWING
                    error = calculate_steering_error(scan_data, target_distance_mm=target_distance_mm, safety_distance_mm=safety_distance_mm)
                    
                    if error == 9999.0:
                        print("Obstacle detected ahead! Stopping for safety.")
                        robot_motion.robot_stop()
                        time.sleep(1)
                        continue
                        
                    pid_output = pid.update(error)
                    target_servo_angle = map_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=clockwise_mode)

                # Finalize and send commands to robot
                final_servo_angle = int(round(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE)))
                
                robot_motion.robot_forward_speed(ROBOT_SPEED)
                robot_motion.adjust_servo_angle(final_servo_angle)
                
                time.sleep(0.15)

    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        print("Shutting down...")
        picam2.close()
        robot_motion.robot_stop()
        robot_motion.motor_standby()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()