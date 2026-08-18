"""
Shared configuration for the multiprocess WRO robot control stack.

nav_process.py, lidar_process.py, and vision_process.py all import from
here so there is a single source of truth for every tunable — same role
this section played at the top of the old single-file script.
"""

# --- LIDAR / SERIAL PORTS ---
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400

PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- LIDAR CONTROL DESIGN PARAMETERS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 200
WALL_LOSS_THRESHOLD_MM = 350.0
CLOCKWISE_WALL_FOLLOWING = True  # True -> track/follow the LEFT wall
WALL_FOLLOW_TARGET_MM = LIDAR_TARGET_DISTANCE_MM
WALL_FOLLOW_SIDE = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

# --- CORNERING TUNABLES ---
CORNER_FRONT_TRIGGER_MM = 1000.0
CORNER_COOLDOWN_SEC = 5.0

# --- OUTER WALL-FOLLOW TUNABLES ---
WALL_FOLLOW_TARGET_DISTANCE_MM = 500.0
PID_OUTPUT_CLAMP = 40.0

# --- SMOOTH LIDAR SIDE-AVOIDANCE TUNABLES ---
SIDE_TRIGGER_DISTANCE_MM = 200
SIDE_STEER_MIN_MAGNITUDE = 15
SIDE_STEER_MAX_MAGNITUDE = 35

MIN_TRIGGER_POINTS = 3
TRIGGER_CONFIRM_FRAMES = 3
RELEASE_CONFIRM_FRAMES = 4

SIDE_RIGHT_ZONE_ANGLES = range(50, 90)
SIDE_LEFT_ZONE_ANGLES = range(-90, -50)

# --- FRONT SECTOR ---
FRONT_TURN_TRIGGER_MM = 200.0
FRONT_SCAN_ANGLE_DEG = 15

# --- ACTUATION CONSTANTS ---
SPEED = 255
SERVO_CENTER_ANGLE = 95
SERVO_ANGLE_LIMIT = 30
ROBOT_CRUISE_SPEED = SPEED
ROBOT_MANEUVER_SPEED = SPEED

# --- LOGGING ---
LOG_EVERY_N_LOOPS = 5
VERBOSE_LOGGING = True
PROFILING_ENABLED = True

# --- LOOP TIMING ---
NAV_LOOP_SLEEP_S = 0.005      # was time.sleep(0.02); nav is compute-only now
LIDAR_STALE_TIMEOUT_S = 0.5   # nav falls back to gyro-straight if lidar data is older than this
VISION_STALE_TIMEOUT_S = 0.5  # nav ignores vision (falls through to Tier 5) if vision data is older than this

# --- CAMERA CONFIGURATION ---
CAMERA_RESOLUTION = (2304, 1296)
LORES_RESOLUTION = (960, 540)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = LORES_RESOLUTION[0]
PROCESSING_HEIGHT = LORES_RESOLUTION[1]

# --- VISION / OBSTACLE-AVOIDANCE TUNABLES ---
# Applied in vision_process.py to turn the raw steering_angle from
# process_frame_for_steering() into the final servo-adjust it publishes.
# (Ported as-is from the old single-file main loop.)
STEERING_GAIN_GREEN = 0.06
STEERING_GAIN_RED = 0.14
RED_CLEARANCE_OFFSET = 0

# --- VISION DEBUG / STREAMING ---
STREAM_VIDEO = True            # master on/off for the Flask MJPEG debug stream
DEBUG_UI_OVERLAYS = True       # draw ROI/contour/text overlays on the streamed frame
STREAM_FRAME_INTERVAL_S = 1.0 / 15.0
VISION_FLASK_PORT = 5000


class RobotState:
    INITIALIZING = "INITIALIZING"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    VISION_OBSTACLE_AVOIDANCE = "VISION_OBSTACLE_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    CORNER_MANEUVER = "CORNER_MANEUVER"
    LAP_TERMINATION = "LAP_TERMINATION"
    STOP = "STOP"
