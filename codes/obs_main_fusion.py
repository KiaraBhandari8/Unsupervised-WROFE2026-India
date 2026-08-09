import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response, request, jsonify
import threading
import time
import os
import json
import signal

try:
    import serial
except ImportError:
    serial = None

# Import custom functions
try:
    from lidar_steering4sept import LidarScanner, PIDController, calculate_steering_error
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit()

# --- Global Variables ---
output_frame = None
output_frame_lock = threading.Lock()

# Shared LiDAR data buffer and its lock
latest_lidar_data = {}

lidar_data_lock = threading.Lock()

# Shared gyro yaw (degrees) streamed from the ESP32 as "YAW:<float>" lines.
# gyro_ok flips True only once a YAW line is actually received, so the
# recentering feature stays a no-op on firmware that doesn't emit yaw.
latest_yaw = 0.0
yaw_lock = threading.Lock()
gyro_ok = False

# Shared buffer for the latest camera frame and its lock
latest_camera_frame = None
latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

# --- POWER / CPU BUDGET (3A-friendly profile) ---
CONTROL_LOOP_MAX_HZ = 30.0      # max control loop iterations per second
STARTUP_STAGE_DELAY_SEC = 2.0   # pause between subsystem bring-ups so the
                                # camera and LiDAR spin-up (USB inrush) never
                                # stack on one current peak


app = Flask(__name__)
# Pick up template (obs_lab.html) edits without needing a full server restart.
app.config["TEMPLATES_AUTO_RELOAD"] = True
app.jinja_env.auto_reload = True

# Shutdown event for graceful Ctrl+C
shutdown_event = threading.Event()

# Turn Counter Variables
turn_counter = 0
max_turn_count = 12
previous_increment_time = time.time()
START_PAUSE_DURATION = 5
DELAY_BETWEEN_TURNS = 7
OUT_PARKING_MANEUVER = False
straight_detected_time = 0.0
straight_override_duration = 1.5

# --- CONTROL CONSTANTS ---
SERVO_CENTER_ANGLE = 102
STEERING_GAIN = 0.1
ROBOT_MANEUVER_SPEED = 0.8
ROBOT_CRUISE_SPEED = 0.85
ROBOT_SPEED_MAX = 0.9


# --- CAMERA CONFIGURATION ---
# main stays at the full-FOV binned sensor mode (wide angle preserved).
# lores is a hardware-scaled copy of the SAME full-FOV image, produced by the
# ISP at zero CPU cost. All processing reads lores; the full-res main stream
# is never copied into Python.
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2   # 1152 (lores stream size)
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2  # 648
# Detection works on a smaller LAB frame to keep the morphology cheap.
LAB_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   # 768
LAB_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  # 432


# ===================== LAB COLOUR DETECTION =====================
# LAB colour ranges loaded from vision_presets.json (tuned in hsv_webpage.py).
# Each preset is {l_min,l_max,a_min,a_max,b_min,b_max}.
PRESETS_FILE = "/home/pi8/wrofe2025/vision_presets.json"

# Fallback defaults if the presets file is missing.
DEFAULT_PRESETS = {
    "red":    {"l_min": 0,   "l_max": 255, "a_min": 146, "a_max": 255, "b_min": 100, "b_max": 255},
    "blue":   {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 140, "b_min": 0,   "b_max": 120},
    "green":  {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 120, "b_min": 80,  "b_max": 200},
    "orange": {"l_min": 0,   "l_max": 255, "a_min": 140, "a_max": 255, "b_min": 140, "b_max": 255},
    "white":  {"l_min": 100, "l_max": 255, "a_min": 0,   "a_max": 255, "b_min": 0,   "b_max": 255},
}

# Colours shown as individually-segmented streams on the web page.
STREAM_COLORS = ["red", "green", "orange", "blue", "white"]


def load_presets():
    """Load LAB presets from disk, falling back to built-in defaults."""
    presets = {k: dict(v) for k, v in DEFAULT_PRESETS.items()}
    try:
        with open(PRESETS_FILE) as f:
            loaded = json.load(f)
        for color in STREAM_COLORS:
            if color in loaded:
                presets[color] = loaded[color]
        print(f"[LAB] Loaded presets from {PRESETS_FILE}")
    except Exception as e:
        print(f"[LAB] Could not load {PRESETS_FILE} ({e}); using defaults.")
    return presets


COLOR_PRESETS = load_presets()
presets_lock = threading.Lock()

# ===================== HSV PRESETS (web tuning only) =====================
HSV_PRESETS_FILE = "/home/pi8/wrofe2025/vision_hsv_presets.json"

DEFAULT_HSV_PRESETS = {
    "red":    {"h_min": 0,   "h_max": 10,  "s_min": 160,  "s_max": 255, "v_min": 60,  "v_max": 255},
    "blue":   {"h_min": 90, "h_max": 149, "s_min": 35,  "s_max": 255, "v_min": 50,  "v_max": 255},
    "green":  {"h_min": 55,  "h_max": 106,  "s_min": 20,  "s_max": 208, "v_min": 30,  "v_max": 197},
    "orange": {"h_min": 5,   "h_max": 25,  "s_min": 197, "s_max": 255, "v_min": 100, "v_max": 255},
    "white":  {"h_min": 0,   "h_max": 15, "s_min": 0,   "s_max": 60,  "v_min": 100, "v_max": 255},
}


def load_hsv_presets():
    presets = {k: dict(v) for k, v in DEFAULT_HSV_PRESETS.items()}
    try:
        with open(HSV_PRESETS_FILE) as f:
            loaded = json.load(f)
        for color in STREAM_COLORS:
            if color in loaded:
                presets[color] = loaded[color]
        print(f"[HSV] Loaded presets from {HSV_PRESETS_FILE}")
    except Exception as e:
        print(f"[HSV] Could not load {HSV_PRESETS_FILE} ({e}); using defaults.")
    return presets


HSV_PRESETS = load_hsv_presets()
hsv_presets_lock = threading.Lock()

# Detection / steering tuning for the pillar colours.
LAB_MIN_CONTOUR_AREA = 900       # ignore tiny colour blobs (raised to drop far/off-track specks)
LAB_MIN_WIDTH = 30               # ignore thin blobs
# Pillars on the track project into the LOWER part of the wide-angle frame.
# Anything whose base sits above this fraction of the frame height is horizon /
# background (spectators, banners, far wall) -> ignore it.
LAB_ROI_TOP_FRAC = 0.35          # ignore blobs whose base is above 35% of frame height
COLOR_STEER_MAGNITUDE = 22       # base steering away from a coloured pillar

# --- OBSTACLE AVOIDANCE MANEUVER ---
# The robot starts turning BEFORE it reaches the pillar (at least
# AVOID_CLEARANCE_MM early), holds the turn until it has drawn alongside the
# pillar (the pillar slides to the edge of the frame), then drives straight to
# realign. GREEN -> pass on the LEFT, RED -> pass on the RIGHT.
AVOID_STANDOFF_MM = 250          # desired clearance to the pillar while turning
AVOID_CLEARANCE_MM = 300         # begin turning at least this far early (25 cm) -
                                 # raised so the robot eases onto the trajectory
                                 # line and turns well before it reaches the pillar
AVOID_PASS_CX_FRAC = 0.78        # pillar past this frac toward the edge = passed
REALIGN_DURATION_SEC = 1       # drive straight this long after passing
AVOID_WIDTH_TRIGGER = 30         # LAB-frame px width to start avoiding. Just above
                                 # LAB_MIN_WIDTH (30) so the turn engages almost as
                                 # soon as a pillar is reliably detected (earliest
                                 # possible camera trigger) while a far speck still
                                 # won't fire it
AVOID_DIST_GAIN_REF_MM = 350.0   # reference distance for turn-strength scaling

# ===================== FUSION: LiDAR OBSTACLE + BOUNDARY DETECTION =====================
# LiDAR scans the front sector to find isolated obstacles (pillars) and
# continuous walls (arena boundaries).  The camera then only needs to answer
# WHAT colour the obstacle is, not WHERE it is.
LIDAR_OBSTACLE_FRONT_DEG = 30        # half-width of the front scan sector (deg)
LIDAR_OBSTACLE_GAP_MM = 150          # an obstacle point must be this much closer
                                     # than its local wall median to be flagged
LIDAR_OBSTACLE_MIN_CLUSTER_DEG = 2   # minimum angular width (deg) to count
LIDAR_OBSTACLE_MAX_MM = 2500         # ignore points beyond this range
LIDAR_OBSTACLE_MIN_MM = 80           # ignore points closer than this (noise)

# Arena boundary detection: a continuous wall filling the front FOV.
BOUNDARY_FRONT_DEG = 30              # half-width of the front sector for boundary
BOUNDARY_DISTANCE_MM = 800           # front distances below this count as "wall"
BOUNDARY_FILL_THRESHOLD = 0.70       # 70% of front angles filled = boundary ahead
BOUNDARY_SIDE_STEER_MM = 500         # steer toward the side with more open space

# --- DRIVABLE-AREA GATING ---
# A pillar that is actually ON the track sits on the white floor, so there is
# white directly below its base. Coloured things off the track (spectators,
# walls, banners) have no white floor beneath them -> ignore them so the robot
# does not steer for an obstacle that isn't in its path.
#
# Rather than trusting any stray white pixels below a blob, we first build a
# single "track mask": the ONE connected white region the robot is driving on
# (the largest white component that reaches the bottom of the frame). White
# things that aren't part of that contiguous floor (banners, far walls, the
# ceiling, spectators' shirts) form separate components and are excluded. A
# pillar only counts if its base sits on/inside that track region.
WHITE_FLOOR_BAND_PX = 24         # height of the band sampled below a pillar (LAB-frame px)
WHITE_FLOOR_MIN_RATIO = 0.30     # min fraction of that band that must be on the track
TRACK_MASK_CLOSE_PX = 25         # close gaps (pillars/lines) so the floor is one blob
TRACK_MIN_AREA_FRAC = 0.02       # ignore white components smaller than this frac of frame
TRACK_BASE_DILATE_PX = 12        # tolerance band around the track edge for a pillar base


def build_track_mask(white_mask):
    """Return a binary mask of the single drivable white region.

    The track is the contiguous white floor the robot drives on. We close
    small gaps (the coloured pillars and floor lines punch holes in the white),
    then keep only the largest connected white component that reaches the
    bottom edge of the frame. Every other patch of white in the scene -- walls,
    banners, ceiling, spectators -- is a separate component and is discarded, so
    colour blobs sitting on those patches are NOT treated as on-track."""
    h, w = white_mask.shape[:2]
    # Close gaps so pillars/lines on the floor don't split the track in two.
    k = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (TRACK_MASK_CLOSE_PX, TRACK_MASK_CLOSE_PX))
    closed = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, k, iterations=1)

    num, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    if num <= 1:
        return np.zeros_like(white_mask)

    min_area = TRACK_MIN_AREA_FRAC * (h * w)
    best_label, best_area = 0, 0
    for lbl in range(1, num):
        area = stats[lbl, cv2.CC_STAT_AREA]
        if area < min_area:
            continue
        # Must touch the bottom of the frame -- the floor under the robot does;
        # a high-up banner/wall patch does not.
        top = stats[lbl, cv2.CC_STAT_TOP]
        height = stats[lbl, cv2.CC_STAT_HEIGHT]
        reaches_bottom = (top + height) >= (h - 2)
        if reaches_bottom and area > best_area:
            best_label, best_area = lbl, area

    if best_label == 0:
        return np.zeros_like(white_mask)

    track = np.where(labels == best_label, 255, 0).astype(np.uint8)
    # Small dilation: a pillar's base sits just ABOVE the floor it stands on, so
    # allow a little tolerance around the track edge.
    if TRACK_BASE_DILATE_PX > 0:
        dk = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (TRACK_BASE_DILATE_PX, TRACK_BASE_DILATE_PX))
        track = cv2.dilate(track, dk, iterations=1)
    return track


def lab_mask(lab_frame, color):
    """Build the LAB inRange mask for a colour preset, with a small open."""
    with presets_lock:
        p = dict(COLOR_PRESETS[color])
    mask = cv2.inRange(
        lab_frame,
        np.array([p["l_min"], p["a_min"], p["b_min"]]),
        np.array([p["l_max"], p["a_max"], p["b_max"]]),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)
    return mask


def _on_drivable_area(track_mask, bbox):
    """True if the pillar is standing ON the drivable (white) track region.

    `track_mask` is the single connected floor region from build_track_mask().
    A pillar is coloured (not white) so the pillar body itself is NOT inside
    the track mask -- but the floor DIRECTLY BELOW its base is. We sample a
    band just below the base and require it to be mostly track. Coloured
    objects off the track (spectators, walls, banners) have no contiguous floor
    region beneath them -- any white pixels there are separate components that
    build_track_mask() already discarded -- so the band reads empty and they
    are dropped."""
    x1, y1, x2, y2 = bbox
    h, w = track_mask.shape[:2]
    # If the track mask came back empty (no floor found) keep nothing.
    if cv2.countNonZero(track_mask) == 0:
        return False

    by1 = min(y2, h)
    by2 = min(y2 + WHITE_FLOOR_BAND_PX, h)
    bx1 = max(0, x1)
    bx2 = min(x2, w)
    # Pillar is right at the bottom edge -> it's directly in front of us; the
    # floor below is out of frame. Require its base column to be inside the
    # track region instead so an off-frame-bottom object can't sneak through.
    if (by2 - by1) < 2 or (bx2 - bx1) < 2:
        cx = int(np.clip((x1 + x2) // 2, 0, w - 1))
        cy = int(np.clip(y2 - 1, 0, h - 1))
        return track_mask[cy, cx] != 0
    band = track_mask[by1:by2, bx1:bx2]
    ratio = cv2.countNonZero(band) / band.size
    return ratio >= WHITE_FLOOR_MIN_RATIO


def detect_lab_pillars(lab_frame):
    """Detect red/green pillars in a LAB frame. Returns list of detections.

    Only pillars sitting on the white drivable area are returned; coloured
    objects off the track are dropped so they don't trigger steering."""
    white_mask = lab_mask(lab_frame, 'white')
    track_mask = build_track_mask(white_mask)
    frame_h = lab_frame.shape[0]
    roi_top = int(frame_h * LAB_ROI_TOP_FRAC)
    dets = []
    for cls in ("red", "green"):
        mask = lab_mask(lab_frame, cls)
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in conts:
            ar = cv2.contourArea(c)
            if ar > LAB_MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                if w > LAB_MIN_WIDTH:
                    bbox = [x, y, x + w, y + h]
                    # Base of the blob must be in the lower (track) region of the
                    # frame; horizon / background colour sits up high -> drop it.
                    if (y + h) < roi_top:
                        continue
                    if not _on_drivable_area(track_mask, bbox):
                        continue  # off-track colour -> ignore, don't steer
                    dets.append({
                        'class': cls,
                        'area': float(ar),
                        'bbox': bbox,
                        'width': w,
                        'cx': x + w // 2,
                    })
    return dets


# --- Obstacle-avoidance maneuver state (module level, persists across loops) ---
avoid_state = "IDLE"        # IDLE -> TURNING -> REALIGN -> RECENTER -> IDLE
avoid_color = None          # 'green' or 'red' currently being avoided
realign_end_time = 0.0
pre_avoid_yaw = None        # heading snapshot taken just before the turn starts
recenter_end_time = 0.0     # deadline for the gyro recentering phase


def _gyro_recenter_angle(current_yaw, target_yaw):
    """Servo angle that steers the robot back toward its pre-avoidance heading.

    error > 0 means we still need to rotate back toward target_yaw. The sign
    constant maps that to the correct steering direction for the IMU in use."""
    err = target_yaw - current_yaw
    offset = GYRO_RECENTER_SIGN * GYRO_RECENTER_KP * err
    offset = max(-GYRO_RECENTER_MAX_OFFSET, min(GYRO_RECENTER_MAX_OFFSET, offset))
    return SERVO_CENTER_ANGLE + offset


def _front_distance_mm(scan_data):
    """Median LiDAR distance straight ahead (-5..+5 deg), or None."""
    if not scan_data:
        return None
    fronts = [d for a, d in scan_data.items() if -5 <= a <= 5 and d > 0]
    return float(np.median(fronts)) if fronts else None


def _avoid_servo_angle(color, front_dist, cx=None, lab_w=None):
    """Servo angle for the avoidance turn. Turn harder the closer we are.
    GREEN -> steer LEFT (below centre), RED -> steer RIGHT (above centre).
    Sharpens the turn when the pillar is far to the relevant side:
      green pillar on the left edge -> sharper left
      red pillar on the right edge  -> sharper right"""
    mag = COLOR_STEER_MAGNITUDE
    if front_dist and front_dist > 0:
        mag = COLOR_STEER_MAGNITUDE * max(0.6, min(1.8, AVOID_DIST_GAIN_REF_MM / front_dist))
    if cx is not None and lab_w is not None and lab_w > 0:
        if color == 'green':
            side_factor = max(0.0, 1.0 - 2.0 * cx / lab_w)
            mag *= (1.0 + side_factor)
        else:
            side_factor = max(0.0, 2.0 * cx / lab_w - 1.0)
            mag *= (1.0 + side_factor)
    return SERVO_CENTER_ANGLE - mag if color == 'green' else SERVO_CENTER_ANGLE + mag


# ===================== LiDAR OBSTACLE + BOUNDARY DETECTION =====================

def detect_lidar_obstacles(scan_data):
    """Find isolated obstacles (pillars) in the front sector of a LiDAR scan.

    For each angle in [-LIDAR_OBSTACLE_FRONT_DEG, +LIDAR_OBSTACLE_FRONT_DEG]
    the measured distance is compared to the *local wall median* — the median
    distance of non-obstacle points in the ±15° neighbourhood.  If the point
    is significantly closer, it belongs to an obstacle rather than the wall.

    Returns a list of dicts sorted by distance (nearest first):
        {'angle_deg': float, 'distance_mm': float, 'width_deg': float}
    """
    if not scan_data:
        return []

    front = LIDAR_OBSTACLE_FRONT_DEG
    front_angles = sorted(a for a in scan_data
                          if -front <= a <= front
                          and scan_data[a] > LIDAR_OBSTACLE_MIN_MM
                          and scan_data[a] < LIDAR_OBSTACLE_MAX_MM)
    if not front_angles:
        return []

    # --- Step 1: identify wall vs obstacle points in the front sector ---
    obstacle_flags = {}  # angle -> True/False
    for a in front_angles:
        dist = scan_data[a]
        # Gather non-front angles (±15° around this angle but outside [-5,+5])
        # that represent the "wall" at roughly the same direction.
        wall_candidates = []
        for a2, d2 in scan_data.items():
            if a2 == a:
                continue
            if abs(a2 - a) <= 15 and abs(a2) > 5:
                if d2 > LIDAR_OBSTACLE_MIN_MM and d2 < LIDAR_OBSTACLE_MAX_MM:
                    wall_candidates.append(d2)
        # Also include front angles that are clearly further away (wall, not pillar)
        for a2 in front_angles:
            if a2 == a:
                continue
            if abs(a2 - a) <= 15 and scan_data[a2] > dist + LIDAR_OBSTACLE_GAP_MM:
                wall_candidates.append(scan_data[a2])

        if wall_candidates:
            wall_median = float(np.median(wall_candidates))
            obstacle_flags[a] = (wall_median - dist) > LIDAR_OBSTACLE_GAP_MM
        else:
            # No wall context — treat as obstacle if within a reasonable range
            obstacle_flags[a] = LIDAR_OBSTACLE_MIN_MM < dist < LIDAR_OBSTACLE_MAX_MM

    # --- Step 2: cluster consecutive obstacle angles ---
    obstacle_angles = [a for a in front_angles if obstacle_flags.get(a, False)]
    if not obstacle_angles:
        return []

    clusters = []
    cluster_start = obstacle_angles[0]
    cluster_prev = obstacle_angles[0]
    for a in obstacle_angles[1:]:
        if a - cluster_prev <= 3:  # consecutive (allowing small gaps)
            cluster_prev = a
        else:
            clusters.append((cluster_start, cluster_prev))
            cluster_start = a
            cluster_prev = a
    clusters.append((cluster_start, cluster_prev))

    # --- Step 3: build obstacle objects from clusters ---
    obstacles = []
    for a_start, a_end in clusters:
        width = a_end - a_start
        if width < LIDAR_OBSTACLE_MIN_CLUSTER_DEG:
            continue
        center_a = (a_start + a_end) / 2.0
        # Closest distance in the cluster
        cluster_dists = [scan_data[a] for a in range(int(a_start), int(a_end) + 1)
                         if a in scan_data]
        if not cluster_dists:
            continue
        closest = min(cluster_dists)
        obstacles.append({
            'angle_deg': center_a,
            'distance_mm': closest,
            'width_deg': float(width),
        })

    obstacles.sort(key=lambda o: o['distance_mm'])
    return obstacles


def detect_lidar_boundary(scan_data):
    """Detect when a continuous wall fills the front FOV (arena boundary).

    Returns (is_boundary, avg_front_distance, open_side) where:
      is_boundary        – True if >= BOUNDARY_FILL_THRESHOLD of front angles
                           have distances below BOUNDARY_DISTANCE_MM
      avg_front_distance – average distance of the boundary wall ahead
      open_side          – "LEFT" / "RIGHT" / None indicating which side has
                           more open space (for wall-follow direction)
    """
    if not scan_data:
        return False, None, None

    front = BOUNDARY_FRONT_DEG
    front_angles = [a for a in scan_data
                    if -front <= a <= front
                    and scan_data[a] > LIDAR_OBSTACLE_MIN_MM]
    if not front_angles:
        return False, None, None

    near_count = sum(1 for a in front_angles if scan_data[a] < BOUNDARY_DISTANCE_MM)
    fill_ratio = near_count / len(front_angles)

    if fill_ratio < BOUNDARY_FILL_THRESHOLD:
        return False, None, None

    # Average front distance for the wall we are about to follow.
    near_dists = [scan_data[a] for a in front_angles
                  if scan_data[a] < BOUNDARY_DISTANCE_MM]
    avg_front = float(np.mean(near_dists)) if near_dists else None

    # Determine which side has more open space.
    left_dists = [d for a, d in scan_data.items()
                  if -90 <= a <= -40 and d > LIDAR_OBSTACLE_MIN_MM]
    right_dists = [d for a, d in scan_data.items()
                   if 40 <= a <= 90 and d > LIDAR_OBSTACLE_MIN_MM]
    avg_left = float(np.mean(left_dists)) if left_dists else 0
    avg_right = float(np.mean(right_dists)) if right_dists else 0

    open_side = "LEFT" if avg_left > avg_right else "RIGHT"
    return True, avg_front, open_side


def manage_fusion_avoidance(lidar_obstacles, camera_dets, scan_data, lab_w, current_yaw=None):
    """Stateful pillar avoidance driven by LiDAR position + camera colour.

    The LiDAR tells us WHERE the obstacle is (angle, distance).  The camera
    tells us WHAT colour it is (red / green) by matching the LiDAR angle to
    the horizontal position of a camera detection.

    Returns (engaged, servo_angle, label).

    GREEN -> pass on the LEFT, RED -> pass on the RIGHT.
    """
    global avoid_state, avoid_color, realign_end_time
    global pre_avoid_yaw, recenter_end_time
    now = time.time()
    have_gyro = GYRO_ENABLED and current_yaw is not None

    front_dist = _front_distance_mm(scan_data)

    # --- Find the nearest obstacle and its camera colour ---
    target_lidar = lidar_obstacles[0] if lidar_obstacles else None

    # Match camera colour to the LiDAR obstacle angle.
    matched_color = None
    matched_cx = None
    if target_lidar is not None and camera_dets and lab_w > 0:
        # Project the LiDAR obstacle angle to an approximate horizontal pixel.
        # In a wide-angle frame the angle roughly maps to a fraction of the width.
        # angle=0 -> centre, positive angle -> right side of frame.
        lidar_cx_frac = 0.5 + (target_lidar['angle_deg'] / (2.0 * 90.0))
        lidar_cx_frac = max(0.0, min(1.0, lidar_cx_frac))
        target_pixel_x = int(lidar_cx_frac * lab_w)

        # Find the camera detection closest to that pixel x.
        best_det = None
        best_pixel_dist = lab_w  # max possible
        for d in camera_dets:
            if d['class'] not in ('red', 'green'):
                continue
            pixel_dist = abs(d['cx'] - target_pixel_x)
            if pixel_dist < best_pixel_dist:
                best_pixel_dist = pixel_dist
                best_det = d
        # Accept the match if it is within a generous window (half the frame).
        if best_det is not None and best_pixel_dist < lab_w * 0.45:
            matched_color = best_det['class']
            matched_cx = best_det['cx']

    # --- REALIGN: hold straight for a moment after passing the pillar ---
    if avoid_state == "REALIGN":
        if now < realign_end_time:
            return True, SERVO_CENTER_ANGLE, f"realign_{avoid_color}"
        if have_gyro and pre_avoid_yaw is not None:
            avoid_state = "RECENTER"
            recenter_end_time = now + GYRO_RECENTER_DURATION_SEC
            return (True, _gyro_recenter_angle(current_yaw, pre_avoid_yaw),
                    f"recenter_{avoid_color}")
        avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None

    # --- RECENTER: gyro-steer back to the pre-avoidance heading ---
    if avoid_state == "RECENTER":
        if not have_gyro or pre_avoid_yaw is None:
            avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None
        else:
            heading_err = abs(pre_avoid_yaw - current_yaw)
            if now >= recenter_end_time or heading_err <= GYRO_RECENTER_TOL_DEG:
                avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None
            else:
                return (True, _gyro_recenter_angle(current_yaw, pre_avoid_yaw),
                        f"recenter_{avoid_color}")

    # --- TURNING: keep steering away until the pillar is passed ---
    if avoid_state == "TURNING":
        color = avoid_color
        passed = False
        # "Passed" when LiDAR no longer sees the obstacle ahead OR the
        # obstacle angle has moved past the side (robot drove alongside).
        if target_lidar is None:
            passed = True
        else:
            if color == 'green' and target_lidar['angle_deg'] > LIDAR_OBSTACLE_FRONT_DEG:
                passed = True
            elif color == 'red' and target_lidar['angle_deg'] < -LIDAR_OBSTACLE_FRONT_DEG:
                passed = True
            # Also pass if the camera shows the blob has moved to the edge.
            if matched_cx is not None:
                if color == 'green' and matched_cx > lab_w * AVOID_PASS_CX_FRAC:
                    passed = True
                elif color == 'red' and matched_cx < lab_w * (1.0 - AVOID_PASS_CX_FRAC):
                    passed = True
        if passed:
            avoid_state = "REALIGN"
            realign_end_time = now + REALIGN_DURATION_SEC
            return True, SERVO_CENTER_ANGLE, f"realign_{color}"
        return True, _avoid_servo_angle(color, front_dist, matched_cx, lab_w), f"avoid_{color}"

    # --- IDLE: decide whether to start a maneuver ---
    if target_lidar is not None:
        # Engage when the LiDAR obstacle is within the avoidance zone AND
        # we have (or can guess) its colour.
        dist_trigger = target_lidar['distance_mm'] <= (AVOID_STANDOFF_MM + AVOID_CLEARANCE_MM)
        if dist_trigger:
            avoid_color = matched_color or 'unknown'
            avoid_state = "TURNING"
            pre_avoid_yaw = current_yaw if have_gyro else None
            return True, _avoid_servo_angle(avoid_color, front_dist, matched_cx, lab_w), f"avoid_{avoid_color}"

    return False, None, "none"


def manage_color_avoidance(dets, scan_data, lab_w, current_yaw=None):
    """Stateful pillar avoidance. Returns (engaged, servo_angle, label).

    Starts the turn at least AVOID_CLEARANCE_MM early, holds it until the
    robot is alongside the pillar, drives straight to realign, then -- if a
    gyro heading is available -- recenters onto the pre-avoidance heading so
    the robot lines back up with the lane instead of drifting into a wall.

    current_yaw: latest gyro yaw in degrees, or None when no gyro is present
    (in which case the RECENTER phase is skipped and behaviour is unchanged).
    """
    global avoid_state, avoid_color, realign_end_time
    global pre_avoid_yaw, recenter_end_time
    now = time.time()
    have_gyro = GYRO_ENABLED and current_yaw is not None

    objs = [d for d in dets if d['class'] in ('red', 'green')]
    # NEAREST pillar = the one whose base sits lowest in the frame (largest
    # bbox bottom y2). A closer pillar always projects lower in the image, so
    # this is a far more reliable "nearest" signal than raw blob area (which
    # also varies with colour/lighting). Area is only a tie-break.
    target = max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None
    front_dist = _front_distance_mm(scan_data)

    # --- REALIGN: hold straight for a moment after passing the pillar ---
    if avoid_state == "REALIGN":
        if now < realign_end_time:
            return True, SERVO_CENTER_ANGLE, f"realign_{avoid_color}"
        # Realign done. If we have a heading reference, straighten back onto the
        # lane heading with the gyro; otherwise just hand back to wall following.
        if have_gyro and pre_avoid_yaw is not None:
            avoid_state = "RECENTER"
            recenter_end_time = now + GYRO_RECENTER_DURATION_SEC
            return (True, _gyro_recenter_angle(current_yaw, pre_avoid_yaw),
                    f"recenter_{avoid_color}")
        avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None

    # --- RECENTER: gyro-steer back to the pre-avoidance heading ---
    if avoid_state == "RECENTER":
        # Gyro vanished mid-recenter (or disabled) -> stop fighting, hand back.
        if not have_gyro or pre_avoid_yaw is None:
            avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None
        else:
            heading_err = abs(pre_avoid_yaw - current_yaw)
            if now >= recenter_end_time or heading_err <= GYRO_RECENTER_TOL_DEG:
                # Heading recovered (or time up) -> lane following takes it from
                # here and centres laterally between the walls.
                avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None
            else:
                return (True, _gyro_recenter_angle(current_yaw, pre_avoid_yaw),
                        f"recenter_{avoid_color}")

    # --- TURNING: keep steering away until the pillar is passed ---
    if avoid_state == "TURNING":
        color = avoid_color
        passed = False
        if target is None or target['class'] != color:
            passed = True  # pillar left the view -> we're past it
        else:
            cx = target['cx']
            if color == 'green' and cx > lab_w * AVOID_PASS_CX_FRAC:
                passed = True   # pillar now on our right -> we passed on its left
            elif color == 'red' and cx < lab_w * (1.0 - AVOID_PASS_CX_FRAC):
                passed = True   # pillar now on our left -> we passed on its right
        if passed:
            avoid_state = "REALIGN"
            realign_end_time = now + REALIGN_DURATION_SEC
            return True, SERVO_CENTER_ANGLE, f"realign_{color}"
        cx = target['cx'] if target else None
        return True, _avoid_servo_angle(color, front_dist, cx, lab_w), f"avoid_{color}"

    # --- IDLE: decide whether to start a maneuver ---
    if target is not None:
        # Engage as soon as EITHER the camera sees the pillar big enough OR the
        # LiDAR says we're inside the standoff. The camera width trigger is the
        # primary cue: the narrow LiDAR front beam frequently misses a thin
        # pillar (or reads the wall behind it), so relying on front_dist alone
        # let detected pillars slip through into LiDAR wall following.
        width_trigger = target['width'] >= AVOID_WIDTH_TRIGGER
        dist_trigger = (front_dist is not None
                        and front_dist <= (AVOID_STANDOFF_MM + AVOID_CLEARANCE_MM))
        start = width_trigger or dist_trigger
        if start:
            avoid_color = target['class']
            avoid_state = "TURNING"
            # Snapshot the heading we are on NOW so we can return to it after the
            # pass (None if no gyro -> RECENTER phase is simply skipped later).
            pre_avoid_yaw = current_yaw if have_gyro else None
            return True, _avoid_servo_angle(avoid_color, front_dist, target['cx'], lab_w), f"avoid_{avoid_color}"

    return False, None, "none"


def filter_blue_objects_lab(lab_frame):
    """Blue mask (LAB) for blue line lap counting."""
    return lab_mask(lab_frame, 'blue')


def detect_color_binary(mask, threshold=4000):
    """Returns True if color is present above a pixel threshold."""
    return cv2.countNonZero(mask) > threshold


def draw_detections(frame, dets, scale_x=1.0, scale_y=1.0):
    """Draw detection boxes on the display (full-res) frame."""
    for d in dets:
        x1, y1, x2, y2 = d['bbox']
        x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
        y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
        clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
        cv2.putText(frame, d['class'], (x1, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)


# ===================== DEBUG VISUALIZATION (rendering only) =====================
# Everything below is pure overlay/visualisation. It reads the SAME detections
# returned by detect_lab_pillars() and the SAME LiDAR scan_data the control
# loop uses, and never feeds anything back into control / PID / wall-following /
# avoidance / state-machine logic. Tuned to stay cheap on the Pi.

# --- Black wall boundary outlines (camera image) ---
# The split between "black wall" and floor is found ADAPTIVELY with Otsu each
# frame, then clamped into [LO, HI]. A fixed threshold worked for one driving
# direction but failed for the other because the walls are exposed differently
# clockwise vs anti-clockwise (different part of the arena / lighting); Otsu
# tracks that automatically so the outline holds in both directions.
VIZ_WALL_L_CLAMP_LO = 50         # never put the dark/floor cutoff below this
VIZ_WALL_L_CLAMP_HI = 135        # never call anything brighter than this a wall
VIZ_WALL_MAX_DARK_FRAC = 0.60    # if >60% of the frame reads "dark" the scene is
                                 # too dim to trust -> skip (avoids garbage)
VIZ_WALL_MIN_AREA = 1500         # ignore small dark blobs (LAB-frame px area)
VIZ_WALL_MAX_CONTOURS = 6        # draw only the N largest wall contours
VIZ_WALL_APPROX_EPS = 0.004      # polyline simplification (frac of contour perimeter)
VIZ_WALL_CLOSE_PX = 7            # close gaps so a wall reads as one contour
VIZ_WALL_COLOR = (0, 255, 255)   # BGR yellow outline (clearly visible on dark walls)
VIZ_WALL_THICKNESS = 2

# --- Fixed obstacle reference (origin) points ---
# Fractions of the DISPLAY frame size, so they stay fixed regardless of where
# obstacles are detected. Green sits bottom-left, red bottom-right.
VIZ_GREEN_ORIGIN_FRAC = (0.25, 0.93)
VIZ_RED_ORIGIN_FRAC = (0.75, 0.93)

# --- Top-down LiDAR map ---
VIZ_LIDAR_MAP_SIZE = 480         # square map side (px)
VIZ_LIDAR_MAX_RANGE_MM = 3000.0  # ranges beyond this are clipped to the edge
VIZ_LIDAR_WALL_COLOR = (219, 119, 31)   # BGR of matplotlib blue #1f77b4 (wall pts)
VIZ_LIDAR_BG_POINT_COLOR = (70, 70, 70)  # non-wall scan points, dim grey


def extract_wall_contours(lab_frame):
    """Find continuous outlines of the black walls in the LAB frame.

    Returns a list of contours (in LAB-frame pixel coords), largest first,
    already simplified into clean polylines via approxPolyDP. The control loop
    scales these up to the display resolution before drawing."""
    l_channel = lab_frame[:, :, 0]
    # Adaptive cutoff: Otsu finds the dark/bright split for the CURRENT exposure
    # so the walls are caught whether the run is clockwise or anti-clockwise.
    # Clamp it so a washed-out frame can't call the floor a wall, and a very dark
    # frame can't drop the cutoff to nothing.
    otsu_t, _ = cv2.threshold(l_channel, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cutoff = int(min(VIZ_WALL_L_CLAMP_HI, max(VIZ_WALL_L_CLAMP_LO, otsu_t)))
    # Walls become white. (Re-threshold at the clamped cutoff; Otsu's own mask
    # used its unclamped value, so we can't reuse it.)
    _, dark = cv2.threshold(l_channel, cutoff, 255, cv2.THRESH_BINARY_INV)
    # Guard: a frame that is mostly "dark" is too dim to trust -> draw nothing.
    if cv2.countNonZero(dark) > VIZ_WALL_MAX_DARK_FRAC * dark.size:
        return []
    if VIZ_WALL_CLOSE_PX > 0:
        k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (VIZ_WALL_CLOSE_PX, VIZ_WALL_CLOSE_PX))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, k, iterations=1)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, k, iterations=1)
    conts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in conts if cv2.contourArea(c) >= VIZ_WALL_MIN_AREA]
    valid.sort(key=cv2.contourArea, reverse=True)
    valid = valid[:VIZ_WALL_MAX_CONTOURS]
    out = []
    for c in valid:
        eps = VIZ_WALL_APPROX_EPS * cv2.arcLength(c, True)
        out.append(cv2.approxPolyDP(c, eps, True))
    return out


def draw_wall_contours(frame, contours, scale_x=1.0, scale_y=1.0):
    """Draw black-wall boundary polylines (scaled to the display frame)."""
    for c in contours:
        pts = c.reshape(-1, 2).astype(np.float32)
        pts[:, 0] *= scale_x
        pts[:, 1] *= scale_y
        cv2.polylines(frame, [pts.astype(np.int32)], True,
                      VIZ_WALL_COLOR, VIZ_WALL_THICKNESS, cv2.LINE_AA)


def _primary_obstacle(objs):
    """Mirror manage_color_avoidance's 'nearest' pick for DISPLAY only.

    Same key as the avoidance logic (lowest bbox base, area as tie-break) so the
    drawn connection line points at the obstacle the avoidance logic would act
    on. This is read-only and never affects control."""
    return max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None


def draw_obstacle_overlay(frame, dets, scale_x=1.0, scale_y=1.0):
    """Boxes + centre circles + fixed reference points + connection lines.

    Green obstacles connect to the fixed green reference point, red obstacles to
    the fixed red one. With multiple obstacles of a colour, the line goes to the
    primary (nearest) one the avoidance logic would use."""
    h, w = frame.shape[:2]
    gx, gy = int(VIZ_GREEN_ORIGIN_FRAC[0] * w), int(VIZ_GREEN_ORIGIN_FRAC[1] * h)
    rx, ry = int(VIZ_RED_ORIGIN_FRAC[0] * w), int(VIZ_RED_ORIGIN_FRAC[1] * h)

    def center(d):
        x1, y1, x2, y2 = d['bbox']
        return (int((x1 + x2) * 0.5 * scale_x), int((y1 + y2) * 0.5 * scale_y))

    # Boxes + class label + centre marker for every detection.
    for d in dets:
        x1, y1, x2, y2 = d['bbox']
        x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
        y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
        clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
        cx, cy = center(d)
        cv2.circle(frame, (cx, cy), 5, clr, -1)
        cv2.putText(frame, d['class'], (x1, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)

    # Connection line from each fixed reference point to its primary obstacle.
    g = _primary_obstacle([d for d in dets if d['class'] == 'green'])
    if g is not None:
        cv2.line(frame, (gx, gy), center(g), (0, 255, 0), 2, cv2.LINE_AA)
    r = _primary_obstacle([d for d in dets if d['class'] == 'red'])
    if r is not None:
        cv2.line(frame, (rx, ry), center(r), (0, 0, 255), 2, cv2.LINE_AA)

    # Fixed reference points drawn last so they sit on top of the lines.
    for (px, py), clr in (((gx, gy), (0, 255, 0)), ((rx, ry), (0, 0, 255))):
        cv2.circle(frame, (px, py), 7, clr, -1)
        cv2.circle(frame, (px, py), 9, (255, 255, 255), 2)


def render_lidar_map(scan_data, clockwise=True):
    """Top-down map of the current scan, with the wall-following points in blue.

    Uses the SAME polar->cartesian convention as the LiDAR data: 0 deg = front
    (up), +90 deg = right. The wall-following points are exactly the angle bands
    calculate_steering_error() samples for the current direction, so the blue
    dots trace the black walls the robot is following."""
    size = VIZ_LIDAR_MAP_SIZE
    img = np.zeros((size, size, 3), np.uint8)
    cx = cy = size // 2
    scale = (size * 0.45) / VIZ_LIDAR_MAX_RANGE_MM

    # Range rings (1 m / 2 m / 3 m) + heading line.
    for ring_mm in (1000, 2000, 3000):
        cv2.circle(img, (cx, cy), int(ring_mm * scale), (45, 45, 45), 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy), (cx, cy - int(size * 0.45)), (60, 60, 60), 1)

    # Same angle bands the wall-following error uses for this direction.
    if clockwise:
        wall_angles = set(range(30, 105)) | set(range(-90, -29))
    else:
        wall_angles = set(range(30, 91)) | set(range(-105, -29))

    if scan_data:
        for a, d in scan_data.items():
            if d is None or d <= 0:
                continue
            rad = np.radians(a)
            r = min(d, VIZ_LIDAR_MAX_RANGE_MM) * scale
            px = int(cx + r * np.sin(rad))
            py = int(cy - r * np.cos(rad))
            if not (0 <= px < size and 0 <= py < size):
                continue
            if a in wall_angles and 0 < d < 3000:
                cv2.circle(img, (px, py), 2, VIZ_LIDAR_WALL_COLOR, -1)
            else:
                cv2.circle(img, (px, py), 1, VIZ_LIDAR_BG_POINT_COLOR, -1)

    # Robot at centre + legend.
    cv2.circle(img, (cx, cy), 4, (255, 255, 255), -1)
    cv2.putText(img, "LiDAR wall-following pts", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, VIZ_LIDAR_WALL_COLOR, 1, cv2.LINE_AA)
    cv2.putText(img, "FRONT", (cx - 22, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA)
    return img


# Pre-built kernel for the colour masks
MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)

# --- ESP32 SERIAL CONSTANTS ---
SERVO_CENTER = 95
SERVO_MIN = 77
SERVO_MAX = 127
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- LIDAR CONTROL CONSTANTS ---
LIDAR_TARGET_DISTANCE_MM = 500
# Minimum safety clearance from the black boundary: the robot must never get
# closer than 8 cm to a wall ahead. calculate_steering_error() commands STOP
# once the front distance drops below this, so 8 cm is the hard floor.
LIDAR_SAFETY_DISTANCE_MM = 80
CLOCKWISE_WALL_FOLLOWING = True

# --- GYRO LANE RE-CENTERING (after each obstacle pass) ---
# After the robot finishes passing a pillar it is left angled/offset toward the
# wall it passed toward. We snapshot the heading just BEFORE the avoidance turn
# starts, then -- once the pillar is cleared -- use the gyro to steer the robot
# back onto that original heading so it ends up parallel to the walls and lined
# up for the next pillar (instead of drifting into the inner/outer wall).
# Auto-disables if the ESP32 never sends YAW (gyro_ok stays False).
GYRO_ENABLED = True
GYRO_RECENTER_DURATION_SEC = 1.3   # max time to spend straightening after a pass
GYRO_RECENTER_TOL_DEG = 4.0        # heading counts as recovered within this band
GYRO_RECENTER_KP = 1.2             # servo degrees per degree of heading error
GYRO_RECENTER_MAX_OFFSET = 22      # clamp the recenter steer (deg from centre)
# Flip to -1 if recentering steers the WRONG way on your IMU (i.e. it makes the
# heading error grow instead of shrink). Depends on the gyro's yaw sign.
GYRO_RECENTER_SIGN = +1

# PID parameters (tuned for corridor centering with closest-wall distances)
LIDAR_PID_KP = 0.12
LIDAR_PID_KI = 0.002
LIDAR_PID_KD = 0.05

LIDAR_SERVO_MIN_ANGLE = 77
LIDAR_SERVO_MAX_ANGLE = 127
LIDAR_STEERING_SCALE_FACTOR = 0.25

# LiDAR side-check parameters
LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE = 40
LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE = 75
LIDAR_RIGHT_SIDE_DISTANCE_MM = 180
LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE = -75
LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE = -40
LIDAR_LEFT_SIDE_DISTANCE_MM = 180
LIDAR_SIDE_STEER_MAGNITUDE = 20

# --- DEBUGGING AND UI ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = False

# --- WEB STREAM TUNING (keep the 6 live feeds from saturating the Pi/Wi-Fi) ---
# Each browser <img> opens its own MJPEG generator, so 6 feeds = 6 JPEG
# encodes per cycle. Downscale + cap FPS + lower quality keeps it smooth.
STREAM_WIDTH = 480          # output width per feed (px); aspect kept
STREAM_MAX_FPS = 12         # cap per-feed frame rate
STREAM_JPEG_QUALITY = 65    # JPEG quality (lower = smaller/faster)

STARTUP_TEST_ENABLED = False

# Run blue/orange line masking every Nth control loop.
LINE_CHECK_EVERY_N_LOOPS = 2


# --- BEHAVIOR STATES ---
class RobotState:
    IMMINENT_COLLISION_AVOIDANCE = "IMMINENT_COLLISION_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    RED_AVOIDANCE = "RED_AVOIDANCE"
    GREEN_AVOIDANCE = "GREEN_AVOIDANCE"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    STOP = "STOP"
    INITIALIZING = "INITIALIZING"
    FALLBACK_STRAIGHT = "FALLBACK_STRAIGHT"

current_robot_state = RobotState.INITIALIZING


# ===================== ESP32 SERIAL =====================
ser = None
ESP32_OK = False

if serial is not None:
    try:
        ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.1)
        time.sleep(2)
        ESP32_OK = True
        print(f"[SERIAL] ESP32 on {PI_TO_ESP_PORT}")
    except Exception as e:
        print(f"[SERIAL] Fail: {e}")
else:
    print("[SERIAL] pyserial not available")

def cmd(angle, speed):
    if ser and ESP32_OK:
        packet = f"STR:{angle},SPD:{int(speed * 255)}\n"
        ser.write(packet.encode())
        ser.flush()

def stop_robot():
    if ser and ESP32_OK:
        ser.write(f"STR:{SERVO_CENTER},SPD:0\n".encode())
        ser.flush()

def startup_test():
    if not (ser and ESP32_OK):
        print("[STARTUP] Skipped - no serial")
        return
    print("[STARTUP] Sweeping servo...")
    for angle in [SERVO_CENTER - 20, SERVO_CENTER, SERVO_CENTER + 20, SERVO_CENTER]:
        ser.write(f"STR:{angle},SPD:0\n".encode())
        ser.flush()
        time.sleep(0.3)
    print("[STARTUP] Motor pulse...")
    cmd(SERVO_CENTER, 0.4)
    time.sleep(0.4)
    stop_robot()
    print("[STARTUP] Done")


# --- Helper Functions ---
def map_lidar_steering_angle(center_angle, pid_output, clockwise=True):
    adjusted_output = -1 * pid_output * LIDAR_STEERING_SCALE_FACTOR
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    return max(LIDAR_SERVO_MIN_ANGLE, min(angle, LIDAR_SERVO_MAX_ANGLE))

def check_lidar_side_alerts(scan_data):
    if not scan_data: return None
    for angle, distance in scan_data.items():
        if LIDAR_RIGHT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_RIGHT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_RIGHT_SIDE_DISTANCE_MM:
            return "RIGHT"
    for angle, distance in scan_data.items():
        if LIDAR_LEFT_SIDE_CHECK_MIN_ANGLE <= angle <= LIDAR_LEFT_SIDE_CHECK_MAX_ANGLE and 0 < distance < LIDAR_LEFT_SIDE_DISTANCE_MM:
            return "LEFT"
    return None

def check_front_obstacle_proximity(scan_data, distance_mm=1000):
    if not scan_data: return False
    for angle, dist in scan_data.items():
        if -2 <= angle <= 2 and 0 < dist < distance_mm:
            return True
    return False

def get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=2, speed=ROBOT_MANEUVER_SPEED):
    """
    Analyzes LiDAR data to choose the most open path (left or right),
    sets the global wall-following direction, and executes the turn.
    """
    global CLOCKWISE_WALL_FOLLOWING

    start_time = time.time()
    end_time = start_time + duration_sec

    if not scan_data:
        print("Parking Maneuver Warning: No LiDAR data. Defaulting to RIGHT turn.")
        CLOCKWISE_WALL_FOLLOWING = True
    else:
        left_distances = [dist for angle, dist in scan_data.items() if -90 <= angle <= -40 and dist > 0]
        right_distances = [dist for angle, dist in scan_data.items() if 40 <= angle <= 90 and dist > 0]

        avg_left = np.mean(left_distances) if left_distances else 0
        avg_right = np.mean(right_distances) if right_distances else 0

        print(f"Parking Maneuver Analysis: Avg Left Space={avg_left:.0f}mm, Avg Right Space={avg_right:.0f}mm")

        if avg_left > avg_right:
            CLOCKWISE_WALL_FOLLOWING = False
            print("Decision: Turning LEFT (Anti-Clockwise). Setting wall-following mode.")
        else:
            CLOCKWISE_WALL_FOLLOWING = True
            print("Decision: Turning RIGHT (Clockwise). Setting wall-following mode.")

    direction_multiplier = 1 if CLOCKWISE_WALL_FOLLOWING else -1
    servo_angle = SERVO_CENTER_ANGLE + (direction_multiplier * max_angle_magnitude)
    print(f"Executing escape maneuver with Servo Angle: {servo_angle}")

    while time.time() < end_time:
        cmd(servo_angle, speed)
        time.sleep(0.05)

    stop_robot()
    time.sleep(0.5)

def check_imminent_collision_and_get_escape_route(scan_data):
    """
    Checks for an imminent forward collision and determines the best escape direction.
    Trigger: Any distance < 100mm in the -10 to +10 degree range.
    Returns: "LEFT", "RIGHT", or None.
    """
    if not scan_data:
        return None

    is_collision_imminent = False
    for angle, distance in scan_data.items():
        if -10 <= angle <= 10 and 0 < distance < 100:
            is_collision_imminent = True
            break

    if not is_collision_imminent:
        return None

    left_distances = [d for a, d in scan_data.items() if -90 <= a < 0 and d > 0]
    right_distances = [d for a, d in scan_data.items() if 0 < a <= 90 and d > 0]

    avg_left = np.mean(left_distances) if left_distances else 0
    avg_right = np.mean(right_distances) if right_distances else 0

    if avg_left > avg_right:
        return "LEFT"
    else:
        return "RIGHT"


def check_for_straight_corridor(scan_data, min_dist_mm=1000, max_dist_mm=3500, angle_range=10):
    if not scan_data:
        return False

    left_front_distances = []
    right_front_distances = []
    angle_range_max = angle_range
    for angle, distance in scan_data.items():
        if -1*angle_range_max <= angle < 0 and distance > 0:
            left_front_distances.append(distance)
        elif 0 <= angle <= angle_range_max and distance > 0:
            right_front_distances.append(distance)

    if not left_front_distances or not right_front_distances:
        return False

    avg_left_dist = sum(left_front_distances) / len(left_front_distances)
    avg_right_dist = sum(right_front_distances) / len(right_front_distances)

    is_left_in_range = min_dist_mm < avg_left_dist < max_dist_mm
    is_right_in_range = min_dist_mm < avg_right_dist < max_dist_mm

    return is_left_in_range and is_right_in_range

# --- LiDAR Data Acquisition Thread ---
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("LiDAR acquisition thread started.")
    try:
        while True:
            if not any(t.name == 'MainThread' and t.is_alive() for t in threading.enumerate()):
                break
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data
            time.sleep(0.01)
    except Exception as e:
        print(f"LiDAR Acquisition Thread Error: {e}")
    finally:
        print("LiDAR acquisition thread stopping.")

# --- Gyro Acquisition Thread ---
def gyro_acquisition_thread_func(serial_obj, stop_event):
    """Read the ESP32's "YAW:<deg>" stream off the shared serial link.

    Only reads (the control loop only writes), so the two directions don't
    collide on the full-duplex port. Sets gyro_ok once any yaw is parsed;
    until then the recentering feature stays inert."""
    global latest_yaw, gyro_ok
    if serial_obj is None:
        print("[GYRO] No serial link; gyro recentering disabled.")
        return
    print("Gyro acquisition thread started.")
    try:
        while not stop_event.is_set():
            try:
                raw = serial_obj.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                time.sleep(0.02)
                continue
            if raw.startswith("YAW:"):
                try:
                    y = float(raw.split(":", 1)[1])
                except ValueError:
                    continue
                with yaw_lock:
                    latest_yaw = y
                if not gyro_ok:
                    gyro_ok = True
                    print("[GYRO] Yaw stream live; lane recentering active.")
    except Exception as e:
        print(f"Gyro Acquisition Thread Error: {e}")
    finally:
        print("Gyro acquisition thread stopping.")


# --- Camera Acquisition Thread ---
def camera_acquisition_thread_func(picam2_instance, stop_event, lab_processing_size):
    """
    Captures the hardware-scaled lores stream (full wide-angle FOV, already
    downscaled by the ISP) and prepares the BGR + LAB frames. The full-res
    main stream is never pulled into Python.
    """
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition and processing thread started.")
    frame_seq = 0
    try:
        while not stop_event.is_set():
            # 1. Capture the lores stream (YUV420 planar, processing size)
            yuv420 = picam2_instance.capture_array("lores")
            capture_ts = time.time()

            # 2. Convert YUV420 -> BGR (cheap at lores size)
            frame_bgr = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)

            # 3. Smaller LAB frame for colour detection
            lab_source_frame = cv2.resize(
                frame_bgr,
                lab_processing_size,
                interpolation=cv2.INTER_AREA
            )
            lab_frame = cv2.cvtColor(lab_source_frame, cv2.COLOR_BGR2LAB)

            # 4. Publish the prepared frames with seq + timestamp
            frame_seq += 1
            with camera_frame_lock:
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['lab'] = lab_frame
                latest_processed_frames['seq'] = frame_seq
                latest_processed_frames['ts'] = capture_ts

            # Throttle to the camera frame rate. Without this the loop spins
            # flat-out doing cvtColor/resize/LAB on every spare CPU cycle,
            # pinning a core and starving the control loop (-> stuttery motors).
            time.sleep(max(0.0, (1.0 / CAMERA_FRAMERATE) - (time.time() - capture_ts)))

    except Exception as e:
        print(f"Camera Acquisition Thread Error: {e}")
    finally:
        print("Camera acquisition thread stopping.")


# --- Main Robot Control Loop ---
def robot_control_loop(shutdown_event):
    global output_frame, output_frame_lock, current_robot_state, latest_camera_frame, camera_frame_lock, camera_thread_stop_event
    global straight_detected_time, OUT_PARKING_MANEUVER, START_PAUSE_DURATION, previous_increment_time, turn_counter, max_turn_count, DELAY_BETWEEN_TURNS
    global ROBOT_SPEED_MAX, ROBOT_MANEUVER_SPEED, ROBOT_CRUISE_SPEED
    global CLOCKWISE_WALL_FOLLOWING

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        lores={"size": (PROCESSING_WIDTH, PROCESSING_HEIGHT)},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started: main {CAMERA_RESOLUTION} (FOV reference), "
          f"lores {(PROCESSING_WIDTH, PROCESSING_HEIGHT)} at {CAMERA_FRAMERATE} FPS.")

    time.sleep(1)
    lab_processing_size = (LAB_PROCESSING_WIDTH, LAB_PROCESSING_HEIGHT)

    camera_acquisition_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, lab_processing_size)
    )
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    # Start reading the gyro yaw from the ESP32 (no-op if no serial / no YAW).
    if ser and ESP32_OK:
        try:
            ser.write(b"RST_YAW\n")   # zero the heading at startup
            ser.flush()
        except Exception:
            pass
    gyro_acquisition_thread = threading.Thread(
        target=gyro_acquisition_thread_func,
        args=(ser, camera_thread_stop_event)
    )
    gyro_acquisition_thread.daemon = True
    gyro_acquisition_thread.start()

    # STAGED STARTUP: let the camera's current draw settle before the LiDAR spin-up.
    print(f"[POWER] Camera settled. Waiting {STARTUP_STAGE_DELAY_SEC}s before LiDAR spin-up...")
    time.sleep(STARTUP_STAGE_DELAY_SEC)

    lidar_scanner, lidar_pid, lidar_acquisition_thread = None, None, None
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
        print("LiDAR system initialized successfully.")
    except (IOError, Exception) as e:
        print(f"WARNING: Failed to initialize LiDAR system: {e}.")
        lidar_scanner = None

    if STARTUP_TEST_ENABLED:
        startup_test()
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    try:
        first_loop = True
        blue_count = 0
        orange_count = 0
        max_line_crossings = 12
        prev_blue_state = False
        prev_orange_state = False
        blue_cooldown_end_time = 0
        orange_cooldown_end_time = 0
        loop_counter = 0
        program_start_time = time.monotonic()
        out_direction = None
        crossed_12 = False
        crossed_time = 0
        print_timer = 0

        while not shutdown_event.is_set():
            loop_start_time = time.monotonic()
            loop_counter += 1

            with camera_frame_lock:
                if 'bgr' not in latest_processed_frames:
                    time.sleep(0.01)
                    continue
                frame_bgr = latest_processed_frames['bgr'].copy()
                lab = latest_processed_frames['lab'].copy()

            scan_data = None
            if lidar_scanner:
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            # Latest gyro heading (None until the ESP32 actually streams YAW).
            current_yaw = None
            if gyro_ok:
                with yaw_lock:
                    current_yaw = latest_yaw

            if first_loop and OUT_PARKING_MANEUVER:
                print("Executing parking lot escape maneuver...")
                out_direction = get_out_of_parking_lot_maneuver(scan_data, max_angle_magnitude=35, duration_sec=1.25, speed=0.5)
                first_loop = True
                OUT_PARKING_MANEUVER = True

            # Line counting only every Nth loop (LAB blue mask).
            if loop_counter % LINE_CHECK_EVERY_N_LOOPS == 0:
                current_time = time.time()
                blue_mask = filter_blue_objects_lab(lab)
                blue_in_view = detect_color_binary(blue_mask)

                if not blue_in_view and prev_blue_state:
                    if current_time > blue_cooldown_end_time:
                        blue_count += 1
                        print(f"Blue line crossed! Total blue lines: {blue_count}")
                        blue_cooldown_end_time = current_time + 6
                prev_blue_state = blue_in_view

            if loop_counter % 3 == 0 and time.time() - print_timer >= 0.5:
                print_timer = time.time()
                elapsed_time = time.monotonic() - program_start_time
                print(f"[{loop_counter}] Blue:{blue_count}/{max_line_crossings} Time:{elapsed_time:.1f}s")

            # --- STOPPING LOGIC BASED ON LINE COUNT ---
            if not crossed_12 and blue_count >= max_line_crossings:
                print(f"Max line count ({max_line_crossings}) reached for blue")
                crossed_12 = True
                crossed_time = time.time()
                blue_count = 0

            if crossed_12 and (time.time() - crossed_time) > 4:
                stop_robot()
                time.sleep(120)
                break

            is_near_field_mode = check_front_obstacle_proximity(scan_data, distance_mm=1100)

            # --- FUSION: LiDAR obstacle + boundary detection + camera colour ---
            lidar_obstacles = detect_lidar_obstacles(scan_data) if scan_data else []
            is_boundary, boundary_dist, boundary_open_side = detect_lidar_boundary(scan_data) if scan_data else (False, None, None)
            # Camera detections are still needed for colour classification.
            detections = detect_lab_pillars(lab)
            det_method = "FUSION"
            # Fusion avoidance: LiDAR gives position, camera gives colour.
            avoid_engaged, avoid_angle_val, logic_label = manage_fusion_avoidance(
                lidar_obstacles, detections, scan_data, lab.shape[1], current_yaw)

            label_method = f"[{det_method}]"

            escape_direction = check_imminent_collision_and_get_escape_route(scan_data)
            side_alert_status = check_lidar_side_alerts(scan_data)

            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # --- BEHAVIOR ARBITRATION ---
            # PRIORITY 0: IMMINENT COLLISION (always highest)
            if escape_direction == "LEFT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: ESCAPE LEFT!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("Imminent Collision: Escaping LEFT")
            elif escape_direction == "RIGHT":
                current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + 20
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: ESCAPE RIGHT!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("Imminent Collision: Escaping RIGHT")

            # PRIORITY 1: FUSION OBSTACLE (LiDAR position + camera colour).
            # GREEN -> pass on the LEFT, RED -> pass on the RIGHT.
            # Once engaged we COMMIT — outranks side alerts.
            elif avoid_engaged:
                robot_speed_current = ROBOT_MANEUVER_SPEED
                if 'red' in logic_label:
                    current_robot_state = RobotState.RED_AVOIDANCE
                elif 'green' in logic_label:
                    current_robot_state = RobotState.GREEN_AVOIDANCE
                else:
                    current_robot_state = RobotState.IMMINENT_COLLISION_AVOIDANCE
                target_servo_angle = avoid_angle_val
                display_text = f"MODE: {logic_label} {det_method} | {int(round(target_servo_angle))}deg"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print(f"Fusion: {logic_label} -> servo {round(target_servo_angle)}")

            # PRIORITY 2: ARENA BOUNDARY (LiDAR front filled with wall).
            # Wall-follow along the boundary — steer toward the open side.
            elif is_boundary:
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                robot_speed_current = ROBOT_CRUISE_SPEED
                if boundary_open_side == "LEFT":
                    # More space on the left -> steer left to follow along it
                    target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                else:
                    target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                display_text = f"MODE: BOUNDARY | Follow {boundary_open_side} | Dist:{boundary_dist:.0f}mm"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print(f"Boundary detected: follow {boundary_open_side}, "
                          f"front dist={boundary_dist:.0f}mm")

            # PRIORITY 3: SIDE OBSTACLE (LIDAR)
            elif side_alert_status == "RIGHT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Right LiDAR!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("LiDAR Side: RIGHT")
            elif side_alert_status == "LEFT":
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Left LiDAR!"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print("LiDAR Side: LEFT")

            # PRIORITY 3: LIDAR WALL FOLLOWING
            elif lidar_scanner and lidar_pid:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                if scan_data:
                    if straight_detected_time > 0 and (time.time() - straight_detected_time) < straight_override_duration:
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "MODE: LiDARWF | Straight Ovrd"
                        if time.time() - print_timer >= 0.5:
                            print_timer = time.time()
                            print("LiDAR: Straight override active")
                    elif check_for_straight_corridor(scan_data, min_dist_mm=1000, max_dist_mm=3500, angle_range=10):
                        straight_detected_time = time.time()
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "MODE: LiDARWF | Straight"
                        if time.time() - print_timer >= 0.5:
                            print_timer = time.time()
                            print("LiDAR: Straight corridor detected")
                    else:
                        straight_detected_time = 0.0
                        lidar_error = calculate_steering_error(
                            scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM,
                            clockwise=CLOCKWISE_WALL_FOLLOWING
                        )
                        if lidar_error == 9999.0:
                            stop_robot()
                            current_robot_state = RobotState.STOP
                            display_text = "MODE: STOP"
                            time.sleep(0.1)
                            continue
                        pid_output = lidar_pid.update(lidar_error)
                        target_servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, clockwise=CLOCKWISE_WALL_FOLLOWING)
                        display_text = f"MODE: LiDARWF | Steer: {round(target_servo_angle)}deg | Err: {lidar_error:.0f}mm"
                else:
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE
                    display_text = "MODE: Fallback (No LiDAR)"

            # PRIORITY 4: FALLBACK
            else:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                display_text = f"MODE: Fallback | Logic: {logic_label}"

            # APPLY ROBOT MOTION
            final_angle = SERVO_CENTER_ANGLE
            if current_robot_state != RobotState.STOP:
                if check_front_obstacle_proximity(scan_data, distance_mm=150):
                    min_angle_limit = LIDAR_SERVO_MIN_ANGLE - 5
                    max_angle_limit = LIDAR_SERVO_MAX_ANGLE + 5
                else:
                    min_angle_limit = LIDAR_SERVO_MIN_ANGLE
                    max_angle_limit = LIDAR_SERVO_MAX_ANGLE

                final_angle = int(round(np.clip(target_servo_angle, min_angle_limit, max_angle_limit)))

                deviation = abs(final_angle - SERVO_CENTER_ANGLE)
                if deviation > 15:
                    turn_speed = ROBOT_CRUISE_SPEED * 0.75
                    cmd(final_angle, turn_speed)
                else:
                    cmd(final_angle, robot_speed_current)
            else:
                stop_robot()

            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0

            if time.time() - print_timer >= 0.5:
                print_timer = time.time()
                print(f"[{loop_counter}] {current_robot_state} | Angle:{final_angle} Speed:{robot_speed_current:.2f} FPS:{int(fps)} LiDAR_obs:{len(lidar_obstacles)} Cam:{len(detections)} {logic_label}")

            # Build the ORIGINAL display frame (with detection boxes) for the web page.
            if STREAM_VIDEO:
                processed_frame = frame_bgr.copy()
                # LAB detections were computed on the smaller lab frame; scale boxes up.
                scale_x = frame_bgr.shape[1] / lab.shape[1]
                scale_y = frame_bgr.shape[0] / lab.shape[0]
                # Black wall outlines first (so boxes/markers draw on top of them),
                # then obstacle boxes + centre circles + fixed reference points
                # and the green/red connection lines. All read-only overlays.
                wall_contours = extract_wall_contours(lab)
                draw_wall_contours(processed_frame, wall_contours, scale_x, scale_y)
                draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)
                # Draw LiDAR obstacle markers (cyan wedges) on the display.
                fh, fw = processed_frame.shape[:2]
                for obs in lidar_obstacles:
                    a_rad = np.radians(obs['angle_deg'])
                    lx = int(fw * 0.5 + fw * 0.4 * np.sin(a_rad))
                    ly = int(fh * 0.85 - fh * 0.4 * np.cos(a_rad))
                    cv2.circle(processed_frame, (lx, ly), 8, (255, 255, 0), 2)
                    cv2.putText(processed_frame, f"{obs['distance_mm']:.0f}mm",
                                (lx + 10, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (255, 255, 0), 1)
                if DEBUG_UI_OVERLAYS:
                    cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(processed_frame, f"FPS: {int(fps)}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                with output_frame_lock:
                    output_frame = processed_frame

            pace_remaining = (1.0 / CONTROL_LOOP_MAX_HZ) - (time.monotonic() - loop_start_time)
            if pace_remaining > 0:
                time.sleep(pace_remaining)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down gracefully...")

    finally:
        print("Control loop ending. Cleaning up resources...")
        shutdown_event.set()
        camera_thread_stop_event.set()
        if camera_acquisition_thread and camera_acquisition_thread.is_alive():
            camera_acquisition_thread.join(timeout=2)
        stop_robot()
        if lidar_scanner:
            print("Disconnecting LiDAR...")
            lidar_scanner.disconnect()
        try:
            picam2.stop()
        except:
            pass
        print("All resources released.")


# --- Flask Streaming Functions ---
def _get_latest_bgr():
    with camera_frame_lock:
        if 'bgr' in latest_processed_frames:
            return latest_processed_frames['bgr'].copy()
    return None


def _segment_for_stream(color):
    """Segment the latest frame to one colour, reusing the already-converted
    LAB frame published by the camera thread (no second BGR->LAB per feed)."""
    with camera_frame_lock:
        if 'lab' not in latest_processed_frames:
            return None
        lab = latest_processed_frames['lab']
        bgr = latest_processed_frames['bgr']
    # The published LAB frame is smaller than the BGR frame; match sizes so
    # the mask lines up, working at the cheaper LAB resolution.
    small_bgr = cv2.resize(bgr, (lab.shape[1], lab.shape[0]), interpolation=cv2.INTER_NEAREST)
    mask = lab_mask(lab, color)
    return cv2.bitwise_and(small_bgr, small_bgr, mask=mask)


def _segment_track_for_stream():
    """Show ONLY the white drivable track; everything else is blacked out.

    Reuses the exact same build_track_mask() the detection logic relies on: the
    single connected white floor region bounded by the black walls. Pixels
    outside that region (walls, banners, ceiling, off-track colour) are zeroed."""
    with camera_frame_lock:
        if 'lab' not in latest_processed_frames:
            return None
        lab = latest_processed_frames['lab']
        bgr = latest_processed_frames['bgr']
    small_bgr = cv2.resize(bgr, (lab.shape[1], lab.shape[0]), interpolation=cv2.INTER_NEAREST)
    track = build_track_mask(lab_mask(lab, 'white'))
    return cv2.bitwise_and(small_bgr, small_bgr, mask=track)


def _segment_hsv_for_stream(color):
    """Segment the latest frame using HSV presets for web tuning."""
    with camera_frame_lock:
        if 'bgr' not in latest_processed_frames:
            return None
        bgr = latest_processed_frames['bgr']
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    with hsv_presets_lock:
        p = dict(HSV_PRESETS[color])
    mask = cv2.inRange(
        hsv,
        np.array([p["h_min"], p["s_min"], p["v_min"]]),
        np.array([p["h_max"], p["s_max"], p["v_max"]]),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)
    return cv2.bitwise_and(bgr, bgr, mask=mask)


def _downscale_for_stream(img):
    target_w = STREAM_WIDTH
    if img.shape[1] > target_w:
        h = int(img.shape[0] * target_w / img.shape[1])
        img = cv2.resize(img, (target_w, h), interpolation=cv2.INTER_AREA)
    return img


def generate_stream(mode):
    """Generate an MJPEG stream for 'original' or one of the colour names.

    The width / FPS / quality knobs are read live each iteration so they can
    be tuned from the web UI without restarting."""
    last_emit = 0.0
    while True:
        if not STREAM_VIDEO:
            time.sleep(0.5)
            continue

        # Per-feed FPS cap so 6 streams don't burn the CPU spinning.
        min_interval = 1.0 / max(1, STREAM_MAX_FPS)
        wait = min_interval - (time.monotonic() - last_emit)
        if wait > 0:
            time.sleep(wait)
        last_emit = time.monotonic()

        if mode == "original":
            # Use the annotated control-loop frame if available.
            with output_frame_lock:
                img = output_frame
            if img is None:
                img = _get_latest_bgr()
        elif mode == "lidar":
            # Top-down LiDAR map with blue wall-following points.
            with lidar_data_lock:
                scan = dict(latest_lidar_data)
            img = render_lidar_map(scan, CLOCKWISE_WALL_FOLLOWING)
        elif mode == "track":
            # White drivable track only; everything else black.
            img = _segment_track_for_stream()
        else:
            img = _segment_for_stream(mode)

        if img is None:
            time.sleep(0.01)
            continue

        img = _downscale_for_stream(img)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_JPEG_QUALITY]
        (flag, encoded_image) = cv2.imencode(".jpg", img, encode_params)
        if not flag:
            continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')


def generate_hsv_stream(mode):
    """Generate an MJPEG stream using HSV presets for web tuning."""
    last_emit = 0.0
    while True:
        if not STREAM_VIDEO:
            time.sleep(0.5)
            continue
        min_interval = 1.0 / max(1, STREAM_MAX_FPS)
        wait = min_interval - (time.monotonic() - last_emit)
        if wait > 0:
            time.sleep(wait)
        last_emit = time.monotonic()
        img = _segment_hsv_for_stream(mode)
        if img is None:
            time.sleep(0.01)
            continue
        img = _downscale_for_stream(img)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_JPEG_QUALITY]
        (flag, encoded_image) = cv2.imencode(".jpg", img, encode_params)
        if not flag:
            continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')


@app.route("/")
def index():
    with presets_lock:
        lab_defaults = dict(COLOR_PRESETS)
    with hsv_presets_lock:
        hsv_defaults = dict(HSV_PRESETS)
    return render_template("obs_lab.html", colors=STREAM_COLORS,
                           lab_presets_json=json.dumps(lab_defaults),
                           hsv_presets_json=json.dumps(hsv_defaults))


@app.route("/video/<mode>")
def video_feed(mode):
    if mode not in ("original", "lidar", "track") and mode not in STREAM_COLORS:
        return "Unknown stream", 404
    return Response(generate_stream(mode), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/get_stream")
def get_stream():
    return jsonify({"width": STREAM_WIDTH, "fps": STREAM_MAX_FPS,
                    "quality": STREAM_JPEG_QUALITY})


@app.route("/set_stream", methods=["POST"])
def set_stream():
    global STREAM_WIDTH, STREAM_MAX_FPS, STREAM_JPEG_QUALITY
    d = request.json
    if "width" in d:
        STREAM_WIDTH = max(120, min(1280, int(d["width"])))
    if "fps" in d:
        STREAM_MAX_FPS = max(1, min(30, int(d["fps"])))
    if "quality" in d:
        STREAM_JPEG_QUALITY = max(10, min(95, int(d["quality"])))
    return jsonify({"width": STREAM_WIDTH, "fps": STREAM_MAX_FPS,
                    "quality": STREAM_JPEG_QUALITY})


@app.route("/get_presets")
def get_presets():
    with presets_lock:
        return jsonify(COLOR_PRESETS)


@app.route("/update", methods=["POST"])
def update():
    mode = request.json["mode"]
    values = request.json["values"]
    with presets_lock:
        if mode in COLOR_PRESETS:
            COLOR_PRESETS[mode].update(values)
    return jsonify({"ok": True})


@app.route("/save")
def save():
    with presets_lock:
        snapshot = {k: dict(v) for k, v in COLOR_PRESETS.items()}
    try:
        with open(PRESETS_FILE, "w") as f:
            json.dump(snapshot, f, indent=4)
        return jsonify({"saved": True})
    except Exception as e:
        print(f"[LAB] Save failed: {e}")
        return jsonify({"saved": False})


@app.route("/load")
def load():
    loaded = load_presets()
    with presets_lock:
        COLOR_PRESETS.clear()
        COLOR_PRESETS.update(loaded)
    return jsonify({"loaded": True})


# ===================== HSV TUNING ENDPOINTS =====================

@app.route("/get_hsv_presets")
def get_hsv_presets():
    with hsv_presets_lock:
        return jsonify(HSV_PRESETS)


@app.route("/update_hsv", methods=["POST"])
def update_hsv():
    mode = request.json["mode"]
    values = request.json["values"]
    with hsv_presets_lock:
        if mode in HSV_PRESETS:
            HSV_PRESETS[mode].update(values)
    return jsonify({"ok": True})


@app.route("/save_hsv")
def save_hsv():
    with hsv_presets_lock:
        snapshot = {k: dict(v) for k, v in HSV_PRESETS.items()}
    try:
        with open(HSV_PRESETS_FILE, "w") as f:
            json.dump(snapshot, f, indent=4)
        return jsonify({"saved": True})
    except Exception as e:
        print(f"[HSV] Save failed: {e}")
        return jsonify({"saved": False})


@app.route("/load_hsv")
def load_hsv():
    loaded = load_hsv_presets()
    with hsv_presets_lock:
        HSV_PRESETS.clear()
        HSV_PRESETS.update(loaded)
    return jsonify({"loaded": True})


@app.route("/video_hsv/<mode>")
def video_feed_hsv(mode):
    if mode not in STREAM_COLORS:
        return "Unknown stream", 404
    return Response(generate_hsv_stream(mode), mimetype="multipart/x-mixed-replace; boundary=frame")


# --- Main Execution Block ---
if __name__ == '__main__':
    print("--- Starting Robot Control System (LiDAR + Camera Fusion) ---")
    stop_robot()
    time.sleep(0.5)

    control_thread = threading.Thread(target=robot_control_loop, args=(shutdown_event,))
    control_thread.start()
    print("Robot control thread started.")

    def handle_sigint(sig, frame):
        print("\nSIGINT received. Shutting down...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        hostname = os.uname()[1]
        print(f"Web server starting. Open http://{hostname}.local:5000 or http://<your_pi_ip>:5000")
    except AttributeError:
         import socket
         hostname = socket.gethostname()
         ip_address = socket.gethostbyname(hostname)
         print(f"Web server starting. Open http://{ip_address}:5000")

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass

    shutdown_event.set()
    print("Waiting for control thread to stop...")
    control_thread.join(timeout=5)
    stop_robot()
    print("Main application exiting.")
