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
import serial
import signal  # Native signal event tracking utility

# --- IMPORT CUSTOM VISION AND LIDAR EXTENSIONS ---
try: 
    from image_frame_combine_outer_inner_depth import process_frame_for_steering
    from lidar_steering_new import LidarScanner, PIDController, calculate_steering_error, get_wall_parallel_error, get_wall_parallel_sector_stats, PARALLEL_TOLERANCE_MM
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to mount local tracking components: {e}")
    sys.exit(1)

# --- GLOBAL SHUTDOWN SYSTEM TRACKERS ---
global_shutdown_event = threading.Event()  # Master termination trigger event flag
esp_ser = None                              # Global handle for ESP32 serial link
lidar_scanner = None                        # Global handle for LiDAR object
picam2 = None                               # Global handle for camera driver

# --- LIDAR CONTROL DESIGN PARAMETERS ---
LIDAR_TARGET_DISTANCE_MM = 500
LIDAR_SAFETY_DISTANCE_MM = 200  # Front trigger distance line (20 cm)
WALL_LOSS_THRESHOLD_MM = 350.0  # Open pocket validation limit to ignore missing walls (NORMAL driving only)
CLOCKWISE_WALL_FOLLOWING = True  # Dynamically modified tracking direction flag
WALL_FOLLOW_TARGET_MM = 600
# Configurations for the close-range side panic600 state
LIDAR_RIGHT_SIDE_DISTANCE_MM = 260  # 26cm side distance panic limit (was 180): trigger side avoidance earlier
LIDAR_LEFT_SIDE_DISTANCE_MM = 260   # 26cm side distance panic limit (was 180): trigger side avoidance earlier
LIDAR_SIDE_STEER_MAGNITUDE = 20     # Fixed steering shift magnitude away from side walls

# NEW: Imminent side-collision tier (tighter threshold, stronger steer). Fires BEFORE
# the normal 18cm panic band is reached so the robot reacts early while brushing a wall.
IMMINENT_RIGHT_SIDE_MIN_ANGLE = 50
IMMINENT_RIGHT_SIDE_MAX_ANGLE = 80
IMMINENT_RIGHT_SIDE_DISTANCE_MM = 160  # (was 120): urgent tier now fires earlier
IMMINENT_LEFT_SIDE_MIN_ANGLE = -80
IMMINENT_LEFT_SIDE_MAX_ANGLE = -50
IMMINENT_LEFT_SIDE_DISTANCE_MM = 160   # (was 120): urgent tier now fires earlier
IMMINENT_SIDE_STEER_MAGNITUDE = 35

# Hardware Servo Limits
LIDAR_SERVO_MIN_ANGLE = 10
LIDAR_SERVO_MAX_ANGLE = 170

# --- OBSTACLE SIGHT THRESHOLD BOUNDARIES ---
FRONT_TURN_TRIGGER_MM = 200.0  # Strict 20cm front trigger boundary
FRONT_SCAN_ANGLE_DEG = 15      # Width of front scan cone (+/- 15°)

# --- GLOBAL BUFFER LOCKS AND REGISTERS ---
output_frame = None
output_frame_lock = threading.Lock()

latest_lidar_data = {}
lidar_data_lock = threading.Lock()
latest_tof_distance_mm = None
tof_data_lock = threading.Lock()
tof_sensor = None

latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

# Vision overlay results computed in the camera thread (track-mask pillar detections,
# wall contours, LAB frame) so the control loop never does heavy segmentation work.
latest_detections = []
latest_wall_contours = []
latest_lab_shape = None
vision_overlay_lock = threading.Lock()

app = Flask(__name__)

# --- RUNTIME ACTUATION PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# --- CONTROL DESIGN CONSTANTS (8-BIT EXECUTION LAYER) ---
SERVO_CENTER_ANGLE = 100       # Absolute mechanical steering straight alignment midpoint
ROBOT_CRUISE_SPEED = 180      # Operational forward driving speed sent to ESP32 (0-255)
ROBOT_MANEUVER_SPEED = 155    # TUNED (was 155) -- backward docking speed, ported from corner_detection_debug.py

# --- NEW: INDEPENDENT VISION CALIBRATION PARAMETERS ---
STEERING_GAIN_GREEN = 0.1     # Baseline multiplier that keeps Green working perfectly
STEERING_GAIN_RED = 0.14      # INCREASE THIS to make the steering more aggressive for Red
RED_CLEARANCE_OFFSET = 0      # Static angular nudge (in degrees) to push the chassis wider right

# NEW: Obstacle-avoidance exit debounce. Once avoidance latches on, this many
# CONSECUTIVE frames with no obstacle detected are required before the robot
# is allowed to release back to wall-align. This is frame-count based (not
# time-based) -- tune it against your actual measured loop FPS (see the
# on-screen FPS readout) so it corresponds to roughly 0.3-0.6s of confirmed
# clear view. Too low and glare/saturation blips will still cause premature
# exits; too high and the robot will coast past a genuinely-cleared obstacle
# on stale steering for too long.
OBSTACLE_MISS_EXIT_FRAMES = 10

# --- GYRO DRIFT MANAGEMENT ---
PERIODIC_YAW_RESET_INTERVAL_SEC = 5.0   # re-zero gyro this often during normal driving

CORNER_DETECTION_COOLDOWN_SEC = 10
CORNER_PIVOT_SPEED = 140              # TUNED (was 140), ported from corner_detection_debug.py
CORNER_PIVOT_SAFETY_TIMEOUT = 2.5
CORNER_BACKWARD_DURATION = 1.5        # TUNED (was 4)
CORNER_BACKWARD_TOF_TARGET_MM = 50.0  # TUNED (was 190.0): stop the backward post-pivot motion ~5cm off the rear wall
CORNER_BACKWARD_TOF_TOLERANCE_MM = 10.0
CORNER_BRAKE_DELAY = 0.20             # TUNED (was 0.25)
CORNER_CHECK_LOG_EVERY_N_FRAMES = 10
STATE_LOG_EVERY_N_FRAMES = 10         # throttle per-frame prints inside active corner sub-states

# --- CORNER SIGNATURE / LANE DETECTION PARAMETERS ---
CORNER_SIGNATURE_STOP_DELAY_SEC = 0.25          # brake pause when corner signature first detected
CORNER_SIGNATURE_FRONT_TRIGGER_MM = 1000.0      # TUNED (was 700.0) -- how far out a corner signature can fire
LANE_REFERENCE_ARC_THRESHOLD_MM = 600.0         # 60cm lane-1 cutoff
CORNER_APPROACH_TRIGGER_MM = 550.0              # TUNED (was 350.0) -- turn earlier so the pivot starts further from the wall and the robot lands mid-lane

# --- ARC-REVERSE CORNER PARAMETERS (RIGHT turn, lane-1 wide case only) ---
CORNER_ARC_STEER_OFFSET = 60             # degrees off-center while reversing (toward SERVO_HARD_LEFT side)
CORNER_ARC_PIVOT_SPEED = 140             # TUNED (was 120), reverse speed magnitude during the arc
CORNER_ARC_PIVOT_SAFETY_TIMEOUT = 2.0    # TUNED (was 4.0)
CORNER_ARC_CLEAR_VIEW_TOF_MM = 400.0     # exit arc early once rear ToF reads ~40cm, for a clear front view sooner

TURN_TARGET_RIGHT_DEGREES = 60.0    # TUNED (was 80.0 -> 60.0 -> 45.0); reduced to counter turn overshoot
TURN_TARGET_LEFT_DEGREES = 60.0     # TUNED (was 80.0 -> 60.0 -> 45.0); reduced to counter turn overshoot

# --- GYRO LANE RECENTERING (post-obstacle return to lane heading) ---
# Ported from obs_main_lab_detection_masked.py's RECENTER phase. After the robot
# steers around a pillar the heading has drifted off the lane; the gyro steers it
# back to the heading recorded when the pass started, so it re-enters the lane
# straight instead of drifting into a wall or meeting the next corner crooked.
GYRO_RECENTER_DURATION_SEC = 1.5   # max time to spend straightening after a pass
GYRO_RECENTER_TOL_DEG = 2.0        # heading counts as recovered within this band
GYRO_RECENTER_KP = 1.2             # servo degrees per degree of heading error
GYRO_RECENTER_MAX_OFFSET = 22      # clamp the recenter steer (deg from centre)
GYRO_RECENTER_SIGN = +1            # flip to -1 if it steers the wrong way on your IMU

# --- POST-MANEUVER REALIGN (drive straight briefly after an obstacle pass) ---
# Ported from obs_main_lab_detection_masked.py: REALIGN -> RECENTER -> IDLE. After
# the pillar clears, hold the wheel straight for a moment so the chassis settles
# out of the avoid-arc, THEN the gyro recenter steers back to the pre-avoid lane
# heading. Without this settle the recenter fights the residual sway of the turn.
REALIGN_DURATION_SEC = 1.0
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0
WALL_ALIGN_CREEP_SPEED = 140
WALL_ALIGN_SAFETY_TIMEOUT = 3.0
WALL_ALIGN_NO_WALL_TIMEOUT = 1.0

# Dedicated wall-loss threshold for the corner-approach phase specifically -- kept SEPARATE
# from WALL_LOSS_THRESHOLD_MM (used by normal driving) so tuning the corner behavior doesn't
# silently change normal wall-following sensitivity elsewhere in the file.
CORNER_APPROACH_WALL_LOSS_THRESHOLD_MM = 400.0   # TUNED, ported from corner_detection_debug.py

# --- CAMERA CONFIGURATION MATRIX ---
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
HSV_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   
HSV_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3  
LAB_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3   # 768 (LAB detection frame)
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
    "red":    {"h_min": 0,   "h_max": 10,  "s_min": 160, "s_max": 255, "v_min": 60,  "v_max": 255},
    "blue":   {"h_min": 90,  "h_max": 149, "s_min": 35,  "s_max": 255, "v_min": 50,  "v_max": 255},
    "green":  {"h_min": 55,  "h_max": 106, "s_min": 20,  "s_max": 208, "v_min": 30,  "v_max": 197},
    "orange": {"h_min": 5,   "h_max": 25,  "s_min": 197, "s_max": 255, "v_min": 100, "v_max": 255},
    "white":  {"h_min": 0,   "h_max": 15,  "s_min": 0,   "s_max": 60,  "v_min": 100, "v_max": 255},
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

# --- DRIVABLE-AREA GATING (TRACK MASK / SEGMENTATION) ---
# A pillar that is actually ON the track sits on the white floor, so there is
# white directly below its base. Coloured things off the track (spectators,
# walls, banners) have no white floor beneath them -> ignore them.
WHITE_FLOOR_BAND_PX = 24         # height of the band sampled below a pillar (LAB-frame px)
WHITE_FLOOR_MIN_RATIO = 0.30     # min fraction of that band that must be on the track
TRACK_MASK_CLOSE_PX = 25         # close gaps (pillars/lines) so the floor is one blob
TRACK_MIN_AREA_FRAC = 0.02       # ignore white components smaller than this frac of frame
TRACK_BASE_DILATE_PX = 12        # tolerance band around the track edge for a pillar base

LAB_MIN_CONTOUR_AREA = 900       # ignore tiny colour blobs
LAB_MIN_WIDTH = 30               # ignore thin blobs
LAB_ROI_TOP_FRAC = 0.35          # ignore blobs whose base is above 35% of frame height

# Pre-built kernel for the colour masks
MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)


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
    """True if the pillar is standing ON the drivable (white) track region."""
    x1, y1, x2, y2 = bbox
    h, w = track_mask.shape[:2]
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
                        continue  # off-track colour -> ignore
                    dets.append({
                        'class': cls,
                        'area': float(ar),
                        'bbox': bbox,
                        'width': w,
                        'cx': x + w // 2,
                    })
    return dets


def filter_blue_objects_lab(lab_frame):
    """Blue mask (LAB) for blue line lap counting."""
    return lab_mask(lab_frame, 'blue')


# ===================== DEBUG VISUALIZATION (rendering only) =====================
# Everything below is pure overlay/visualisation. It reads the SAME detections
# returned by detect_lab_pillars() and the SAME LiDAR scan_data the control
# loop uses, and never feeds anything back into control / PID / state-machine
# logic. Tuned to stay cheap on the Pi.

# --- Black wall boundary outlines (camera image) ---
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
    already simplified into clean polylines via approxPolyDP."""
    l_channel = lab_frame[:, :, 0]
    otsu_t, _ = cv2.threshold(l_channel, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cutoff = int(min(VIZ_WALL_L_CLAMP_HI, max(VIZ_WALL_L_CLAMP_LO, otsu_t)))
    _, dark = cv2.threshold(l_channel, cutoff, 255, cv2.THRESH_BINARY_INV)
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
    """Mirror the 'nearest' pick (lowest bbox base, area tie-break) for DISPLAY."""
    return max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None


def draw_obstacle_overlay(frame, dets, scale_x=1.0, scale_y=1.0):
    """Boxes + centre circles + fixed reference points + connection lines."""
    h, w = frame.shape[:2]
    gx, gy = int(VIZ_GREEN_ORIGIN_FRAC[0] * w), int(VIZ_GREEN_ORIGIN_FRAC[1] * h)
    rx, ry = int(VIZ_RED_ORIGIN_FRAC[0] * w), int(VIZ_RED_ORIGIN_FRAC[1] * h)

    def center(d):
        x1, y1, x2, y2 = d['bbox']
        return (int((x1 + x2) * 0.5 * scale_x), int((y1 + y2) * 0.5 * scale_y))

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

    g = _primary_obstacle([d for d in dets if d['class'] == 'green'])
    if g is not None:
        cv2.line(frame, (gx, gy), center(g), (0, 255, 0), 2, cv2.LINE_AA)
    r = _primary_obstacle([d for d in dets if d['class'] == 'red'])
    if r is not None:
        cv2.line(frame, (rx, ry), center(r), (0, 0, 255), 2, cv2.LINE_AA)

    for (px, py), clr in (((gx, gy), (0, 255, 0)), ((rx, ry), (0, 0, 255))):
        cv2.circle(frame, (px, py), 7, clr, -1)
        cv2.circle(frame, (px, py), 9, (255, 255, 255), 2)


def draw_segmentation_overlay(processed_frame, lab_frame, detections, wall_contours):
    """Draw the track-wall outlines + track-gated pillar boxes on the display frame.

    Read-only: scales the LAB-frame contours/boxes up to the display frame."""
    if lab_frame is None:
        return
    scale_x = processed_frame.shape[1] / lab_frame.shape[1]
    scale_y = processed_frame.shape[0] / lab_frame.shape[0]
    draw_wall_contours(processed_frame, wall_contours, scale_x, scale_y)
    draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)


def render_lidar_map(scan_data, clockwise=True):
    """Top-down map of the current scan, with the wall-following points in blue."""
    size = VIZ_LIDAR_MAP_SIZE
    img = np.zeros((size, size, 3), np.uint8)
    cx = cy = size // 2
    scale = (size * 0.45) / VIZ_LIDAR_MAX_RANGE_MM

    for ring_mm in (1000, 2000, 3000):
        cv2.circle(img, (cx, cy), int(ring_mm * scale), (45, 45, 45), 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy), (cx, cy - int(size * 0.45)), (60, 60, 60), 1)

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

    cv2.circle(img, (cx, cy), 4, (255, 255, 255), -1)
    cv2.putText(img, "LiDAR wall-following pts", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, VIZ_LIDAR_WALL_COLOR, 1, cv2.LINE_AA)
    cv2.putText(img, "FRONT", (cx - 22, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA)
    return img

# --- DEBUG MATRIX CONFIGURATION ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

# --- ROBOT LOGIC STATES ---
class RobotState:
    INITIALIZING = "INITIALIZING"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    VISION_OBSTACLE_AVOIDANCE = "VISION_OBSTACLE_AVOIDANCE"
    LIDAR_SIDE_AVOIDANCE = "LIDAR_SIDE_AVOIDANCE"
    LAP_TERMINATION = "LAP_TERMINATION"
    STOP = "STOP"
    WALL_ALIGN_CORRECTION = "WALL_ALIGN_CORRECTION"
    CORNER_PRE_ALIGN = "CORNER_PRE_ALIGN"          # NEW: straighten against wall before approach
    CORNER_APPROACH_WALL = "CORNER_APPROACH_WALL"
    CORNER_ACTIVE_PIVOT = "CORNER_ACTIVE_PIVOT"
    CORNER_ARC_ACTIVE_PIVOT = "CORNER_ARC_ACTIVE_PIVOT"
    CORNER_ALIGN_BACKWARD = "CORNER_ALIGN_BACKWARD"
    CORNER_POST_PIVOT_ALIGN = "CORNER_POST_PIVOT_ALIGN"
current_robot_state = RobotState.INITIALIZING
current_yaw = 0.0

# --- PACKET SYSTEM TRANSMISSION WRAPPERS ---
def send_esp_packet(ser_port, steering, speed):
    """Encapsulates control variables into standard serial strings safely."""
    if ser_port and ser_port.is_open and not global_shutdown_event.is_set():
        try:
            packet = f"STR:{steering},SPD:{speed}\n"
            ser_port.write(packet.encode('utf-8'))
        except Exception:
            pass

# ====================================================
# CRITICAL SIGNAL INTERCEPT HANDLER (Graceful Braking)
# ====================================================
def emergency_shutdown_handler(signum, frame):
    """Intercepts terminal signals immediately to safely kill mechanical movement."""
    print("\n\n[EMERGENCY BRAKE] Shutdown signal captured! Halting hardware registers...")
    global_shutdown_event.set()
    camera_thread_stop_event.set()
    
    global esp_ser, lidar_scanner, picam2
    
    if esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
            print("[CLEANUP] Safety stop dispatched. Serial interface closed securely.")
        except Exception as e:
            print(f"[CLEANUP ERROR] Failed to flush serial stop command: {e}")
            
    if lidar_scanner:
        try:
            lidar_scanner.disconnect()
            print("[CLEANUP] LiDAR scanner safely disconnected.")
        except Exception as e:
            print(f"[CLEANUP ERROR] Failed to kill lidar spin: {e}")
            
    if picam2:
        try:
            picam2.stop()
            print("[CLEANUP] Picamera2 resource array unmounted.")
        except:
            pass
            
    print("[SUCCESS] All mechanical systems isolated. Exiting clean.\n")
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_shutdown_handler)   # Intercepts Ctrl+C
signal.signal(signal.SIGQUIT, emergency_shutdown_handler)  # Intercepts Ctrl+\

# --- COLOR PROCESSING MASKS ---
def filter_blue_objects(hsv_frame):
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    return cv2.dilate(mask, kernel, iterations=2)

def detect_color_binary(mask, threshold=4000):
    return cv2.countNonZero(mask) > threshold

def get_compensated_front_distance(scan_data, current_yaw):
    """Dynamically shifts the scanning cone window and applies cosine projection math."""
    if not scan_data:
        return 2000.0

    yaw_offset = int(round(current_yaw))
    dynamic_angles = range(-FRONT_SCAN_ANGLE_DEG + yaw_offset, FRONT_SCAN_ANGLE_DEG + yaw_offset + 1)

    compensated_points = []
    yaw_rad = np.radians(current_yaw)

    for a in dynamic_angles:
        if a in scan_data and scan_data[a] > 0:
            raw_distance = scan_data[a]
            true_distance = raw_distance * np.cos(yaw_rad)
            compensated_points.append(true_distance)

    if not compensated_points:
        return 2000.0

    return sum(compensated_points) / len(compensated_points)

# --- LIDAR DATA ACQUISITION BACKGROUND TASK ---
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("[SYSTEM] LiDAR background ingestion thread active.")
    try:
        while not global_shutdown_event.is_set():
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data.copy()
            time.sleep(0.01)
    except Exception as e:
        if not global_shutdown_event.is_set():
            print(f"[CRITICAL] LiDAR thread collapsed: {e}")
# --- TOF DATA ACQUISITION BACKGROUND TASK ---
def tof_acquisition_thread_func(sensor_instance):
    global latest_tof_distance_mm, tof_data_lock
    print("[SYSTEM] ToF background ingestion thread active.")
    tof_log_counter = 0
    while not global_shutdown_event.is_set():
        try:
            dist = sensor_instance.range
            with tof_data_lock:
                latest_tof_distance_mm = dist
            if tof_log_counter % 20 == 0:
                print(f"[TOF RAW] range={dist}mm")
        except Exception as e:
            with tof_data_lock:
                latest_tof_distance_mm = None
            if tof_log_counter % 20 == 0:
                print(f"[TOF RAW] read failed: {e}")
        tof_log_counter += 1
        time.sleep(0.03)

# --- CAMERA ACQUISITION BACKGROUND TASK ---
# OPTIMIZATION: derive the HSV source from the already-downscaled processing_frame_rgb
# instead of re-resizing the full-resolution captured frame a second time. This removes
# one full-res cv2.resize() call per loop iteration.
def camera_acquisition_thread_func(picam2_instance, stop_event, processing_size, hsv_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("[SYSTEM] Camera thread active. Processing dual-resize frame array.")
    try:
        while not stop_event.is_set() and not global_shutdown_event.is_set():
            captured_frame_rgb = picam2_instance.capture_array()

            processing_frame_rgb = cv2.resize(captured_frame_rgb, processing_size, interpolation=cv2.INTER_AREA)
            frame_bgr = cv2.cvtColor(processing_frame_rgb, cv2.COLOR_RGB2BGR)

            # Downscale further FROM the already-shrunk processing frame, not the raw capture.
            hsv_source_frame = cv2.resize(processing_frame_rgb, hsv_processing_size, interpolation=cv2.INTER_AREA)
            hsv_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_RGB2HSV)

            # LAB frame (same budget as the HSV frame) for the track mask /
            # drivable-area segmentation and the colour segment streams.
            lab_frame = cv2.cvtColor(hsv_source_frame, cv2.COLOR_RGB2LAB)

            with camera_frame_lock:
                latest_processed_frames['rgb'] = processing_frame_rgb
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['hsv'] = hsv_frame
                latest_processed_frames['lab'] = lab_frame

            # Track-gated pillar detection + wall extraction run HERE (camera thread) so the
            # control loop never spends time on segmentation. Results feed the overlay and the
            # segmented web streams only -- they never steer the robot.
            with vision_overlay_lock:
                latest_detections = detect_lab_pillars(lab_frame)
                latest_wall_contours = extract_wall_contours(lab_frame)
                latest_lab_shape = lab_frame

            # Small yield so this thread doesn't starve the control loop / Flask thread.
            time.sleep(0.001)
    except Exception as e:
        if not global_shutdown_event.is_set():
            print(f"[CRITICAL] Camera acquisition thread crashed: {e}")

# --- MAIN ROBOT NAVIGATION EXECUTION ENGINE ---
def check_imminent_right_side(scan_data):
    """True when any right-side ray in the imminent cone is closer than the urgent wall threshold."""
    if not scan_data:
        return False
    for angle, distance in scan_data.items():
        if IMMINENT_RIGHT_SIDE_MIN_ANGLE <= angle <= IMMINENT_RIGHT_SIDE_MAX_ANGLE and 0 < distance < IMMINENT_RIGHT_SIDE_DISTANCE_MM:
            return True
    return False


def check_imminent_left_side(scan_data):
    """True when any left-side ray in the imminent cone is closer than the urgent wall threshold."""
    if not scan_data:
        return False
    for angle, distance in scan_data.items():
        if IMMINENT_LEFT_SIDE_MIN_ANGLE <= angle <= IMMINENT_LEFT_SIDE_MAX_ANGLE and 0 < distance < IMMINENT_LEFT_SIDE_DISTANCE_MM:
            return True
    return False


def _gyro_recenter_angle(current_yaw, target_yaw):
    """Servo angle that steers the robot back toward its pre-avoidance heading.

    error > 0 means we still need to rotate back toward target_yaw. The sign
    constant maps that to the correct steering direction for the IMU in use."""
    err = target_yaw - current_yaw
    offset = GYRO_RECENTER_SIGN * GYRO_RECENTER_KP * err
    offset = max(-GYRO_RECENTER_MAX_OFFSET, min(GYRO_RECENTER_MAX_OFFSET, offset))
    return SERVO_CENTER_ANGLE + offset


def robot_control_loop():
    global output_frame, output_frame_lock, current_robot_state, latest_processed_frames, camera_frame_lock
    global CLOCKWISE_WALL_FOLLOWING, current_yaw, esp_ser, lidar_scanner, picam2

    # Initialize Hardware Serial Bus Connection
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[INFO] High-speed serial connection established with ESP32 execution layer.")
    except Exception as e:
        print(f"[FATAL] Serial bridge initialization failed on {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # ====================================================
    # NEW: STARTUP GYRO RESET
    # Zero the gyro right away, before anything else spins up, so current_yaw
    # starts from a known-good baseline of 0.0 rather than whatever the ESP32's
    # IMU happened to be reporting at power-on (residual drift, mounting offset,
    # or leftover state from a previous run).
    # ====================================================
    print("[INFO] Performing startup gyro reset...")
    try:
        esp_ser.write(b"RST_YAW\n")
        esp_ser.flush()
        time.sleep(0.2)
        # Drain any immediate YAW: lines so we don't act on stale pre-reset values.
        while esp_ser.in_waiting > 0:
            esp_ser.readline()
    except Exception as e:
        print(f"[WARN] Startup gyro reset failed to send: {e}")
    current_yaw = 0.0
    print("[INFO] Startup gyro reset complete. current_yaw = 0.0")

    # Initialize Camera Pipelines
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(1) 

    processing_size = (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    hsv_processing_size = (HSV_PROCESSING_WIDTH, HSV_PROCESSING_HEIGHT)

    camera_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event, processing_size, hsv_processing_size)
    )
    camera_thread.daemon = True
    camera_thread.start()

    # Initialize LiDAR Sensors
    try:
        lidar_scanner = LidarScanner(port='/dev/ttyUSB0', baudrate=230400)
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        print("[INFO] LiDAR scanner pipeline mounted safely.")
    except Exception as e:
        print(f"[WARN] LiDAR interface offline: {e}. Switching to vision fallback maps.")
        lidar_scanner = None
    # Initialize Rear ToF Distance Sensor
    global tof_sensor
    try:
        import board
        import busio
        import adafruit_vl53l0x
        i2c_tof = busio.I2C(board.SCL, board.SDA)
        tof_sensor = adafruit_vl53l0x.VL53L0X(i2c_tof)
        tof_thread = threading.Thread(target=tof_acquisition_thread_func, args=(tof_sensor,))
        tof_thread.daemon = True
        tof_thread.start()
        print("[INFO] Rear ToF sensor pipeline mounted safely.")
    except Exception as e:
        print(f"[WARN] ToF sensor offline: {e}. Backward phase will rely on timeout only.")
        tof_sensor = None

    # Initialize Controller Mathematics Loops
    # NOTE: standalone open-loop LiDAR wall-following (and its gyro-straight
    # fallback) has been REMOVED. gyro_straight_pid is only reset (never
    # updated) at corner-cycle boundaries now; it's kept around in case a
    # future fallback mode needs it, but it does no active steering work.
    # All default driving now runs through WALL_ALIGN_CORRECTION, which
    # judges straightness against the tracked wall's front/rear LiDAR
    # sectors (get_wall_parallel_error / get_wall_parallel_sector_stats)
    # via alignment_pid -- same approach used inside the corner state machine.
    gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
    wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)
    alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)

    # Set Initial Behavioral States
    # NOTE: PURE_GYRO_START and standalone LIDAR_WALL_FOLLOWING driving logic
    # have both been removed. The robot still starts labeled LIDAR_WALL_FOLLOWING
    # (used elsewhere as the "idle/default" state marker after corners and
    # obstacle avoidance), but Priority Level 5 immediately promotes it into
    # WALL_ALIGN_CORRECTION on the very first loop pass, which is now the sole
    # default driving behavior.
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
    turn_count = 0
    baseline_start_yaw = 0.0
    turn_direction = None
    use_reverse_arc = False
    locked_lane_reference_mm = None   # distance to tracked wall, locked at corner-detection instant
    lane_number = None                # 1 (arc-eligible) or "2/3" (pivot-only)
    pivot_phase_start_time = 0.0
    backward_phase_start_time = 0.0
    corner_cooldown_end_time = 0.0
    corner_check_frame_counter = 0
    state_log_frame_counter = 0       # throttles prints inside corner sub-states
    align_phase_start_time = 0.0
    align_return_state = None
    was_avoiding_obstacle = False
    align_no_wall_start_time = 0.0
    post_pivot_align_start_time = 0.0
    post_pivot_align_no_wall_start_time = 0.0
    pre_align_start_time = 0.0            # NEW: pre-turn wall-straighten phase timer
    pre_align_no_wall_start_time = 0.0    # NEW: pre-turn "no wall visible" sub-timer
    last_periodic_yaw_reset_time = time.monotonic()   # NEW: periodic drift-correction tracker

    # NEW: Obstacle avoidance latch / debounce registers.
    # Prevents a single dropped/misclassified vision frame (glare, saturation
    # clipping, brief occlusion) from immediately kicking the robot out of
    # avoidance and back into wall-align while the obstacle is still in front
    # of it. See PRIORITY LEVEL 4 below.
    obstacle_avoidance_active = False
    obstacle_miss_streak = 0
    last_avoid_servo_angle = SERVO_CENTER_ANGLE
    last_avoid_logic_label = None

    # NEW: Gyro lane-recentering registers (post-obstacle return to heading).
    # pre_avoid_yaw is the heading captured when a pass starts; once the pillar
    # clears, the loop gyro-steers back to it before resuming wall alignment --
    # ported from obs_main_lab_detection_masked.py.
    recenter_active = False
    pre_avoid_yaw = None
    recenter_end_time = 0.0

    # NEW: Post-maneuver realign registers (hold straight after an obstacle pass,
    # before the gyro recenter). Ported from obs_main_lab_detection_masked.py.
    realign_active = False
    realign_end_time = 0.0

    # Blue Line Crossing Telemetry Registers
    blue_count = 0
    prev_blue_state = False
    blue_cooldown_end_time = 0.0
    
    print(f"[SYSTEM] Calibration complete. Initial State: {current_robot_state}")


    # States during which current_yaw is actively load-bearing as a continuous
    # rotation/position reference (pivot targets, backward baselines, etc.) --
    # periodic drift-correction resets must NEVER fire during any of these.
    # NOTE: WALL_ALIGN_CORRECTION is deliberately NOT here -- it is the default
    # driving state, steers on LiDAR distance (not yaw), and must get the 5s
    # re-zero so the front-scan yaw compensation stays fresh for corner detection.
    CORNER_ACTIVE_STATES = [
        RobotState.CORNER_PRE_ALIGN, RobotState.CORNER_APPROACH_WALL,
        RobotState.CORNER_ACTIVE_PIVOT, RobotState.CORNER_ARC_ACTIVE_PIVOT,
        RobotState.CORNER_POST_PIVOT_ALIGN, RobotState.CORNER_ALIGN_BACKWARD,
    ]

    # Fallback display text so the overlay never shows a blank string on the
    # very first loop iteration before any branch has set one.
    display_text = "MODE: Initializing"

    try:
        while not global_shutdown_event.is_set():
            loop_start_time = time.monotonic()

            while esp_ser.in_waiting > 0:
                try:
                    raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line.startswith("YAW:"):
                        current_yaw = float(raw_line.split(":")[1])
                except Exception:
                    pass

            # ====================================================
            # PERIODIC GYRO RESET (every 5s during normal driving)
            # Nothing actively pins current_yaw back to a fresh 0 reference
            # (wall-alignment steering is LiDAR-distance based, not heading).
            # Left unchecked, that drift is exactly what
            # get_compensated_front_distance() uses to shift the front-scan cone
            # for the NEXT corner-signature check -- so stale yaw skews corner
            # detection. Re-zero unconditionally every interval, but ONLY outside
            # any state that depends on continuous yaw tracking: the corner
            # pivot/realign sequence, and active vision avoidance (resetting
            # mid-pass would destroy the heading reference the post-pass LANE
            # RECENTER needs).
            # ====================================================
            if (current_robot_state not in CORNER_ACTIVE_STATES
                and current_robot_state != RobotState.VISION_OBSTACLE_AVOIDANCE
                and not recenter_active
                and not realign_active
                and (time.monotonic() - last_periodic_yaw_reset_time) >= PERIODIC_YAW_RESET_INTERVAL_SEC):
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                current_yaw = 0.0
                last_periodic_yaw_reset_time = time.monotonic()
                print(f"[GYRO] Periodic drift-correction reset (state={current_robot_state}).")

            with camera_frame_lock:
                if not latest_processed_frames:
                    time.sleep(0.01)
                    continue
                frame_bgr = latest_processed_frames['bgr'].copy()
                hsv = latest_processed_frames['hsv'].copy()

            # --- TRACK-GATED LAB PILLAR DETECTION (overlay/display only) ---
            # Segmentation runs in the camera thread (see camera_acquisition_thread_func);
            # the control loop only picks up the finished results. These feed the overlay
            # and the segmented web streams, never the steering decision (which stays on
            # process_frame_for_steering below).
            detections = []
            wall_contours = []
            lab = None
            with vision_overlay_lock:
                if latest_lab_shape is not None:
                    lab = latest_lab_shape
                    detections = latest_detections
                    wall_contours = latest_wall_contours

            scan_data = {}
            if lidar_scanner:
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            avg_front_baseline = get_compensated_front_distance(scan_data, current_yaw)

            left_pts = [scan_data[a] for a in range(-105, -75) if a in scan_data and scan_data[a] > 0]
            right_pts = [scan_data[a] for a in range(75, 105) if a in scan_data and scan_data[a] > 0]
            avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
            avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
            in_cooldown = time.monotonic() < corner_cooldown_end_time

            if scan_data and corner_check_frame_counter % CORNER_CHECK_LOG_EVERY_N_FRAMES == 0:
                cooldown_note = f" | COOLDOWN ({corner_cooldown_end_time - time.monotonic():.1f}s left)" if in_cooldown else ""
                print(f"[{current_robot_state}] [CORNER CHECK] Front: {avg_front_baseline:.1f}mm | Left: {avg_left:.1f}mm | Right: {avg_right:.1f}mm | Yaw: {current_yaw:+.1f}°{cooldown_note}")

            corner_check_frame_counter += 1

            # ====================================================
            # NEW: ALWAYS RUN VISION OBSTACLE DETECTION FIRST
            # This guarantees the obstacle-detection overlay (ROI boxes, target
            # lines, obstacle bounding box/contour, depth factor text) is drawn
            # on every single frame, regardless of which state the machine is
            # in this iteration. Previously this call only lived inside the
            # Priority Level 5 branch below, so the overlay would vanish any
            # time the robot was cornering, wall-aligning, or side-avoiding --
            # that's the "disappearing/reappearing" UI you were seeing.
            # The resulting vision_angle / logic_label are still only ACTED ON
            # later, inside Priority Level 5, exactly as before -- this change
            # only affects what gets drawn/streamed, not the steering decision.
            # ====================================================
            is_near_field_mode = avg_front_baseline < 1100.0
            processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                frame_bgr, use_outer_roi_and_bottom_point=is_near_field_mode
            )
            vision_angle = -1 * vision_angle

            current_timestamp = time.time()
            blue_mask = filter_blue_objects(hsv)
            blue_in_view = detect_color_binary(blue_mask, threshold=4000)

            if not blue_in_view and prev_blue_state:
                if current_timestamp > blue_cooldown_end_time:
                    blue_count += 1
                    print(f"[RACE telemet] Blue line passed! Total lines crossed: {blue_count}/12")
                    blue_cooldown_end_time = current_timestamp + 5.0
            prev_blue_state = blue_in_view

            # ====================================================
            # PRIORITY LEVEL 0: LAP TERMINATION OVERRIDE
            # ====================================================
            if current_robot_state == RobotState.LAP_TERMINATION or turn_count >= 12:
                current_robot_state = RobotState.LAP_TERMINATION
                print("\n==========================================================")
                print(f"[MATCH COMPLETE] 12 Race Turns Logged! Locking wheels to finish...")
                print("==========================================================")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                time.sleep(4.0)
                
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                print("[SYSTEM] Hard race shutdown executed successfully.")
                break

            # ====================================================
            # PRIORITY LEVEL 1: ACTIVE CORNERING RUNTIME EXECUTION
            # (ported from the tuned corner_detection_debug.py state machine)
            # ====================================================
            if current_robot_state in [RobotState.CORNER_PRE_ALIGN, RobotState.CORNER_APPROACH_WALL, RobotState.CORNER_ACTIVE_PIVOT, RobotState.CORNER_ARC_ACTIVE_PIVOT, RobotState.CORNER_POST_PIVOT_ALIGN, RobotState.CORNER_ALIGN_BACKWARD]:

                state_log_frame_counter += 1
                should_log_state = (state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

                if current_robot_state == RobotState.CORNER_PRE_ALIGN:
                    # Straighten against the tracked wall FIRST, immediately after a corner
                    # is detected -- same parallel front/rear LiDAR-error PID logic as
                    # WALL_ALIGN_CORRECTION, run here as a dedicated squaring-up step BEFORE
                    # the approach phase begins.
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                    parallel_error = get_wall_parallel_error(scan_data, align_side)
                    elapsed_pre_align = time.monotonic() - pre_align_start_time

                    if parallel_error is None:
                        if pre_align_no_wall_start_time == 0.0:
                            pre_align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - pre_align_no_wall_start_time
                    else:
                        pre_align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    hard_timeout = elapsed_pre_align >= WALL_ALIGN_SAFETY_TIMEOUT
                    no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                    display_text = (
                        f"[CORNER] Phase 1.5: Pre-Turn Straighten | Side: {align_side.title()} | Err: {parallel_error:.1f}mm"
                        if parallel_error is not None else
                        f"[CORNER] Phase 1.5: Pre-Turn Straighten | Side: {align_side.title()} | Err: N/A"
                    )

                    if should_log_state:
                        print(
                            f"[{current_robot_state}] [PRE-TURN STRAIGHTEN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                            f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                            f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_pre_align:.2f}s"
                        )

                    if is_aligned or hard_timeout or no_wall_timeout:
                        reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                        print(f"[{current_robot_state}] [PRE-TURN STRAIGHTEN] Exit condition reached ({reason}). Proceeding to corner approach.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)
                        alignment_pid.reset()
                        current_robot_state = RobotState.CORNER_APPROACH_WALL
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                    else:
                        if parallel_error is None:
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = alignment_pid.update(normalized_error)
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            send_esp_packet(esp_ser, final_servo_angle, WALL_ALIGN_CREEP_SPEED)

                elif current_robot_state == RobotState.CORNER_APPROACH_WALL:
                    # Straightness fallback (side wall lost / no side-wall points at all) now
                    # uses the same front/rear parallel-wall alignment method as
                    # CORNER_PRE_ALIGN / WALL_ALIGN_CORRECTION, driven by alignment_pid,
                    # instead of gyro_straight_pid -- matches the tuned debug-harness version.
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

                    if avg_front_baseline < CORNER_APPROACH_TRIGGER_MM:
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Approach limit reached. Front {avg_front_baseline:.1f}mm < {CORNER_APPROACH_TRIGGER_MM:.0f}mm. Applying hard brake...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Resetting gyro registers before turn...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        # ================================================================
                        # FIX: previously current_yaw was NOT explicitly zeroed here -- only
                        # the ESP32's internal register was reset via the write() above.
                        # baseline_start_yaw was then set from current_yaw's STALE
                        # pre-reset value (whatever residual drift existed at this instant),
                        # not 0. Every other RST_YAW call site in this file follows
                        # write -> sleep -> current_yaw = 0.0; this was the one missing it,
                        # and it directly explains inconsistent pivot overshoot/undershoot --
                        # the entire yaw_delta_signed calculation for the whole pivot was
                        # offset by that leftover drift for its entire duration.
                        # ================================================================
                        current_yaw = 0.0

                        pivot_phase_start_time = time.monotonic()
                        baseline_start_yaw = current_yaw

                        if use_reverse_arc:
                            arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Entering reverse arc pivot (Lane 1): steer={arc_steer_angle}° speed=-{CORNER_ARC_PIVOT_SPEED}")
                            current_robot_state = RobotState.CORNER_ARC_ACTIVE_PIVOT
                            send_esp_packet(esp_ser, arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)
                        else:
                            final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Locking wheels to extreme pivot angle (Lane {lane_number}): {final_servo}°")
                            send_esp_packet(esp_ser, final_servo, 0)
                            time.sleep(0.15)
                            current_robot_state = RobotState.CORNER_ACTIVE_PIVOT
                            send_esp_packet(esp_ser, final_servo, CORNER_PIVOT_SPEED)
                    else:
                        wall_pts = left_pts if CLOCKWISE_WALL_FOLLOWING else right_pts
                        parallel_error = get_wall_parallel_error(scan_data, align_side)

                        if wall_pts:
                            avg_wall = sum(wall_pts) / len(wall_pts)
                            if avg_wall > CORNER_APPROACH_WALL_LOSS_THRESHOLD_MM:
                                # Side wall too far / effectively lost -> fall back to
                                # parallel-wall straightening instead of gyro-straight.
                                if parallel_error is None:
                                    approach_servo_angle = SERVO_CENTER_ANGLE
                                    display_text = f"[CORNER] Phase 2: Approach (Wall Lost, No Parallel Data) | Front: {avg_front_baseline:.0f}mm"
                                else:
                                    normalized_error = parallel_error if align_side == "left" else -parallel_error
                                    pid_output = alignment_pid.update(normalized_error)
                                    approach_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                    display_text = f"[CORNER] Phase 2: Approach (Wall Lost -> Parallel Straighten) | Front: {avg_front_baseline:.0f}mm | Err: {parallel_error:.0f}mm"
                            else:
                                wall_error = (avg_wall - WALL_FOLLOW_TARGET_MM) if CLOCKWISE_WALL_FOLLOWING else (WALL_FOLLOW_TARGET_MM - avg_wall)
                                pid_output = wall_follow_pid.update(wall_error)
                                approach_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                display_text = f"[CORNER] Phase 2: Approach (Wall Follow) | Front: {avg_front_baseline:.0f}mm | Err: {wall_error:.0f}mm"
                        else:
                            # No side-wall points at all -> parallel straighten if we have
                            # front/rear sector data, otherwise just hold center.
                            if parallel_error is None:
                                approach_servo_angle = SERVO_CENTER_ANGLE
                                display_text = f"[CORNER] Phase 2: Approach (No Wall Data) | Front: {avg_front_baseline:.0f}mm"
                            else:
                                normalized_error = parallel_error if align_side == "left" else -parallel_error
                                pid_output = alignment_pid.update(normalized_error)
                                approach_servo_angle = SERVO_CENTER_ANGLE - pid_output
                                display_text = f"[CORNER] Phase 2: Approach (No Side-Wall Pts -> Parallel Straighten) | Front: {avg_front_baseline:.0f}mm | Err: {parallel_error:.0f}mm"

                        final_approach_servo = int(round(np.clip(approach_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                        if should_log_state:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] {display_text} | servo={final_approach_servo} | Lane: {lane_number}")
                        send_esp_packet(esp_ser, final_approach_servo, ROBOT_CRUISE_SPEED)

                elif current_robot_state == RobotState.CORNER_ACTIVE_PIVOT:
                    elapsed_pivot = time.monotonic() - pivot_phase_start_time
                    yaw_delta_signed = current_yaw - baseline_start_yaw

                    if turn_direction == "RIGHT":
                        target_degrees = TURN_TARGET_RIGHT_DEGREES
                        pivot_complete = yaw_delta_signed <= -TURN_TARGET_RIGHT_DEGREES
                    else:
                        target_degrees = TURN_TARGET_LEFT_DEGREES
                        pivot_complete = yaw_delta_signed >= TURN_TARGET_LEFT_DEGREES

                    display_text = f"[CORNER] Phase 3: Gyro Pivot | Yaw: {yaw_delta_signed:+.1f}° / target {target_degrees}° ({turn_direction}) | Lane: {lane_number}"
                    pivot_timed_out = elapsed_pivot >= CORNER_PIVOT_SAFETY_TIMEOUT

                    if pivot_complete or pivot_timed_out:
                        if pivot_timed_out and not pivot_complete:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] WARNING: Pivot safety timeout hit before target yaw reached (only {yaw_delta_signed:+.1f}° / target {target_degrees}°). Yaw: {current_yaw:+.1f}°. Check gyro data.")
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Pivot complete ({yaw_delta_signed:+.1f}°). Braking...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Resetting chassis orientation...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        # ====================================================
                        # POST-MANEUVER DISPATCH (user request 2026-08-01)
                        # Backward motion during a corner is ONLY the ToF-terminated
                        # rear dock: reverse at maneuver speed until the rear ToF reads
                        # < 5cm (CORNER_BACKWARD_TOF_TARGET_MM), or the 1.5s safety
                        # timeout fires. There is deliberately NO backward-creep
                        # straightening phase anymore -- the robot docks against the
                        # corner wall, then WALL_ALIGN_CORRECTION below realigns it
                        # straight down the new lane using the lidar triangle method
                        # (front/rear parallel-error sectors). Applies to EVERY lane.
                        # ====================================================
                        print(f"[{current_robot_state}] [CORNER EXECUTION] ToF dock (reverse until rear ToF < {CORNER_BACKWARD_TOF_TARGET_MM:.0f}mm) -- Lane {lane_number}.")
                        backward_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                    else:
                        final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                        send_esp_packet(esp_ser, final_servo, CORNER_PIVOT_SPEED)
                        if should_log_state:
                            print(f"[{current_robot_state}] [PIVOT] Yaw: {yaw_delta_signed:+.1f}° / target {target_degrees}° | Elapsed: {elapsed_pivot:.2f}s")

                elif current_robot_state == RobotState.CORNER_ARC_ACTIVE_PIVOT:
                    elapsed_arc = time.monotonic() - pivot_phase_start_time
                    yaw_delta_signed = current_yaw - baseline_start_yaw
                    arc_complete = yaw_delta_signed <= -TURN_TARGET_RIGHT_DEGREES
                    arc_timed_out = elapsed_arc >= CORNER_ARC_PIVOT_SAFETY_TIMEOUT

                    with tof_data_lock:
                        rear_distance = latest_tof_distance_mm

                    # Exit the arc EARLY once the rear ToF sees ~40cm clearance, so the front
                    # sensor gets a clear view of the new lane sooner (final docking distance
                    # is handled later, in CORNER_ALIGN_BACKWARD, as a separate step).
                    rear_tof_clear_view_hit = (
                        rear_distance is not None
                        and rear_distance <= CORNER_ARC_CLEAR_VIEW_TOF_MM
                    )

                    display_text = f"[CORNER] Arc Reverse Pivot | Yaw: {yaw_delta_signed:+.1f}° / target -{TURN_TARGET_RIGHT_DEGREES}° | Rear: {rear_distance if rear_distance is not None else float('nan'):.0f}mm"
                    if should_log_state:
                        print(f"[{current_robot_state}] [ARC PIVOT] Yaw: {yaw_delta_signed:+.1f}° | Rear: {rear_distance} | Elapsed: {elapsed_arc:.2f}s")

                    if arc_complete or arc_timed_out or rear_tof_clear_view_hit:
                        if arc_timed_out and not arc_complete:
                            print(f"[{current_robot_state}] [ARC PIVOT] WARNING: Arc safety timeout hit before target yaw reached (only {yaw_delta_signed:+.1f}°). Check gyro data.")
                        if rear_tof_clear_view_hit:
                            print(f"[{current_robot_state}] [ARC PIVOT] Rear ToF clear-view distance reached ({rear_distance:.0f}mm <= {CORNER_ARC_CLEAR_VIEW_TOF_MM:.0f}mm). Ending arc early for forward visibility.")
                        print(f"[{current_robot_state}] [ARC PIVOT] Arc complete ({yaw_delta_signed:+.1f}°). Hard braking chassis...")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [ARC PIVOT] Resetting chassis orientation before post-maneuver ToF dock...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        backward_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                    else:
                        arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                        send_esp_packet(esp_ser, arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)

                elif current_robot_state == RobotState.CORNER_POST_PIVOT_ALIGN:
                    # NOTE: Currently BYPASSED by the dispatch above -- backward motion
                    # during corners is now reserved for the ToF-terminated dock only
                    # (CORNER_ALIGN_BACKWARD). Kept as a reachable fallback handler.
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                    parallel_error = get_wall_parallel_error(scan_data, align_side)
                    elapsed_post_pivot_align = time.monotonic() - post_pivot_align_start_time

                    if parallel_error is None:
                        if post_pivot_align_no_wall_start_time == 0.0:
                            post_pivot_align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - post_pivot_align_no_wall_start_time
                    else:
                        post_pivot_align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    hard_timeout = elapsed_post_pivot_align >= WALL_ALIGN_SAFETY_TIMEOUT
                    no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                    display_text = (
                        f"[CORNER] Phase 3.5: Post-Pivot Align | Side: {align_side.title()} | Err: {parallel_error:.1f}mm"
                        if parallel_error is not None else
                        f"[CORNER] Phase 3.5: Post-Pivot Align | Side: {align_side.title()} | Err: N/A"
                    )

                    if should_log_state:
                        print(
                            f"[{current_robot_state}] [POST-PIVOT ALIGN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                            f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                            f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_post_pivot_align:.2f}s"
                        )

                    if is_aligned or hard_timeout or no_wall_timeout:
                        reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                        print(f"[{current_robot_state}] [POST-PIVOT ALIGN] Exit ({reason}). Proceeding to rear-ToF docking phase.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)
                        alignment_pid.reset()
                        backward_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                    else:
                        if parallel_error is None:
                            send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -WALL_ALIGN_CREEP_SPEED)
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = -alignment_pid.update(normalized_error)
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            send_esp_packet(esp_ser, final_servo_angle, -WALL_ALIGN_CREEP_SPEED)

                elif current_robot_state == RobotState.CORNER_ALIGN_BACKWARD:
                    # The ONLY backward motion during a corner maneuver (user request):
                    # reverse at maneuver speed until the rear ToF docks <5cm, then the
                    # cleanup below hands off to WALL_ALIGN_CORRECTION, which realigns
                    # straight down the new lane using the lidar triangle method.
                    elapsed_back = time.monotonic() - backward_phase_start_time

                    with tof_data_lock:
                        rear_distance = latest_tof_distance_mm

                    tof_reached = (
                        rear_distance is not None
                        and rear_distance <= (CORNER_BACKWARD_TOF_TARGET_MM + CORNER_BACKWARD_TOF_TOLERANCE_MM)
                    )
                    hard_timeout_backward = elapsed_back >= CORNER_BACKWARD_DURATION

                    display_text = (
                        f"[CORNER] Phase 4: Backing Away | Rear: {rear_distance:.0f}mm / {CORNER_BACKWARD_TOF_TARGET_MM:.0f}mm"
                        if rear_distance is not None else
                        f"[CORNER] Phase 4: Backing Away | Rear: N/A | {elapsed_back:.1f}s / {CORNER_BACKWARD_DURATION}s"
                    )
                    if should_log_state:
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Phase 4: Reversing | Rear: {rear_distance} | Elapsed: {elapsed_back:.2f}s | Yaw: {current_yaw:+.1f}°")

                    if tof_reached or hard_timeout_backward:
                        if hard_timeout_backward and not tof_reached:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] WARNING: Backward safety timeout hit before ToF target reached (rear={rear_distance}). Check ToF sensor.")
                        else:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] ToF target reached (rear={rear_distance:.0f}mm). Hard braking applied.")
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                        time.sleep(0.3)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Cleaning up spatial registers before resuming wall-follow...")
                        esp_ser.write(b"RST_YAW\n")
                        esp_ser.flush()
                        time.sleep(0.1)
                        current_yaw = 0.0

                        turn_count += 1
                        gyro_straight_pid.reset()
                        wall_follow_pid.reset()
                        alignment_pid.reset()
                        corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC

                        align_return_state = RobotState.LIDAR_WALL_FOLLOWING
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        current_robot_state = RobotState.WALL_ALIGN_CORRECTION
                        turn_direction = None
                        locked_lane_reference_mm = None
                        lane_number = None
                    else:
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)

                if DEBUG_UI_OVERLAYS:
                    loop_duration = time.monotonic() - loop_start_time
                    fps = 1.0 / loop_duration if loop_duration > 0 else 0
                    draw_segmentation_overlay(processed_frame, lab, detections, wall_contours)
                    cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    if STREAM_VIDEO:
                        with output_frame_lock:
                            output_frame = processed_frame.copy()

                time.sleep(0.02)
                continue

            # ====================================================
            # PRIORITY LEVEL 2: RADAR FRONTAL TRIPWIRE CHECK
            # ====================================================
            is_corner_signature = False
            if (not in_cooldown
                and avg_front_baseline <= CORNER_SIGNATURE_FRONT_TRIGGER_MM
                and ((avg_left < 950.0 and avg_right > 1600.0)
                     or (avg_right < 900.0 and avg_left > 1800.0))):
                is_corner_signature = True

            if is_corner_signature:
                print(f"\n[CORNER INTERSECTION] Front Wall at: {avg_front_baseline:.1f}mm. Stopping chassis... (signature)")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0)
                time.sleep(CORNER_SIGNATURE_STOP_DELAY_SEC)
                wall_follow_pid.reset()

                if avg_left < avg_right:
                    turn_direction = "RIGHT"
                    if turn_count == 0:
                        CLOCKWISE_WALL_FOLLOWING = True
                        print("[LAYOUT LOCKDOWN] Track direction set to: CLOCKWISE (CW)")
                else:
                    turn_direction = "LEFT"
                    if turn_count == 0:
                        CLOCKWISE_WALL_FOLLOWING = False
                        print("[LAYOUT LOCKDOWN] Track direction set to: COUNTER-CLOCKWISE (CCW)")

                # ====================================================
                # LANE DETECTION: lock the tracked-wall distance at the
                # exact instant the corner is detected. CW course tracks
                # the LEFT wall, CCW tracks the RIGHT wall.
                # ====================================================
                locked_lane_reference_mm = avg_left if CLOCKWISE_WALL_FOLLOWING else avg_right

                if locked_lane_reference_mm > LANE_REFERENCE_ARC_THRESHOLD_MM:
                    lane_number = 1
                    use_reverse_arc = (turn_direction == "RIGHT")
                    if turn_direction != "RIGHT":
                        print("[LANE DETECT] Lane 1 detected on a LEFT turn -- arc geometry not "
                              "tuned for this side yet, falling back to pivot turn.")
                else:
                    lane_number = "2/3"
                    use_reverse_arc = False

                print(f"[LANE DETECT] Locked {'left' if CLOCKWISE_WALL_FOLLOWING else 'right'} wall dist: "
                      f"{locked_lane_reference_mm:.0f}mm (threshold {LANE_REFERENCE_ARC_THRESHOLD_MM:.0f}mm) -> "
                      f"Lane {lane_number} -> {'ARC turn' if use_reverse_arc else 'PIVOT turn'}")

                print(f"[PRE-TURN ACTUATION] Entering pre-turn wall straighten before {turn_direction} turn approach...")
                print("[ACTION] Flushing scrub vibration error -> Resetting Gyro Yaw to 0°...")
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                time.sleep(0.1)
                
                current_yaw = 0.0
                baseline_start_yaw = 0.0
                pre_align_start_time = time.monotonic()
                pre_align_no_wall_start_time = 0.0
                current_robot_state = RobotState.CORNER_PRE_ALIGN
                
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                continue

            # ====================================================
            # PRIORITY LEVEL 3: PROXIMITY CRITICAL WALL OVERRIDES
            # ====================================================
            side_alert = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, safety_distance_mm=150, clockwise=CLOCKWISE_WALL_FOLLOWING)
            right_side_panic = [scan_data[a] for a in range(35, 80) if a in scan_data and 0 < scan_data[a] < LIDAR_RIGHT_SIDE_DISTANCE_MM]
            left_side_panic = [scan_data[a] for a in range(-79, -34) if a in scan_data and 0 < scan_data[a] < LIDAR_LEFT_SIDE_DISTANCE_MM]

            # NEW: Imminent side-collision tier. Tighter cone (120mm) than the normal
            # 18cm panic band, with a stronger steer magnitude, so a wall brushing the
            # chassis is yanked away BEFORE it reaches the close panic threshold.
            right_imminent = check_imminent_right_side(scan_data)
            left_imminent = check_imminent_left_side(scan_data)

            if right_imminent:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - IMMINENT_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: IMMINENT | Right Side Collision"
            elif left_imminent:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + IMMINENT_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: IMMINENT | Left Side Collision"
            elif right_side_panic:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Right Wall Close"
            elif left_side_panic:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = "MODE: OVERRIDE | Left Wall Close"

            # ====================================================
            # PRIORITY LEVEL 4: COMPUTER VISION PILLAR AVOIDANCE
            # (vision_angle / logic_label were already computed at the top
            # of this loop iteration, so the same detection that feeds the
            # persistent overlay is used here for the steering decision.)
            # ====================================================
            else:
                obstacle_detected_this_frame = logic_label in ["red_obstacle", "obstacle"]

                if obstacle_detected_this_frame:
                    # Fresh confirmed detection -- (re)latch avoidance and reset the miss streak.
                    if not obstacle_avoidance_active:
                        # Heading before this pass starts, for the post-pass LANE RECENTER.
                        realign_active = False
                        recenter_active = False
                        pre_avoid_yaw = current_yaw
                    obstacle_avoidance_active = True
                    obstacle_miss_streak = 0
                    was_avoiding_obstacle = True
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    last_avoid_logic_label = logic_label

                    # MODIFICATION: Split Red vs Green processing blocks to handle asymmetric steering weights
                    if logic_label == "red_obstacle":
                        servo_adjust = -vision_angle * STEERING_GAIN_RED
                        # Apply the steering scale along with a positive clearance bias to widen right-hand arcs
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust + RED_CLEARANCE_OFFSET
                        display_text = f"MODE: Red Avoid | Steer: {int(target_servo_angle)}°"
                    else:
                        # Standard Green avoidance loop handles left-hand transitions cleanly
                        servo_adjust = -vision_angle * STEERING_GAIN_GREEN
                        target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                        display_text = f"MODE: Green Avoid | Steer: {int(target_servo_angle)}°"

                    last_avoid_servo_angle = target_servo_angle

                elif obstacle_avoidance_active and obstacle_miss_streak < OBSTACLE_MISS_EXIT_FRAMES:
                    # Vision lost the obstacle THIS frame (glare, saturation clip, brief
                    # occlusion, or it's now too close/off-ROI to classify) -- do NOT
                    # snap back to wall-align on a single missed frame. Keep coasting on
                    # the LAST confirmed steering command/direction until either detection
                    # resumes (branch above) or enough consecutive misses accumulate to
                    # conclude the obstacle has genuinely been cleared.
                    obstacle_miss_streak += 1
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    target_servo_angle = last_avoid_servo_angle
                    display_text = f"MODE: Avoid Coast ({last_avoid_logic_label}) | Miss {obstacle_miss_streak}/{OBSTACLE_MISS_EXIT_FRAMES}"

                else:
                    # Either never avoiding, or the miss streak has run out -- obstacle is
                    # considered genuinely cleared. Release the latch and fall through to
                    # default driving (post-maneuver wall alignment).
                    obstacle_avoidance_active = False
                    obstacle_miss_streak = 0

                    # ====================================================
                    # LANE RECENTERING (gyro return-to-heading after the pass)
                    # Ported from obs_main_lab_detection_masked.py: the pass has
                    # left the heading off the lane. Steer back toward the heading
                    # captured when the pass started, then hand back to default
                    # wall alignment below.
                    # ====================================================
                    if realign_active:
                        if time.monotonic() >= realign_end_time:
                            # Straight-settle done -> hand off to the gyro recenter.
                            print("[POST-MANEUVER REALIGN] Settle done -- recentering onto lane heading.")
                            realign_active = False
                            if pre_avoid_yaw is not None:
                                recenter_active = True
                                recenter_end_time = time.monotonic() + GYRO_RECENTER_DURATION_SEC
                            else:
                                pre_avoid_yaw = None
                        else:
                            # Hold the wheel straight while the chassis settles out of
                            # the avoid-arc; the RECENTER phase above then steers back
                            # to the pre-avoid lane heading. Ported from
                            # obs_main_lab_detection_masked.py's REALIGN phase.
                            current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                            robot_speed_current = ROBOT_MANEUVER_SPEED
                            target_servo_angle = SERVO_CENTER_ANGLE
                            display_text = "MODE: Post-Maneuver Realign (holding straight)"

                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

                            if DEBUG_UI_OVERLAYS:
                                loop_duration = time.monotonic() - loop_start_time
                                fps = 1.0 / loop_duration if loop_duration > 0 else 0
                                draw_segmentation_overlay(processed_frame, lab, detections, wall_contours)
                                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                                cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                                cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                                if STREAM_VIDEO:
                                    with output_frame_lock:
                                        output_frame = processed_frame.copy()

                            time.sleep(0.02)
                            continue
                    if recenter_active:
                        if (pre_avoid_yaw is None
                                or time.monotonic() >= recenter_end_time
                                or abs(pre_avoid_yaw - current_yaw) <= GYRO_RECENTER_TOL_DEG):
                            print(f"[LANE RECENTER] Heading recovered ({pre_avoid_yaw - current_yaw if pre_avoid_yaw is not None else 0.0:+.1f}° err) -- returning to wall alignment.")
                            recenter_active = False
                            pre_avoid_yaw = None
                        else:
                            current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                            robot_speed_current = ROBOT_MANEUVER_SPEED
                            target_servo_angle = _gyro_recenter_angle(current_yaw, pre_avoid_yaw)
                            display_text = f"MODE: Lane Recenter | Err: {pre_avoid_yaw - current_yaw:+.1f}°"

                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

                            if DEBUG_UI_OVERLAYS:
                                loop_duration = time.monotonic() - loop_start_time
                                fps = 1.0 / loop_duration if loop_duration > 0 else 0
                                draw_segmentation_overlay(processed_frame, lab, detections, wall_contours)
                                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                                cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                                cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                                if STREAM_VIDEO:
                                    with output_frame_lock:
                                        output_frame = processed_frame.copy()

                            time.sleep(0.02)
                            continue
                    elif pre_avoid_yaw is not None:
                        # Pass just completed -- kick off the REALIGN settle phase first
                        # (hold straight), THEN the gyro recenter above takes over once
                        # the chassis has stopped swaying out of the avoid-arc.
                        realign_active = True
                        realign_end_time = time.monotonic() + REALIGN_DURATION_SEC
                        print(f"[POST-MANEUVER REALIGN] Pass complete ({pre_avoid_yaw - current_yaw:+.1f}° drift) -- holding straight before recentering...")

                    # ====================================================
                    # PRIORITY LEVEL 5: POST-MANEUVER WALL ALIGNMENT
                    # (this now ALSO doubles as the default driving mode --
                    # standalone open-loop LiDAR wall-following has been removed.
                    # Any time nothing higher-priority claims the loop, the robot
                    # continuously re-enters this parallel-wall alignment behavior
                    # to track the wall, returning to itself when "aligned".)
                    # ====================================================
                    if was_avoiding_obstacle:
                        was_avoiding_obstacle = False

                    if current_robot_state != RobotState.WALL_ALIGN_CORRECTION:
                        current_robot_state = RobotState.WALL_ALIGN_CORRECTION
                        align_return_state = RobotState.LIDAR_WALL_FOLLOWING
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        alignment_pid.reset()

                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                    parallel_error = get_wall_parallel_error(scan_data, align_side)
                    elapsed_align = time.monotonic() - align_phase_start_time

                    if parallel_error is None:
                        if align_no_wall_start_time == 0.0:
                            align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - align_no_wall_start_time
                    else:
                        align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    hard_timeout = elapsed_align >= WALL_ALIGN_SAFETY_TIMEOUT
                    no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                    display_text = (
                        f"MODE: Wall Align | Side: {align_side.title()} | Err: {parallel_error:.1f}mm"
                        if parallel_error is not None else
                        f"MODE: Wall Align | Side: {align_side.title()} | Err: N/A"
                    )

                    state_log_frame_counter += 1
                    if state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0:
                        print(
                            f"[{current_robot_state}] [WALL ALIGN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                            f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                            f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_align:.2f}s"
                        )

                    if is_aligned or hard_timeout or no_wall_timeout:
                        # "Aligned" no longer exits back to a separate default state -- it just
                        # resets the phase timer and keeps tracking, since this state IS the
                        # default now. Timeout / no_wall reasons are logged for visibility.
                        # Falls through below to the single shared steer/speed send.
                        reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                        if state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0:
                            print(f"[{current_robot_state}] [WALL ALIGN] Re-centering cycle ({reason}).")
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        last_periodic_yaw_reset_time = time.monotonic()

                    if parallel_error is None:
                        send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                    else:
                        normalized_error = parallel_error if align_side == "left" else -parallel_error
                        pid_output = alignment_pid.update(normalized_error)
                        target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                        final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                        send_esp_packet(esp_ser, final_servo_angle, WALL_ALIGN_CREEP_SPEED)

                    if DEBUG_UI_OVERLAYS:
                        loop_duration = time.monotonic() - loop_start_time
                        fps = 1.0 / loop_duration if loop_duration > 0 else 0
                        draw_segmentation_overlay(processed_frame, lab, detections, wall_contours)
                        cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(processed_frame, f"State: {current_robot_state} | Align Side: {align_side}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(processed_frame, f"FrontPts: {front_count} RearPts: {rear_count} | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        if STREAM_VIDEO:
                            with output_frame_lock:
                                output_frame = processed_frame.copy()

                    time.sleep(0.02)
                    continue

            # 5. Output packets to hardware layers
            # (reached by: side-panic override, AND vision obstacle avoidance
            #  in both "detected" and "coasting/latched" states -- the wall-align
            #  default branch above has its own continue and never reaches here)
            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
            send_esp_packet(esp_ser, final_servo_angle, robot_speed_current)

            # Frame serving calculations
            loop_duration = time.monotonic() - loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0

            if DEBUG_UI_OVERLAYS:
                draw_segmentation_overlay(processed_frame, lab, detections, wall_contours)
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"Lines Logged: {blue_count}/12 | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                if STREAM_VIDEO:
                    with output_frame_lock:
                        output_frame = processed_frame.copy()

            time.sleep(0.02)

    except Exception as e:
        print(f"[SYSTEM FAILURE] Main runtime error tripped: {e}")
    finally:
        emergency_shutdown_handler(None, None)

# --- FLASK JPEGMOTION WEB SERVER PIPELINES ---
# Web layout / streams ported from obs_main_lab_detection_masked.py:
# original annotated feed, top-down LiDAR map, track mask (drivable area) feed,
# per-colour LAB segment feeds + HSV tuning feeds, and the preset tuning endpoints.

# --- WEB STREAM TUNING (keep the 6 live feeds from saturating the Pi/Wi-Fi) ---
STREAM_WIDTH = 480          # output width per feed (px); aspect kept
STREAM_MAX_FPS = 12         # cap per-feed frame rate
STREAM_JPEG_QUALITY = 65    # JPEG quality (lower = smaller/faster)


def _front_distance_mm(scan_data):
    """Median LiDAR distance straight ahead (-5..+5 deg), or None."""
    if not scan_data:
        return None
    fronts = [d for a, d in scan_data.items() if -5 <= a <= 5 and d > 0]
    return float(np.median(fronts)) if fronts else None


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
    """Generate an MJPEG stream for 'original', 'lidar', 'track' or a colour name."""
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


@app.route("/video_hsv/<mode>")
def video_feed_hsv(mode):
    if mode not in STREAM_COLORS:
        return "Unknown stream", 404
    return Response(generate_hsv_stream(mode), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/get_lidar")
def get_lidar():
    """Telemetry for the web panel: front / left / right LiDAR distances (mm)."""
    with lidar_data_lock:
        scan = dict(latest_lidar_data)
    front = _front_distance_mm(scan)
    left_pts = [scan[a] for a in range(-105, -75) if a in scan and scan[a] > 0]
    right_pts = [scan[a] for a in range(75, 105) if a in scan and scan[a] > 0]
    left = sum(left_pts) / len(left_pts) if left_pts else None
    right = sum(right_pts) / len(right_pts) if right_pts else None
    return jsonify({"front": front, "left": left, "right": right})


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


if __name__ == '__main__':
    # OPTIMIZATION: cap OpenCV's internal thread pool so it doesn't compete
    # with the control/camera/Flask Python threads for the Pi's limited cores.
    cv2.setNumThreads(2)

    print("--- Booting WRO 2026 Unified Obstacle Round System ---")
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

    #got issue of brushing to the wall cause of side alert avoidance not well working and red color obstacle proper avoidance