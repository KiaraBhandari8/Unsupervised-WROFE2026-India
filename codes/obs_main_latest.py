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
    from lidar_steering_new import get_wall_parallel_sector_stats, get_wall_parallel_error, PARALLEL_TOLERANCE_MM
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
latest_yaw = 0.0
yaw_lock = threading.Lock()
gyro_ok = False

# Shared buffer for the latest camera frame and its lock
latest_camera_frame = None
latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

# --- POWER / CPU BUDGET (3A-friendly profile) ---
CONTROL_LOOP_MAX_HZ = 30.0
STARTUP_STAGE_DELAY_SEC = 2.0


app = Flask(__name__)
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
CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
LAB_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3
LAB_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3


# ===================== LAB COLOUR DETECTION =====================
PRESETS_FILE = "/home/pi8/wrofe2025/vision_presets.json"

DEFAULT_PRESETS = {
    "red":    {"l_min": 0,   "l_max": 255, "a_min": 146, "a_max": 255, "b_min": 100, "b_max": 255},
    "blue":   {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 140, "b_min": 0,   "b_max": 120},
    "green":  {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 120, "b_min": 80,  "b_max": 200},
    "orange": {"l_min": 0,   "l_max": 255, "a_min": 140, "a_max": 255, "b_min": 140, "b_max": 255},
    "white":  {"l_min": 100, "l_max": 255, "a_min": 0,   "a_max": 255, "b_min": 0,   "b_max": 255},
}

STREAM_COLORS = ["red", "green", "orange", "blue", "white"]


def load_presets():
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

LAB_MIN_CONTOUR_AREA = 900
LAB_MIN_WIDTH = 30
LAB_ROI_TOP_FRAC = 0.35
COLOR_STEER_MAGNITUDE = 22
COLOR_STEER_MAGNITUDE_RED = 16
RED_AVOID_MAX_MAG_DEG = 15.0

AVOID_STANDOFF_MM = 250
AVOID_CLEARANCE_MM = 450
AVOID_PASS_CX_FRAC = 0.78
REALIGN_DURATION_SEC = 1
AVOID_WIDTH_TRIGGER = 30
AVOID_DIST_GAIN_REF_MM = 350.0

WHITE_FLOOR_BAND_PX = 24
WHITE_FLOOR_MIN_RATIO = 0.30
TRACK_MASK_CLOSE_PX = 25
TRACK_MIN_AREA_FRAC = 0.02
TRACK_BASE_DILATE_PX = 12


def build_track_mask(white_mask):
    h, w = white_mask.shape[:2]
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
        top = stats[lbl, cv2.CC_STAT_TOP]
        height = stats[lbl, cv2.CC_STAT_HEIGHT]
        reaches_bottom = (top + height) >= (h - 2)
        if reaches_bottom and area > best_area:
            best_label, best_area = lbl, area

    if best_label == 0:
        return np.zeros_like(white_mask)

    track = np.where(labels == best_label, 255, 0).astype(np.uint8)
    if TRACK_BASE_DILATE_PX > 0:
        dk = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (TRACK_BASE_DILATE_PX, TRACK_BASE_DILATE_PX))
        track = cv2.dilate(track, dk, iterations=1)
    return track


def lab_mask(lab_frame, color):
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
    x1, y1, x2, y2 = bbox
    h, w = track_mask.shape[:2]
    if cv2.countNonZero(track_mask) == 0:
        return False

    by1 = min(y2, h)
    by2 = min(y2 + WHITE_FLOOR_BAND_PX, h)
    bx1 = max(0, x1)
    bx2 = min(x2, w)
    if (by2 - by1) < 2 or (bx2 - bx1) < 2:
        cx = int(np.clip((x1 + x2) // 2, 0, w - 1))
        cy = int(np.clip(y2 - 1, 0, h - 1))
        return track_mask[cy, cx] != 0
    band = track_mask[by1:by2, bx1:bx2]
    ratio = cv2.countNonZero(band) / band.size
    return ratio >= WHITE_FLOOR_MIN_RATIO


def detect_lab_pillars(lab_frame):
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
                    if (y + h) < roi_top:
                        continue
                    if not _on_drivable_area(track_mask, bbox):
                        continue
                    dets.append({
                        'class': cls,
                        'area': float(ar),
                        'bbox': bbox,
                        'width': w,
                        'cx': x + w // 2,
                    })
    return dets


# --- Obstacle-avoidance maneuver state (module level, persists across loops) ---
avoid_state = "IDLE"
avoid_color = None
realign_end_time = 0.0
pre_avoid_yaw = None
recenter_end_time = 0.0


def _gyro_recenter_angle(current_yaw, target_yaw):
    err = target_yaw - current_yaw
    offset = GYRO_RECENTER_SIGN * GYRO_RECENTER_KP * err
    offset = max(-GYRO_RECENTER_MAX_OFFSET, min(GYRO_RECENTER_MAX_OFFSET, offset))
    return SERVO_CENTER_ANGLE + offset


def _front_distance_mm(scan_data):
    if not scan_data:
        return None
    fronts = [d for a, d in scan_data.items() if -5 <= a <= 5 and d > 0]
    return float(np.median(fronts)) if fronts else None


def _avoid_servo_angle(color, front_dist, cx=None, lab_w=None):
    base = COLOR_STEER_MAGNITUDE_RED if color == 'red' else COLOR_STEER_MAGNITUDE
    mag = base
    if front_dist and front_dist > 0:
        mag = base * max(0.6, min(1.8, AVOID_DIST_GAIN_REF_MM / front_dist))
    if cx is not None and lab_w is not None and lab_w > 0:
        if color == 'green':
            side_factor = max(0.0, 1.0 - 2.0 * cx / lab_w)
            mag *= (1.0 + side_factor)
        else:
            side_factor = max(0.0, 2.0 * cx / lab_w - 1.0)
            mag *= (1.0 + side_factor)
    if color == 'red':
        mag = min(mag, RED_AVOID_MAX_MAG_DEG)
    return SERVO_CENTER_ANGLE - mag if color == 'green' else SERVO_CENTER_ANGLE + mag


def manage_color_avoidance(dets, scan_data, lab_w, current_yaw=None):
    global avoid_state, avoid_color, realign_end_time
    global pre_avoid_yaw, recenter_end_time
    now = time.time()
    have_gyro = GYRO_ENABLED and current_yaw is not None

    objs = [d for d in dets if d['class'] in ('red', 'green')]
    target = max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None
    front_dist = _front_distance_mm(scan_data)

    if avoid_state == "REALIGN":
        if now < realign_end_time:
            return True, SERVO_CENTER_ANGLE, f"realign_{avoid_color}"
        if have_gyro and pre_avoid_yaw is not None:
            avoid_state = "RECENTER"
            recenter_end_time = now + GYRO_RECENTER_DURATION_SEC
            return (True, _gyro_recenter_angle(current_yaw, pre_avoid_yaw),
                    f"recenter_{avoid_color}")
        avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None

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

    if avoid_state == "TURNING":
        color = avoid_color
        passed = False
        if target is None or target['class'] != color:
            passed = True
        else:
            cx = target['cx']
            if color == 'green' and cx > lab_w * AVOID_PASS_CX_FRAC:
                passed = True
            elif color == 'red' and cx < lab_w * (1.0 - AVOID_PASS_CX_FRAC):
                passed = True
        if passed:
            avoid_state = "REALIGN"
            realign_end_time = now + REALIGN_DURATION_SEC
            return True, SERVO_CENTER_ANGLE, f"realign_{color}"
        cx = target['cx'] if target else None
        return True, _avoid_servo_angle(color, front_dist, cx, lab_w), f"avoid_{color}"

    if target is not None:
        width_trigger = target['width'] >= AVOID_WIDTH_TRIGGER
        dist_trigger = (front_dist is not None
                        and front_dist <= (AVOID_STANDOFF_MM + AVOID_CLEARANCE_MM))
        start = width_trigger or dist_trigger
        if start:
            avoid_color = target['class']
            avoid_state = "TURNING"
            pre_avoid_yaw = current_yaw if have_gyro else None
            return True, _avoid_servo_angle(avoid_color, front_dist, target['cx'], lab_w), f"avoid_{avoid_color}"

    return False, None, "none"


def filter_blue_objects_lab(lab_frame):
    return lab_mask(lab_frame, 'blue')


def detect_color_binary(mask, threshold=4000):
    return cv2.countNonZero(mask) > threshold


def draw_detections(frame, dets, scale_x=1.0, scale_y=1.0):
    for d in dets:
        x1, y1, x2, y2 = d['bbox']
        x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
        y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
        clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
        cv2.putText(frame, d['class'], (x1, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)


# ===================== DEBUG VISUALIZATION (rendering only) =====================
VIZ_WALL_L_CLAMP_LO = 50
VIZ_WALL_L_CLAMP_HI = 135
VIZ_WALL_MAX_DARK_FRAC = 0.60
VIZ_WALL_MIN_AREA = 1500
VIZ_WALL_MAX_CONTOURS = 6
VIZ_WALL_APPROX_EPS = 0.004
VIZ_WALL_CLOSE_PX = 7
VIZ_WALL_COLOR = (0, 255, 255)
VIZ_WALL_THICKNESS = 2

VIZ_GREEN_ORIGIN_FRAC = (0.25, 0.93)
VIZ_RED_ORIGIN_FRAC = (0.75, 0.93)

VIZ_LIDAR_MAP_SIZE = 480
VIZ_LIDAR_MAX_RANGE_MM = 3000.0
VIZ_LIDAR_WALL_COLOR = (219, 119, 31)
VIZ_LIDAR_BG_POINT_COLOR = (70, 70, 70)


def extract_wall_contours(lab_frame):
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
    for c in contours:
        pts = c.reshape(-1, 2).astype(np.float32)
        pts[:, 0] *= scale_x
        pts[:, 1] *= scale_y
        cv2.polylines(frame, [pts.astype(np.int32)], True,
                      VIZ_WALL_COLOR, VIZ_WALL_THICKNESS, cv2.LINE_AA)


def _primary_obstacle(objs):
    return max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None


def draw_obstacle_overlay(frame, dets, scale_x=1.0, scale_y=1.0):
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


def render_lidar_map(scan_data, clockwise=True):
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
LIDAR_SAFETY_DISTANCE_MM = 200
CLOCKWISE_WALL_FOLLOWING = True

# --- GYRO LANE RE-CENTERING (after each obstacle pass) ---
GYRO_ENABLED = True
GYRO_RECENTER_DURATION_SEC = 1.3
GYRO_RECENTER_TOL_DEG = 4.0
GYRO_RECENTER_KP = 1.2
GYRO_RECENTER_MAX_OFFSET = 22
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

# --- CORNER MANEUVER CONSTANTS (ported from esp32_obstacle_triangle_Cc-priority1.py) ---
FRONT_SCAN_ANGLE_DEG = 15
FRONT_TURN_TRIGGER_MM = 200.0

CORNER_DETECTION_COOLDOWN_SEC = 10
CORNER_PIVOT_SPEED = 140
CORNER_PIVOT_SAFETY_TIMEOUT = 3.5
CORNER_BACKWARD_DURATION = 3.0
CORNER_BACKWARD_TOF_TARGET_MM = 40.0
CORNER_BACKWARD_TOF_TOLERANCE_MM = 10.0
CORNER_BRAKE_DELAY = 0.20
CORNER_CHECK_LOG_EVERY_N_FRAMES = 10
STATE_LOG_EVERY_N_FRAMES = 10

CORNER_SIGNATURE_STOP_DELAY_SEC = 0.25
CORNER_SIGNATURE_FRONT_TRIGGER_MM = 1000.0
LANE_REFERENCE_ARC_THRESHOLD_MM = 600.0
CORNER_APPROACH_TRIGGER_MM = 450.0

CORNER_ARC_STEER_OFFSET = 60
CORNER_ARC_PIVOT_SPEED = 140
CORNER_ARC_PIVOT_SAFETY_TIMEOUT = 3.0
CORNER_ARC_CLEAR_VIEW_TOF_MM = 400.0

TURN_TARGET_RIGHT_DEGREES = 45.0
TURN_TARGET_LEFT_DEGREES = 45.0
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0
WALL_ALIGN_CREEP_SPEED = 140
WALL_ALIGN_SAFETY_TIMEOUT = 3.0
WALL_ALIGN_NO_WALL_TIMEOUT = 1.0
CORNER_APPROACH_WALL_LOSS_THRESHOLD_MM = 400.0
WALL_FOLLOW_TARGET_MM = 600

# Rear ToF sensor (backward corner docking)
latest_tof_distance_mm = None
tof_data_lock = threading.Lock()
tof_sensor = None

# --- DEBUGGING AND UI ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = False

STREAM_WIDTH = 480
STREAM_MAX_FPS = 12
STREAM_JPEG_QUALITY = 65

STARTUP_TEST_ENABLED = False

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
    CORNER_PRE_ALIGN = "CORNER_PRE_ALIGN"
    CORNER_APPROACH_WALL = "CORNER_APPROACH_WALL"
    CORNER_ACTIVE_PIVOT = "CORNER_ACTIVE_PIVOT"
    CORNER_ARC_ACTIVE_PIVOT = "CORNER_ARC_ACTIVE_PIVOT"
    CORNER_ALIGN_BACKWARD = "CORNER_ALIGN_BACKWARD"
    WALL_ALIGN_CORRECTION = "WALL_ALIGN_CORRECTION"

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
    for angle in [SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE, SERVO_CENTER_ANGLE + 20, SERVO_CENTER_ANGLE]:
        ser.write(f"STR:{angle},SPD:0\n".encode())
        ser.flush()
        time.sleep(0.3)
    print("[STARTUP] Motor pulse...")
    cmd(SERVO_CENTER_ANGLE, 0.4)
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


# --- ESP32 gyro re-zero helper (corner state machine) ---
def reset_yaw():
    if ser and ESP32_OK:
        try:
            ser.write(b"RST_YAW\n")
            ser.flush()
        except Exception:
            pass


# --- Compensated front distance for corner detection ---
def get_compensated_front_distance(scan_data, current_yaw):
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


# --- ToF Data Acquisition Background Task ---
def tof_acquisition_thread_func(sensor_instance):
    global latest_tof_distance_mm, tof_data_lock
    print("ToF acquisition thread started.")
    tof_log_counter = 0
    while not shutdown_event.is_set():
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


# --- Camera Acquisition Thread ---
def camera_acquisition_thread_func(picam2_instance, stop_event, lab_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition and processing thread started.")
    frame_seq = 0
    try:
        while not stop_event.is_set():
            yuv420 = picam2_instance.capture_array("lores")
            capture_ts = time.time()

            frame_bgr = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)

            lab_source_frame = cv2.resize(
                frame_bgr,
                lab_processing_size,
                interpolation=cv2.INTER_AREA
            )
            lab_frame = cv2.cvtColor(lab_source_frame, cv2.COLOR_BGR2LAB)

            frame_seq += 1
            with camera_frame_lock:
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['lab'] = lab_frame
                latest_processed_frames['seq'] = frame_seq
                latest_processed_frames['ts'] = capture_ts

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

    if ser and ESP32_OK:
        try:
            ser.write(b"RST_YAW\n")
            ser.flush()
        except Exception:
            pass
    gyro_acquisition_thread = threading.Thread(
        target=gyro_acquisition_thread_func,
        args=(ser, camera_thread_stop_event)
    )
    gyro_acquisition_thread.daemon = True
    gyro_acquisition_thread.start()

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

    # Initialize Rear ToF Distance Sensor (for corner backward-docking phase).
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

        # Corner maneuver state variables (ported from esp32_obstacle_triangle_Cc-priority1.py)
        turn_count = 0
        baseline_start_yaw = 0.0
        turn_direction = None
        use_reverse_arc = False
        locked_lane_reference_mm = None
        lane_number = None
        pivot_phase_start_time = 0.0
        backward_phase_start_time = 0.0
        corner_cooldown_end_time = 0.0
        corner_check_frame_counter = 0
        state_log_frame_counter = 0
        align_phase_start_time = 0.0
        align_return_state = None
        was_avoiding_obstacle = False
        align_no_wall_start_time = 0.0
        pre_align_start_time = 0.0
        pre_align_no_wall_start_time = 0.0

        # Corner PIDs (same tuning as priority1)
        gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
        wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)
        alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)

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

            # --- LAB colour pillar detection (runs before the corner logic so the
            # corner overlay can draw obstacle boxes on every frame). ---
            detections = detect_lab_pillars(lab)
            det_method = "LAB"

            # --- Corner detection side averages (LiDAR) ---
            if scan_data:
                left_pts = [scan_data[a] for a in range(-105, -75) if a in scan_data and scan_data[a] > 0]
                right_pts = [scan_data[a] for a in range(75, 105) if a in scan_data and scan_data[a] > 0]
                avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
                avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
            else:
                left_pts = []
                right_pts = []
                avg_left = 2000.0
                avg_right = 2000.0

            yaw_for_corner = current_yaw if current_yaw is not None else 0.0
            avg_front_baseline = get_compensated_front_distance(scan_data, yaw_for_corner)
            in_cooldown = time.monotonic() < corner_cooldown_end_time

            if scan_data and corner_check_frame_counter % CORNER_CHECK_LOG_EVERY_N_FRAMES == 0:
                cooldown_note = f" | COOLDOWN ({corner_cooldown_end_time - time.monotonic():.1f}s left)" if in_cooldown else ""
                print(f"[{current_robot_state}] [CORNER CHECK] Front: {avg_front_baseline:.1f}mm | Left: {avg_left:.1f}mm | Right: {avg_right:.1f}mm | Yaw: {yaw_for_corner:+.1f}deg{cooldown_note}")
            corner_check_frame_counter += 1

            # ====================================================
            # PRIORITY LEVEL 1: ACTIVE CORNERING RUNTIME EXECUTION
            # (ported from esp32_obstacle_triangle_Cc-priority1.py)
            # ====================================================
            if current_robot_state in [RobotState.CORNER_PRE_ALIGN, RobotState.CORNER_APPROACH_WALL, RobotState.CORNER_ACTIVE_PIVOT, RobotState.CORNER_ARC_ACTIVE_PIVOT, RobotState.CORNER_ALIGN_BACKWARD]:

                state_log_frame_counter += 1
                should_log_state = (state_log_frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

                if current_robot_state == RobotState.CORNER_PRE_ALIGN:
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                    front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data if scan_data else {}, align_side)
                    parallel_error = get_wall_parallel_error(scan_data if scan_data else {}, align_side)
                    elapsed_pre_align = time.monotonic() - pre_align_start_time

                    if parallel_error is None:
                        if pre_align_no_wall_start_time == 0.0:
                            pre_align_no_wall_start_time = time.monotonic()
                        no_wall_elapsed = time.monotonic() - pre_align_no_wall_start_time
                    else:
                        pre_align_no_wall_start_time = 0.0
                        no_wall_elapsed = 0.0

                    is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                    front_reached = avg_front_baseline <= CORNER_APPROACH_TRIGGER_MM
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

                    if is_aligned or front_reached or hard_timeout or no_wall_timeout:
                        reason = "aligned" if is_aligned else "front_reached" if front_reached else "timeout" if hard_timeout else "no_wall"
                        print(f"[{current_robot_state}] [PRE-TURN STRAIGHTEN] Exit condition reached ({reason}). Proceeding to straight corner approach.")
                        cmd(SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)
                        alignment_pid.reset()
                        current_robot_state = RobotState.CORNER_APPROACH_WALL
                        cmd(SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)
                    else:
                        if parallel_error is None:
                            cmd(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED / 255.0)
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = alignment_pid.update(normalized_error)
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            cmd(final_servo_angle, WALL_ALIGN_CREEP_SPEED / 255.0)

                elif current_robot_state == RobotState.CORNER_APPROACH_WALL:
                    align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

                    if avg_front_baseline <= CORNER_APPROACH_TRIGGER_MM:
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Approach limit reached. Front {avg_front_baseline:.1f}mm <= {CORNER_APPROACH_TRIGGER_MM:.0f}mm. Applying hard brake...")
                        cmd(SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Resetting gyro registers before turn...")
                        reset_yaw()
                        time.sleep(0.1)
                        yaw_for_corner = 0.0

                        pivot_phase_start_time = time.monotonic()
                        baseline_start_yaw = yaw_for_corner

                        if use_reverse_arc:
                            arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Entering reverse arc pivot (Lane 1): steer={arc_steer_angle}deg speed=-{CORNER_ARC_PIVOT_SPEED}")
                            current_robot_state = RobotState.CORNER_ARC_ACTIVE_PIVOT
                            cmd(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED / 255.0)
                        else:
                            final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                            print(f"[{current_robot_state}] [CORNER EXECUTION] Locking wheels to extreme pivot angle (Lane {lane_number}): {final_servo}deg")
                            cmd(final_servo, 0)
                            time.sleep(0.15)
                            current_robot_state = RobotState.CORNER_ACTIVE_PIVOT
                            cmd(final_servo, CORNER_PIVOT_SPEED / 255.0)
                    else:
                        display_text = f"[CORNER] Phase 2: Straight Approach | Front: {avg_front_baseline:.0f}mm / {CORNER_APPROACH_TRIGGER_MM:.0f}mm"
                        if should_log_state:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] {display_text} | Lane: {lane_number}")
                        cmd(SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED)

                elif current_robot_state == RobotState.CORNER_ACTIVE_PIVOT:
                    elapsed_pivot = time.monotonic() - pivot_phase_start_time
                    yaw_delta_signed = yaw_for_corner - baseline_start_yaw

                    if turn_direction == "RIGHT":
                        target_degrees = TURN_TARGET_RIGHT_DEGREES
                        pivot_complete = yaw_delta_signed <= -TURN_TARGET_RIGHT_DEGREES
                    else:
                        target_degrees = TURN_TARGET_LEFT_DEGREES
                        pivot_complete = yaw_delta_signed >= TURN_TARGET_LEFT_DEGREES

                    display_text = f"[CORNER] Phase 3: Gyro Pivot | Yaw: {yaw_delta_signed:+.1f}deg / target {target_degrees}deg ({turn_direction}) | Lane: {lane_number}"
                    pivot_timed_out = elapsed_pivot >= CORNER_PIVOT_SAFETY_TIMEOUT

                    if pivot_complete or pivot_timed_out:
                        if pivot_timed_out and not pivot_complete:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] WARNING: Pivot safety timeout hit before target yaw reached (only {yaw_delta_signed:+.1f}deg / target {target_degrees}deg). Check gyro data.")
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Pivot complete ({yaw_delta_signed:+.1f}deg). Braking...")
                        cmd(SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Resetting chassis orientation...")
                        reset_yaw()
                        time.sleep(0.1)
                        yaw_for_corner = 0.0

                        # All lanes run the backward phase: reverse straight into
                        # the dock until the rear ToF reads < 5cm (mirrored
                        # parallel-error steering keeps the chassis aligned).
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Lane {lane_number} -> backward phase (dock to rear ToF).")
                        alignment_pid.reset()
                        backward_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                        cmd(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                    else:
                        final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                        cmd(final_servo, CORNER_PIVOT_SPEED / 255.0)
                        if should_log_state:
                            print(f"[{current_robot_state}] [PIVOT] Yaw: {yaw_delta_signed:+.1f}deg / target {target_degrees}deg | Elapsed: {elapsed_pivot:.2f}s")

                elif current_robot_state == RobotState.CORNER_ARC_ACTIVE_PIVOT:
                    elapsed_arc = time.monotonic() - pivot_phase_start_time
                    yaw_delta_signed = yaw_for_corner - baseline_start_yaw
                    arc_complete = yaw_delta_signed <= -TURN_TARGET_RIGHT_DEGREES
                    arc_timed_out = elapsed_arc >= CORNER_ARC_PIVOT_SAFETY_TIMEOUT

                    with tof_data_lock:
                        rear_distance = latest_tof_distance_mm

                    rear_tof_clear_view_hit = (
                        rear_distance is not None
                        and rear_distance <= CORNER_ARC_CLEAR_VIEW_TOF_MM
                    )

                    display_text = f"[CORNER] Arc Reverse Pivot | Yaw: {yaw_delta_signed:+.1f}deg / target -{TURN_TARGET_RIGHT_DEGREES}deg | Rear: {rear_distance if rear_distance is not None else float('nan'):.0f}mm"
                    if should_log_state:
                        print(f"[{current_robot_state}] [ARC PIVOT] Yaw: {yaw_delta_signed:+.1f}deg | Rear: {rear_distance} | Elapsed: {elapsed_arc:.2f}s")

                    if arc_complete or arc_timed_out or rear_tof_clear_view_hit:
                        if arc_timed_out and not arc_complete:
                            print(f"[{current_robot_state}] [ARC PIVOT] WARNING: Arc safety timeout hit before target yaw reached (only {yaw_delta_signed:+.1f}deg). Check gyro data.")
                        if rear_tof_clear_view_hit:
                            print(f"[{current_robot_state}] [ARC PIVOT] Rear ToF clear-view distance reached ({rear_distance:.0f}mm <= {CORNER_ARC_CLEAR_VIEW_TOF_MM:.0f}mm). Ending arc early for forward visibility.")
                        print(f"[{current_robot_state}] [ARC PIVOT] Arc complete ({yaw_delta_signed:+.1f}deg). Hard braking chassis...")
                        cmd(SERVO_CENTER_ANGLE, 0)
                        time.sleep(CORNER_BRAKE_DELAY)

                        print(f"[{current_robot_state}] [ARC PIVOT] Resetting chassis orientation before backward phase...")
                        reset_yaw()
                        time.sleep(0.1)
                        yaw_for_corner = 0.0

                        print(f"[{current_robot_state}] [ARC PIVOT] -> backward phase (dock to rear ToF).")
                        alignment_pid.reset()
                        backward_phase_start_time = time.monotonic()
                        current_robot_state = RobotState.CORNER_ALIGN_BACKWARD
                        cmd(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                    else:
                        arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                        cmd(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED / 255.0)

                elif current_robot_state == RobotState.CORNER_ALIGN_BACKWARD:
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
                        print(f"[{current_robot_state}] [CORNER EXECUTION] Phase 4: Reversing | Rear: {rear_distance} | Elapsed: {elapsed_back:.2f}s | Yaw: {yaw_for_corner:+.1f}deg")

                    if tof_reached or hard_timeout_backward:
                        if hard_timeout_backward and not tof_reached:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] WARNING: Backward safety timeout hit before ToF target reached (rear={rear_distance}). Check ToF sensor.")
                        else:
                            print(f"[{current_robot_state}] [CORNER EXECUTION] ToF target reached (rear={rear_distance:.0f}mm). Hard braking applied.")
                        cmd(SERVO_CENTER_ANGLE, 0)
                        time.sleep(0.3)

                        print(f"[{current_robot_state}] [CORNER EXECUTION] Cleaning up spatial registers before resuming wall-follow...")
                        reset_yaw()
                        time.sleep(0.1)
                        yaw_for_corner = 0.0

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
                        # Steer opposite to the forward alignment correction
                        # (mirrored servo angle) so the chassis stays parallel to
                        # the tracked wall while reversing.
                        align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                        parallel_error = get_wall_parallel_error(scan_data if scan_data else {}, align_side)
                        if parallel_error is None:
                            cmd(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = -alignment_pid.update(normalized_error)
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            final_servo_angle = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                            cmd(final_servo_angle, -ROBOT_MANEUVER_SPEED)

                # Shared debug/stream overlay for every corner sub-state.
                if DEBUG_UI_OVERLAYS or STREAM_VIDEO:
                    processed_frame = frame_bgr.copy()
                    scale_x = frame_bgr.shape[1] / lab.shape[1]
                    scale_y = frame_bgr.shape[0] / lab.shape[0]
                    wall_contours = extract_wall_contours(lab)
                    draw_wall_contours(processed_frame, wall_contours, scale_x, scale_y)
                    draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)
                    if DEBUG_UI_OVERLAYS:
                        loop_duration = time.monotonic() - loop_start_time
                        fps = 1.0 / loop_duration if loop_duration > 0 else 0
                        cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(processed_frame, f"State: {current_robot_state} | Turns: {turn_count}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(processed_frame, f"Lines Logged: {blue_count}/{max_line_crossings} | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    with output_frame_lock:
                        output_frame = processed_frame

                pace_remaining = (1.0 / CONTROL_LOOP_MAX_HZ) - (time.monotonic() - loop_start_time)
                if pace_remaining > 0:
                    time.sleep(pace_remaining)
                continue

            # ====================================================
            # PRIORITY LEVEL 2.5: POST-CORNER WALL ALIGNMENT
            # (straighten the chassis against the tracked wall before handing
            # control back to normal wall-following)
            # ====================================================
            if current_robot_state == RobotState.WALL_ALIGN_CORRECTION:
                align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data if scan_data else {}, align_side)
                parallel_error = get_wall_parallel_error(scan_data if scan_data else {}, align_side)
                elapsed_align = time.monotonic() - align_phase_start_time
                should_log_state = (loop_counter % STATE_LOG_EVERY_N_FRAMES == 0)

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

                if should_log_state:
                    print(
                        f"[{current_robot_state}] [WALL ALIGN] Side: {align_side} | Front: {front_avg if front_avg is not None else float('nan'):.1f}mm | "
                        f"Rear: {rear_avg if rear_avg is not None else float('nan'):.1f}mm | Err: {parallel_error if parallel_error is not None else float('nan'):.1f}mm | "
                        f"FrontPts: {front_count} | RearPts: {rear_count} | Elapsed: {elapsed_align:.2f}s"
                    )

                if is_aligned or hard_timeout or no_wall_timeout:
                    reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                    print(f"[{current_robot_state}] [WALL ALIGN] Exit condition reached ({reason}). Returning to {align_return_state}.")
                    cmd(SERVO_CENTER_ANGLE, 0)
                    current_robot_state = align_return_state or RobotState.LIDAR_WALL_FOLLOWING
                    align_return_state = None
                    align_phase_start_time = 0.0
                    align_no_wall_start_time = 0.0
                    pace_remaining = (1.0 / CONTROL_LOOP_MAX_HZ) - (time.monotonic() - loop_start_time)
                    if pace_remaining > 0:
                        time.sleep(pace_remaining)
                    continue

                if parallel_error is None:
                    cmd(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED / 255.0)
                else:
                    normalized_error = parallel_error if align_side == "left" else -parallel_error
                    pid_output = alignment_pid.update(normalized_error)
                    target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                    final_sa = int(round(np.clip(target_servo_angle, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                    cmd(final_sa, WALL_ALIGN_CREEP_SPEED / 255.0)

                if DEBUG_UI_OVERLAYS or STREAM_VIDEO:
                    processed_frame = frame_bgr.copy()
                    scale_x = frame_bgr.shape[1] / lab.shape[1]
                    scale_y = frame_bgr.shape[0] / lab.shape[0]
                    wall_contours = extract_wall_contours(lab)
                    draw_wall_contours(processed_frame, wall_contours, scale_x, scale_y)
                    draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)
                    if DEBUG_UI_OVERLAYS:
                        loop_duration = time.monotonic() - loop_start_time
                        fps = 1.0 / loop_duration if loop_duration > 0 else 0
                        cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(processed_frame, f"State: {current_robot_state} | Align Side: {align_side}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(processed_frame, f"FrontPts: {front_count} RearPts: {rear_count} | FPS: {int(fps)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    with output_frame_lock:
                        output_frame = processed_frame

                pace_remaining = (1.0 / CONTROL_LOOP_MAX_HZ) - (time.monotonic() - loop_start_time)
                if pace_remaining > 0:
                    time.sleep(pace_remaining)
                continue

            # Compute avoidance inputs BEFORE the corner check so a corner
            # trigger can never preempt an active avoidance/escape maneuver.
            # This preserves the exact priority scheme from
            # obs_main_lab_detection_masked_corner.py:
            #   imminent collision > vision avoidance > side > corner detection.
            avoid_engaged, avoid_angle_val, logic_label = manage_color_avoidance(
                detections, scan_data, lab.shape[1])
            escape_direction = check_imminent_collision_and_get_escape_route(scan_data)
            side_alert_status = check_lidar_side_alerts(scan_data)

            # ====================================================
            # PRIORITY LEVEL 2: RADAR FRONTAL TRIPWIRE CHECK (CORNER DETECTION)
            # Only evaluated when no escape/avoidance/side-alert is active.
            # ====================================================
            is_corner_signature = False
            if (lidar_scanner
                and scan_data
                and not in_cooldown
                and escape_direction is None
                and side_alert_status is None
                and avg_front_baseline <= CORNER_SIGNATURE_FRONT_TRIGGER_MM
                and ((avg_left < 950.0 and avg_right > 1600.0)
                     or (avg_right < 900.0 and avg_left > 1800.0))):
                is_corner_signature = True

            if is_corner_signature:
                print(f"\n[CORNER INTERSECTION] Front Wall at: {avg_front_baseline:.1f}mm. Stopping chassis... (signature)")
                cmd(SERVO_CENTER_ANGLE, 0)
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

                # Lane detection: lock the tracked-wall distance at the instant
                # the corner is detected. CW course tracks the LEFT wall, CCW
                # tracks the RIGHT wall.
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

                print(f"[PRE-TURN ACTUATION] Entering pre-turn wall alignment before {turn_direction} corner...")
                print("[ACTION] Flushing scrub vibration error -> Resetting Gyro Yaw to 0deg...")
                reset_yaw()
                time.sleep(0.1)

                yaw_for_corner = 0.0
                baseline_start_yaw = 0.0
                pre_align_start_time = time.monotonic()
                pre_align_no_wall_start_time = 0.0
                current_robot_state = RobotState.CORNER_PRE_ALIGN

                cmd(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED / 255.0)
                pace_remaining = (1.0 / CONTROL_LOOP_MAX_HZ) - (time.monotonic() - loop_start_time)
                if pace_remaining > 0:
                    time.sleep(pace_remaining)
                continue

            label_method = f"[{det_method}]"

            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # ====================================================
            # PRIORITY 0: IMMINENT COLLISION (always highest)
            # ====================================================
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

            # PRIORITY 1: CAMERA OBSTACLE (LAB colour) -- committed maneuver.
            elif avoid_engaged:
                robot_speed_current = ROBOT_MANEUVER_SPEED
                current_robot_state = (RobotState.RED_AVOIDANCE
                                       if 'red' in logic_label
                                       else RobotState.GREEN_AVOIDANCE)
                target_servo_angle = avoid_angle_val
                display_text = f"MODE: {logic_label} {det_method} | {int(round(target_servo_angle))}deg"
                if time.time() - print_timer >= 0.5:
                    print_timer = time.time()
                    print(f"Vision: {logic_label} -> servo {round(target_servo_angle)}")

            # PRIORITY 2: SIDE OBSTACLE (LIDAR)
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

            # PRIORITY 3: LIDAR WALL FOLLOWING (triangle-based via calculate_steering_error)
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
                        # Triangle-based wall following: calculate_steering_error
                        # samples the LiDAR at specific angular sectors (triangle
                        # sides) to compute the lateral error from the corridor
                        # centre.  When both walls are visible the error balances
                        # left vs right; when only the following wall is visible
                        # it holds the target distance.  The safety_distance_mm
                        # (= 200 mm = 20 cm) triggers a STOP if anything is too
                        # close directly ahead.
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
                print(f"[{loop_counter}] {current_robot_state} | Angle:{final_angle} Speed:{robot_speed_current:.2f} FPS:{int(fps)} Det:{det_method}({len(detections)}) {logic_label}")

            # Build the ORIGINAL display frame (with detection boxes) for the web page.
            if STREAM_VIDEO:
                processed_frame = frame_bgr.copy()
                scale_x = frame_bgr.shape[1] / lab.shape[1]
                scale_y = frame_bgr.shape[0] / lab.shape[0]
                wall_contours = extract_wall_contours(lab)
                draw_wall_contours(processed_frame, wall_contours, scale_x, scale_y)
                draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)
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
    with camera_frame_lock:
        if 'lab' not in latest_processed_frames:
            return None
        lab = latest_processed_frames['lab']
        bgr = latest_processed_frames['bgr']
    small_bgr = cv2.resize(bgr, (lab.shape[1], lab.shape[0]), interpolation=cv2.INTER_NEAREST)
    mask = lab_mask(lab, color)
    return cv2.bitwise_and(small_bgr, small_bgr, mask=mask)


def _segment_track_for_stream():
    with camera_frame_lock:
        if 'lab' not in latest_processed_frames:
            return None
        lab = latest_processed_frames['lab']
        bgr = latest_processed_frames['bgr']
    small_bgr = cv2.resize(bgr, (lab.shape[1], lab.shape[0]), interpolation=cv2.INTER_NEAREST)
    track = build_track_mask(lab_mask(lab, 'white'))
    return cv2.bitwise_and(small_bgr, small_bgr, mask=track)


def _segment_hsv_for_stream(color):
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

        if mode == "original":
            with output_frame_lock:
                img = output_frame
            if img is None:
                img = _get_latest_bgr()
        elif mode == "lidar":
            with lidar_data_lock:
                scan = dict(latest_lidar_data)
            img = render_lidar_map(scan, CLOCKWISE_WALL_FOLLOWING)
        elif mode == "track":
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


@app.route("/get_lidar")
def get_lidar():
    with lidar_data_lock:
        scan = dict(latest_lidar_data)
    front_pts = [d for a, d in scan.items() if -5 <= a <= 5 and d > 0]
    left_pts  = [d for a, d in scan.items() if -90 <= a <= -40 and d > 0]
    right_pts = [d for a, d in scan.items() if  40 <= a <=  90 and d > 0]
    front = round(float(np.median(front_pts)), 1) if front_pts else None
    left  = round(float(np.median(left_pts)),  1) if left_pts  else None
    right = round(float(np.median(right_pts)), 1) if right_pts else None
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


@app.route("/video_hsv/<mode>")
def video_feed_hsv(mode):
    if mode not in STREAM_COLORS:
        return "Unknown stream", 404
    return Response(generate_hsv_stream(mode), mimetype="multipart/x-mixed-replace; boundary=frame")


# --- Main Execution Block ---
if __name__ == '__main__':
    print("--- Starting Robot Control System (LAB detection) ---")
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
