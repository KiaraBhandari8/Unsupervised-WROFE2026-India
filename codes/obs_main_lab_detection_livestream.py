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


# --- Global Variables ---
output_frame = None
output_frame_lock = threading.Lock()

# Shared LiDAR data buffer and its lock
latest_lidar_data = {}

lidar_data_lock = threading.Lock()

# Shared buffer for the latest camera frame and its lock
latest_camera_frame = None
latest_processed_frames = {}
camera_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

app = Flask(__name__)
app.config["TEMPLATES_AUTO_RELOAD"] = True
app.jinja_env.auto_reload = True

shutdown_event = threading.Event()

SERVO_CENTER_ANGLE = 102

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

# --- DRIVABLE-AREA GATING ---
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


# ===================== DEBUG VISUALIZATION =====================
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


MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)

CLOCKWISE_WALL_FOLLOWING = True

# --- DEBUGGING AND UI ---
STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = False

STREAM_WIDTH = 480
STREAM_MAX_FPS = 12
STREAM_JPEG_QUALITY = 65


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


# --- Webcam fallback: capture + process directly (no lores stream) ---
def camera_acquisition_webcam(cap, stop_event, processing_size, lab_processing_size):
    global latest_processed_frames, camera_frame_lock
    print("Camera acquisition (webcam) thread started.")
    frame_seq = 0
    try:
        while not stop_event.is_set():
            ret, frame_bgr = cap.read()
            if not ret:
                time.sleep(0.03)
                continue
            capture_ts = time.time()
            frame_bgr = cv2.resize(frame_bgr, processing_size, interpolation=cv2.INTER_AREA)
            lab_source_frame = cv2.resize(
                frame_bgr, lab_processing_size, interpolation=cv2.INTER_AREA)
            lab_frame = cv2.cvtColor(lab_source_frame, cv2.COLOR_BGR2LAB)
            frame_seq += 1
            with camera_frame_lock:
                latest_processed_frames['bgr'] = frame_bgr
                latest_processed_frames['lab'] = lab_frame
                latest_processed_frames['seq'] = frame_seq
                latest_processed_frames['ts'] = capture_ts
            time.sleep(max(0.0, (1.0 / CAMERA_FRAMERATE) - (time.time() - capture_ts)))
    except Exception as e:
        print(f"Camera Acquisition (webcam) Error: {e}")
    finally:
        print("Camera acquisition (webcam) thread stopping.")


# --- Main Loop (detection + streaming only) ---
def main_loop(shutdown_event):
    global output_frame, output_frame_lock, latest_camera_frame, camera_frame_lock, camera_thread_stop_event

    picam2 = None
    cap = None
    processing_size = (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    lab_processing_size = (LAB_PROCESSING_WIDTH, LAB_PROCESSING_HEIGHT)

    try:
        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration(
            main={"size": CAMERA_RESOLUTION},
            lores={"size": processing_size},
            transform=libcamera.Transform(vflip=False, hflip=False),
            controls={"FrameRate": CAMERA_FRAMERATE},
            buffer_count=CAMERA_BUFFER_COUNT
        )
        picam2.configure(camera_config)
        picam2.start()
        print(f"[CAM] Pi Camera started: main {CAMERA_RESOLUTION}, "
              f"lores {processing_size} at {CAMERA_FRAMERATE} FPS.")
        time.sleep(1)

        camera_acquisition_thread = threading.Thread(
            target=camera_acquisition_thread_func,
            args=(picam2, camera_thread_stop_event, lab_processing_size)
        )
        camera_acquisition_thread.daemon = True
        camera_acquisition_thread.start()
        print("[CAM] Using Pi Camera with hardware lores stream.")
    except Exception as e:
        print(f"[CAM] Pi Camera failed ({e}); falling back to webcam.")
        picam2 = None
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[CAM] No webcam either!")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])
        print(f"[CAM] Webcam opened.")

        camera_acquisition_thread = threading.Thread(
            target=camera_acquisition_webcam,
            args=(cap, camera_thread_stop_event, processing_size, lab_processing_size)
        )
        camera_acquisition_thread.daemon = True
        camera_acquisition_thread.start()

    lidar_scanner = None
    lidar_acquisition_thread = None
    try:
        from lidar_steering4sept import LidarScanner
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        print("LiDAR initialized (visualization only).")
    except (IOError, Exception) as e:
        print(f"WARNING: Failed to initialize LiDAR: {e}.")
        lidar_scanner = None

    try:
        loop_counter = 0
        program_start_time = time.monotonic()
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

            detections = detect_lab_pillars(lab)

            if time.time() - print_timer >= 2.0:
                print_timer = time.time()
                elapsed_time = time.monotonic() - program_start_time
                print(f"[{loop_counter}] Time:{elapsed_time:.1f}s Det:{len(detections)} pillar(s)")

            if STREAM_VIDEO:
                processed_frame = frame_bgr.copy()
                scale_x = frame_bgr.shape[1] / lab.shape[1]
                scale_y = frame_bgr.shape[0] / lab.shape[0]
                wall_contours = extract_wall_contours(lab)
                draw_wall_contours(processed_frame, wall_contours, scale_x, scale_y)
                draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)
                with output_frame_lock:
                    output_frame = processed_frame

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down gracefully...")

    finally:
        print("Main loop ending. Cleaning up resources...")
        shutdown_event.set()
        camera_thread_stop_event.set()
        if camera_acquisition_thread and camera_acquisition_thread.is_alive():
            camera_acquisition_thread.join(timeout=2)
        if lidar_scanner:
            print("Disconnecting LiDAR...")
            lidar_scanner.disconnect()
        if picam2:
            try:
                picam2.stop()
            except:
                pass
        if cap:
            cap.release()
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
    print("--- Starting Live Stream with LAB Detection (no robot motion) ---")

    control_thread = threading.Thread(target=main_loop, args=(shutdown_event,))
    control_thread.start()
    print("Main loop thread started.")

    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False),
        daemon=True
    )
    flask_thread.start()

    try:
        hostname = os.uname()[1]
        print(f"Web server starting. Open http://{hostname}.local:5000 or http://<your_pi_ip>:5000")
    except AttributeError:
         import socket
         hostname = socket.gethostname()
         ip_address = socket.gethostbyname(hostname)
         print(f"Web server starting. Open http://{ip_address}:5000")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        shutdown_event.set()

    print("Waiting for main loop thread to stop...")
    control_thread.join(timeout=5)
    print("Main application exiting.")
