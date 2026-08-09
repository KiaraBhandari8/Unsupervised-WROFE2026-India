"""
lab_avoidance_debug.py
===========================
Standalone debug harness for CAMERA-ONLY (LAB colour space) obstacle
avoidance -- isolates ONLY the manage_color_avoidance() state machine
from your LAB-detection main script:

    IDLE --(pillar wide/close enough)--> TURNING
    TURNING --(pillar has visually "passed" the frame)--> REALIGN
    REALIGN --(REALIGN_DURATION_SEC elapsed)--> RECENTER (if gyro live)
                                              --> IDLE     (if no gyro)
    RECENTER --(heading back within tolerance OR timeout)--> IDLE

This is a DIFFERENT design from your other (HSV-based) avoidance file:
  - Detection: LAB colour thresholds instead of HSV, plus a
    build_track_mask() "is this obstacle actually standing on the
    drivable white floor" gate (_on_drivable_area), which the HSV
    pipeline doesn't have.
  - Exit condition: geometric "has the obstacle's centroid crossed
    AVOID_PASS_CX_FRAC of the frame width", not an area/miss-streak
    coast. This does NOT suffer from the ROI-clipping failure mode the
    other pipeline can hit when tilted, since detect_lab_pillars() does
    NOT mask the color mask by an ROI rectangle before findContours --
    only a top-of-frame cutoff (LAB_ROI_TOP_FRAC) and the drivable-floor
    check apply.
  - Post-clear: instead of a fixed "drive straight for N seconds", this
    does REALIGN (brief straight coast) then, if gyro is live, RECENTER
    -- actively steers back toward the yaw heading recorded right before
    avoidance started, so the robot re-settles on the same line it was
    on before the pillar, not just "however straight happened to end up".

Everything else -- LiDAR wall-following, corner state machine, blue-line
counting, front emergency stop, side-panic avoidance -- is stripped out
so you can watch ONLY the camera+gyro avoidance behave in isolation.

Run modes
---------
LIVE = False (default) : reads camera (+ gyro/LiDAR if enabled), runs
                          detection + state machine, prints/streams
                          everything, but NEVER writes steering/speed
                          to the ESP32. Motors will not move.
LIVE = True             : actually sends STR/SPD packets, so the
                           chassis drives continuously (cruise speed
                           when clear) and reacts to obstacles.

READ_GYRO = True/False  : independent of LIVE -- if True, opens serial
                           JUST to read "YAW:" lines and sends one
                           RST_YAW at startup, so you can test the
                           REALIGN->RECENTER handoff even with LIVE=False
                           (watch the numbers, wheels don't move). If
                           False, avoidance never enters RECENTER -- it
                           goes straight from REALIGN back to IDLE,
                           matching pre-gyro behavior.

READ_LIDAR = True/False : independent of the above -- only affects the
                           front_dist term in _avoid_servo_angle (steers
                           harder the closer the pillar is by LiDAR
                           range, on top of the vision-only distance
                           proxy). False -> front_dist is always None,
                           so steering magnitude falls back to pure
                           colour-detection-based scaling (base gain,
                           side_factor only). Needs lidar_steering4sept.py
                           on the path; if the import fails this just
                           logs a warning and continues with LiDAR off.

Watching it
-----------
Flask server on port 5002 (different from your main script's 5000 and
the other debug harness's 5001, so all three could run together).
Open http://<pi-ip>:5002/ for the live overlay: detected pillar boxes
+ centroids + target-line lines to the green/red reference points,
state machine phase, servo/speed, and FPS.

Ctrl+C always stops the motors and exits cleanly.
"""

import sys
import time
import json
import threading
import signal

import cv2
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, Response

try:
    import serial
except ImportError:
    serial = None

# =====================================================================
# EDIT THESE TO CONFIGURE THE DEBUG RUN
# =====================================================================
LIVE = True                  # False = dry run, motors never move. True = robot drives.
READ_GYRO = True              # True = read YAW: lines, enables RECENTER phase testing
READ_LIDAR = False            # True = read LiDAR for front_dist-based steering scaling

STREAM_VIDEO = True
FLASK_PORT = 5002
LOG_EVERY_N_FRAMES = 5

# --- Copied VERBATIM from your main script so tuning here stays
#     directly transferable. Edit here to experiment, then copy back
#     into the main script once you're happy. ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

SERVO_CENTER_ANGLE = 102
SERVO_MIN = 77
SERVO_MAX = 127

ROBOT_MANEUVER_SPEED = 0.8    # fraction 0-1, sent as SPD:int(speed*255)
ROBOT_CRUISE_SPEED = 0.85

LAB_MIN_CONTOUR_AREA = 900
LAB_MIN_WIDTH = 30
LAB_ROI_TOP_FRAC = 0.35       # <-- TUNABLE: ignore anything whose bottom edge is above this
                               #     fraction of frame height (i.e. too far away / too small yet)

COLOR_STEER_MAGNITUDE = 22        # <-- TUNABLE: base steer magnitude (deg) for green
COLOR_STEER_MAGNITUDE_RED = 16    # <-- TUNABLE: base steer magnitude (deg) for red
RED_AVOID_MAX_MAG_DEG = 15.0      # <-- TUNABLE: hard cap on red's steer magnitude after scaling

AVOID_STANDOFF_MM = 250           # <-- TUNABLE: with AVOID_CLEARANCE_MM, sets the LiDAR-distance
AVOID_CLEARANCE_MM = 450          #     entry trigger (front_dist <= STANDOFF + CLEARANCE)
AVOID_PASS_CX_FRAC = 0.78         # <-- TUNABLE: how far the obstacle centroid must cross toward
                                   #     the far edge of frame before it's considered "passed"
REALIGN_DURATION_SEC = 1          # <-- TUNABLE: brief straight coast right after "passed"
AVOID_WIDTH_TRIGGER = 30          # <-- TUNABLE: pixel-width entry trigger (vision-only, no LiDAR needed)
AVOID_DIST_GAIN_REF_MM = 350.0    # <-- TUNABLE: reference distance for the LiDAR distance-gain term

# --- Gyro lane re-centering (after each obstacle pass) ---
GYRO_ENABLED = True                    # master switch -- also gated by READ_GYRO at runtime below
GYRO_RECENTER_DURATION_SEC = 1.3       # <-- TUNABLE: max time to spend actively re-centering
GYRO_RECENTER_TOL_DEG = 4.0            # <-- TUNABLE: exit RECENTER once within this many degrees
GYRO_RECENTER_KP = 1.2                 # <-- TUNABLE: proportional gain, yaw error -> servo offset
GYRO_RECENTER_MAX_OFFSET = 22          # <-- TUNABLE: clamp on the recenter servo offset
GYRO_RECENTER_SIGN = +1                # <-- flip to -1 if recenter steers the wrong way on your chassis

TRACK_MASK_CLOSE_PX = 25
TRACK_MIN_AREA_FRAC = 0.02
TRACK_BASE_DILATE_PX = 12

CAMERA_RESOLUTION = (2304, 1296)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 2
PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 2
LAB_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3
LAB_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3

PRESETS_FILE = "/home/pi8/wrofe2025/vision_presets.json"
DEFAULT_PRESETS = {
    "red":    {"l_min": 0,   "l_max": 255, "a_min": 146, "a_max": 255, "b_min": 100, "b_max": 255},
    "green":  {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 120, "b_min": 80,  "b_max": 200},
    "white":  {"l_min": 100, "l_max": 255, "a_min": 0,   "a_max": 255, "b_min": 0,   "b_max": 255},
}
MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)


def load_presets():
    presets = {k: dict(v) for k, v in DEFAULT_PRESETS.items()}
    try:
        with open(PRESETS_FILE) as f:
            loaded = json.load(f)
        for color in DEFAULT_PRESETS:
            if color in loaded:
                presets[color] = loaded[color]
        print(f"[LAB] Loaded presets from {PRESETS_FILE}")
    except Exception as e:
        print(f"[LAB] Could not load {PRESETS_FILE} ({e}); using defaults.")
    return presets


COLOR_PRESETS = load_presets()

# --- Shared state ---
shutdown_event = threading.Event()
camera_lock = threading.Lock()
latest_frames = {}
output_frame = None
output_frame_lock = threading.Lock()
esp_ser = None
picam2 = None
current_yaw = 0.0
latest_lidar_data = {}
lidar_data_lock = threading.Lock()

app = Flask(__name__)


# ===================== DETECTION (ported verbatim) =====================
def lab_mask(lab_frame, color):
    p = COLOR_PRESETS[color]
    mask = cv2.inRange(
        lab_frame,
        np.array([p["l_min"], p["a_min"], p["b_min"]]),
        np.array([p["l_max"], p["a_max"], p["b_max"]]),
    )
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)


def build_track_mask(white_mask):
    h, w = white_mask.shape[:2]
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TRACK_MASK_CLOSE_PX, TRACK_MASK_CLOSE_PX))
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
        if (top + height) >= (h - 2) and area > best_area:
            best_label, best_area = lbl, area
    if best_label == 0:
        return np.zeros_like(white_mask)
    track = np.where(labels == best_label, 255, 0).astype(np.uint8)
    if TRACK_BASE_DILATE_PX > 0:
        dk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TRACK_BASE_DILATE_PX, TRACK_BASE_DILATE_PX))
        track = cv2.dilate(track, dk, iterations=1)
    return track


def _on_drivable_area(track_mask, bbox):
    x1, y1, x2, y2 = bbox
    h, w = track_mask.shape[:2]
    if cv2.countNonZero(track_mask) == 0:
        return False
    by1, by2 = min(y2, h), min(y2 + 24, h)
    bx1, bx2 = max(0, x1), min(x2, w)
    if (by2 - by1) < 2 or (bx2 - bx1) < 2:
        cx = int(np.clip((x1 + x2) // 2, 0, w - 1))
        cy = int(np.clip(y2 - 1, 0, h - 1))
        return track_mask[cy, cx] != 0
    band = track_mask[by1:by2, bx1:bx2]
    return (cv2.countNonZero(band) / band.size) >= 0.30


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
                    dets.append({'class': cls, 'area': float(ar), 'bbox': bbox,
                                 'width': w, 'cx': x + w // 2})
    return dets


# ===================== STATE MACHINE (ported verbatim) =====================
avoid_state = "IDLE"
avoid_color = None
realign_end_time = 0.0
pre_avoid_yaw = None
recenter_end_time = 0.0


def _gyro_recenter_angle(cur_yaw, target_yaw):
    err = target_yaw - cur_yaw
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


def manage_color_avoidance(dets, scan_data, lab_w, cur_yaw=None):
    global avoid_state, avoid_color, realign_end_time
    global pre_avoid_yaw, recenter_end_time
    now = time.time()
    have_gyro = GYRO_ENABLED and READ_GYRO and cur_yaw is not None

    objs = [d for d in dets if d['class'] in ('red', 'green')]
    target = max(objs, key=lambda d: (d['bbox'][3], d['area'])) if objs else None
    front_dist = _front_distance_mm(scan_data)

    if avoid_state == "REALIGN":
        if now < realign_end_time:
            return True, SERVO_CENTER_ANGLE, f"realign_{avoid_color}"
        if have_gyro and pre_avoid_yaw is not None:
            avoid_state = "RECENTER"
            recenter_end_time = now + GYRO_RECENTER_DURATION_SEC
            return True, _gyro_recenter_angle(cur_yaw, pre_avoid_yaw), f"recenter_{avoid_color}"
        avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None

    if avoid_state == "RECENTER":
        if not have_gyro or pre_avoid_yaw is None:
            avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None
        else:
            heading_err = abs(pre_avoid_yaw - cur_yaw)
            if now >= recenter_end_time or heading_err <= GYRO_RECENTER_TOL_DEG:
                avoid_state, avoid_color, pre_avoid_yaw = "IDLE", None, None
            else:
                return True, _gyro_recenter_angle(cur_yaw, pre_avoid_yaw), f"recenter_{avoid_color}"

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
        if width_trigger or dist_trigger:
            avoid_color = target['class']
            avoid_state = "TURNING"
            pre_avoid_yaw = cur_yaw if have_gyro else None
            return True, _avoid_servo_angle(avoid_color, front_dist, target['cx'], lab_w), f"avoid_{avoid_color}"

    return False, None, "none"


# ===================== VISUALIZATION =====================
def draw_obstacle_overlay(frame, dets, scale_x=1.0, scale_y=1.0):
    h, w = frame.shape[:2]
    gx, gy = int(0.25 * w), int(0.93 * h)
    rx, ry = int(0.75 * w), int(0.93 * h)

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
        cv2.putText(frame, f"{d['class']} w={d['width']}", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, clr, 1)

    greens = [d for d in dets if d['class'] == 'green']
    reds = [d for d in dets if d['class'] == 'red']
    g = max(greens, key=lambda d: (d['bbox'][3], d['area'])) if greens else None
    r = max(reds, key=lambda d: (d['bbox'][3], d['area'])) if reds else None
    if g is not None:
        cv2.line(frame, (gx, gy), center(g), (0, 255, 0), 2, cv2.LINE_AA)
    if r is not None:
        cv2.line(frame, (rx, ry), center(r), (0, 0, 255), 2, cv2.LINE_AA)
    for (px, py), clr in (((gx, gy), (0, 255, 0)), ((rx, ry), (0, 0, 255))):
        cv2.circle(frame, (px, py), 7, clr, -1)
        cv2.circle(frame, (px, py), 9, (255, 255, 255), 2)

    # AVOID_PASS_CX_FRAC reference lines, so you can see the "passed" threshold live
    pass_x_right = int(AVOID_PASS_CX_FRAC * w)
    pass_x_left = int((1.0 - AVOID_PASS_CX_FRAC) * w)
    cv2.line(frame, (pass_x_right, 0), (pass_x_right, h), (0, 255, 0), 1, cv2.LINE_AA)
    cv2.line(frame, (pass_x_left, 0), (pass_x_left, h), (0, 0, 255), 1, cv2.LINE_AA)


# ===================== THREADS =====================
def camera_thread_func():
    global latest_frames
    print("[SYSTEM] Camera thread active.")
    try:
        while not shutdown_event.is_set():
            yuv420 = picam2.capture_array("lores")
            frame_bgr = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)
            lab_source = cv2.resize(frame_bgr, (LAB_PROCESSING_WIDTH, LAB_PROCESSING_HEIGHT),
                                     interpolation=cv2.INTER_AREA)
            lab_frame = cv2.cvtColor(lab_source, cv2.COLOR_BGR2LAB)
            with camera_lock:
                latest_frames['bgr'] = frame_bgr
                latest_frames['lab'] = lab_frame
            time.sleep(0.001)
    except Exception as e:
        if not shutdown_event.is_set():
            print(f"[CRITICAL] Camera thread crashed: {e}")


def gyro_thread_func():
    global current_yaw
    if esp_ser is None:
        return
    print("[GYRO] Gyro read thread active.")
    while not shutdown_event.is_set():
        try:
            raw = esp_ser.readline().decode('utf-8', errors='ignore').strip()
            if raw.startswith("YAW:"):
                current_yaw = float(raw.split(":", 1)[1])
        except Exception:
            time.sleep(0.02)


def lidar_thread_func():
    global latest_lidar_data
    try:
        from lidar_steering4sept import LidarScanner
    except ImportError as e:
        print(f"[WARN] LiDAR module not found ({e}); READ_LIDAR effectively off.")
        return
    try:
        scanner = LidarScanner()
        scanner.connect()
        print("[LIDAR] Scanner connected.")
        while not shutdown_event.is_set():
            data = scanner.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data
            time.sleep(0.01)
    except Exception as e:
        print(f"[WARN] LiDAR thread failed: {e}")


def send_packet(angle, speed_frac, live):
    if not live:
        return
    if esp_ser and esp_ser.is_open:
        try:
            packet = f"STR:{int(round(angle))},SPD:{int(speed_frac * 255)}\n"
            esp_ser.write(packet.encode('utf-8'))
        except Exception as e:
            print(f"[SERIAL] write failed: {e}")


def stop_and_exit(*_):
    print("\n[SHUTDOWN] Stopping motors and disconnecting...")
    shutdown_event.set()
    global esp_ser, picam2
    if esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
        except Exception:
            pass
    if picam2:
        try:
            picam2.stop()
        except Exception:
            pass
    print("[SHUTDOWN] Done.")
    sys.exit(0)


def generate_frames():
    global output_frame
    while not shutdown_event.is_set():
        if not STREAM_VIDEO:
            time.sleep(0.2)
            continue
        local_frame = None
        with output_frame_lock:
            if output_frame is not None:
                local_frame = output_frame.copy()
        if local_frame is None:
            time.sleep(0.03)
            continue
        flag, encoded = cv2.imencode(".jpg", local_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not flag:
            continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded) + b'\r\n')
        time.sleep(0.03)


@app.route("/")
def index():
    return "<h3>LAB Avoidance Debug Stream</h3><img src='/video_feed' width='100%'/>"


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


def control_loop():
    global esp_ser, picam2, output_frame

    need_serial = LIVE or READ_GYRO
    if need_serial:
        if serial is None:
            print("[FATAL] pyserial not installed but LIVE or READ_GYRO is True.")
            sys.exit(1)
        try:
            esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
            print(f"[INFO] Serial link open. Motors {'ARE' if LIVE else 'are NOT'} live. "
                  f"Gyro {'ON' if READ_GYRO else 'OFF'}.")
        except Exception as e:
            print(f"[FATAL] Could not open serial port {PI_TO_ESP_PORT}: {e}")
            sys.exit(1)
        if READ_GYRO:
            try:
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                time.sleep(0.2)
                while esp_ser.in_waiting > 0:
                    esp_ser.readline()
            except Exception as e:
                print(f"[WARN] Startup gyro reset failed: {e}")
            print("[INFO] Startup gyro reset complete.")
    else:
        print("[INFO] LIVE=False, READ_GYRO=False -- no serial port opened at all.")

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
    time.sleep(1)

    cam_thread = threading.Thread(target=camera_thread_func)
    cam_thread.daemon = True
    cam_thread.start()

    if READ_GYRO:
        g_thread = threading.Thread(target=gyro_thread_func)
        g_thread.daemon = True
        g_thread.start()

    if READ_LIDAR:
        l_thread = threading.Thread(target=lidar_thread_func)
        l_thread.daemon = True
        l_thread.start()

    print(f"[INFO] COLOR_STEER_MAGNITUDE(green/red)={COLOR_STEER_MAGNITUDE}/{COLOR_STEER_MAGNITUDE_RED} "
          f"AVOID_PASS_CX_FRAC={AVOID_PASS_CX_FRAC}")
    print(f"[INFO] AVOID_WIDTH_TRIGGER={AVOID_WIDTH_TRIGGER} REALIGN_DURATION_SEC={REALIGN_DURATION_SEC} "
          f"GYRO_RECENTER_DURATION_SEC={GYRO_RECENTER_DURATION_SEC}")
    if STREAM_VIDEO:
        print(f"[INFO] Visual stream at http://<this-pi-ip>:{FLASK_PORT}/")
    print("[INFO] Press Ctrl+C to stop.\n")

    frame_count = 0
    print_timer = 0.0

    while not shutdown_event.is_set():
        loop_start = time.monotonic()

        with camera_lock:
            if 'lab' not in latest_frames:
                time.sleep(0.01)
                continue
            frame_bgr = latest_frames['bgr'].copy()
            lab = latest_frames['lab'].copy()

        scan_data = None
        if READ_LIDAR:
            with lidar_data_lock:
                scan_data = dict(latest_lidar_data)

        cur_yaw = current_yaw if READ_GYRO else None

        detections = detect_lab_pillars(lab)
        avoid_engaged, avoid_angle_val, logic_label = manage_color_avoidance(
            detections, scan_data, lab.shape[1], cur_yaw)

        if avoid_engaged:
            target_servo_angle = avoid_angle_val
            speed_frac = ROBOT_MANEUVER_SPEED
            display_text = f"MODE: {logic_label} | Steer: {target_servo_angle:.1f}deg"
        else:
            target_servo_angle = SERVO_CENTER_ANGLE
            speed_frac = ROBOT_CRUISE_SPEED
            display_text = "MODE: Straight (cruise, no obstacle)"

        final_angle = int(round(np.clip(target_servo_angle, SERVO_MIN, SERVO_MAX)))
        send_packet(final_angle, speed_frac, LIVE)

        loop_duration = time.monotonic() - loop_start
        fps = 1.0 / loop_duration if loop_duration > 0 else 0

        processed_frame = frame_bgr.copy()
        scale_x = frame_bgr.shape[1] / lab.shape[1]
        scale_y = frame_bgr.shape[0] / lab.shape[0]
        draw_obstacle_overlay(processed_frame, detections, scale_x, scale_y)
        cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(processed_frame, f"servo={final_angle} speed={speed_frac:.2f} live={LIVE} state={avoid_state}",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        yaw_txt = f"yaw={cur_yaw:+.1f}" if cur_yaw is not None else "yaw=N/A"
        cv2.putText(processed_frame, f"{yaw_txt} FPS={int(fps)} dets={len(detections)}",
                    (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if STREAM_VIDEO:
            with output_frame_lock:
                output_frame = processed_frame.copy()

        if frame_count % LOG_EVERY_N_FRAMES == 0:
            print(f"[VISION] {display_text} | state={avoid_state} logic={logic_label} "
                  f"servo={final_angle} speed={speed_frac:.2f} dets={len(detections)} live={LIVE}")

        frame_count += 1
        time.sleep(0.02)

    stop_and_exit()


if __name__ == "__main__":
    cv2.setNumThreads(2)
    signal.signal(signal.SIGINT, stop_and_exit)
    signal.signal(signal.SIGQUIT, stop_and_exit)
    control_thread = threading.Thread(target=control_loop)
    control_thread.daemon = True
    control_thread.start()
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False, threaded=True, use_reloader=False)