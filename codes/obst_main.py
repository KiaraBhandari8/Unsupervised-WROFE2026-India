"""
obs_main_combine_opt_12june.py  —  PROPOSED REPLACEMENT ARCHITECTURE
====================================================================

This is the LAB-segmentation version of your main robot file. YOLO is
completely removed. The vision pipeline now comes entirely from lab_detector.py.

WHAT IS REAL AND RUNNABLE HERE:
    - Picamera2 capture
    - LAB obstacle detection (via lab_detector)
    - Obstacle steering math (target_x, error, gain-scaled servo angle)
    - LiDAR integration via your unmodified lidar_steering4sept.py
    - Flask streaming skeleton (/video/original, /video/debug)
    - Main loop structure: detect -> primary -> vision_angle -> arbitration -> send

WHAT IS STUBBED — these depend on your ORIGINAL file and are marked
"=== NEED FROM ORIGINAL ===". Send me those sections and I'll wire them in:
    1. ESP32 serial protocol  (send_to_esp32 exact byte/string format)
    2. Blue-line counting + lap counting logic (you said keep as-is)
    3. The obstacle/lap state machine (states + transitions)
    4. arbitration() policy (priority vs blend — I propose a default below)
    5. Exact Picamera2 config you currently use (resolution/format)

Anywhere stubbed is safe: it either no-ops or prints, so the file imports and
the vision/LiDAR/Flask parts can be exercised before the rest is filled in.
"""

import time
import threading

import cv2
import numpy as np
from flask import Flask, Response

# --- your unmodified LiDAR module ---
from lidar_steering4sept import (
    LidarScanner,
    PIDController,
    calculate_steering_error,
)

# --- new LAB detector (replaces YOLO) ---
import lab_detector

# Picamera2 (Pi 5 + PiCamera3 Wide)
from picamera2 import Picamera2


# ===========================================================================
# CONFIG
# ===========================================================================
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480           # === NEED FROM ORIGINAL: confirm your capture height

SERVO_CENTER = 102
# Firmware self-test sweeps 60..120, so your linkage travel is ~that band.
# Keeping limits inside it avoids straining the servo. Widen only if your
# mechanism actually allows more. (=== CONFIRM your real left/right limits ===)
SERVO_MIN    = 60
SERVO_MAX    = 120

# Steering targets per spec.
RED_TARGET_X   = int(0.80 * FRAME_WIDTH)   # red  -> pass left  -> steer right
GREEN_TARGET_X = int(0.20 * FRAME_WIDTH)   # green-> pass right -> steer left

# Gain scaling vs obstacle width (closer obstacle => wider box => stronger steer).
# servo_angle = SERVO_CENTER + error * gain
# gain interpolates linearly between GAIN_MIN (far) and GAIN_MAX (near).
GAIN_WIDTH_NEAR = 220        # px width considered "very close"
GAIN_WIDTH_FAR  = 30         # px width considered "far"
GAIN_MIN        = 0.04
GAIN_MAX        = 0.18

# LiDAR
LIDAR_CLOCKWISE      = True          # === confirm: clockwise vs anticlockwise mode
LIDAR_TARGET_MM      = 750
LIDAR_SAFETY_MM      = 150
LIDAR_PID            = PIDController(Kp=0.05, Ki=0.0, Kd=0.01)  # tune to taste

# Speed
DRIVE_SPEED = 120            # placeholder — depends on ESP32 protocol
STOP_SPEED  = 0


# ===========================================================================
# CAMERA
# ===========================================================================
def init_camera():
    """
    Picamera2 init for PiCamera3 Wide.
    === NEED FROM ORIGINAL: your exact resolution/format/controls. ===
    This is a reasonable default that produces BGR frames for OpenCV.
    """
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)  # let AE/AWB settle
    return picam2


def capture_bgr(picam2):
    """Grab a frame and return it as BGR (OpenCV's expected order)."""
    rgb = picam2.capture_array()           # RGB888
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return bgr


# ===========================================================================
# VISION STEERING
# ===========================================================================
def _gain_for_width(width):
    """Linear interp of gain between far and near widths, clamped."""
    if width <= GAIN_WIDTH_FAR:
        return GAIN_MIN
    if width >= GAIN_WIDTH_NEAR:
        return GAIN_MAX
    frac = (width - GAIN_WIDTH_FAR) / float(GAIN_WIDTH_NEAR - GAIN_WIDTH_FAR)
    return GAIN_MIN + frac * (GAIN_MAX - GAIN_MIN)


def compute_obstacle_steering(obstacle):
    """
    Convert a primary obstacle detection into a servo angle.

    Returns (servo_angle, target_x) or (None, None) if no obstacle.

    Red  -> target_x = 0.80 * W  (push obstacle to the right of frame => steer right)
    Green-> target_x = 0.20 * W  (push obstacle to the left  of frame => steer left)
    error = target_x - obstacle_center_x
    servo_angle = SERVO_CENTER + error * gain(width)
    """
    if obstacle is None:
        return None, None

    if obstacle["class"] == "red":
        target_x = RED_TARGET_X
    elif obstacle["class"] == "green":
        target_x = GREEN_TARGET_X
    else:
        return None, None

    error = target_x - obstacle["cx"]
    gain  = _gain_for_width(obstacle["width"])
    angle = SERVO_CENTER + error * gain
    angle = int(max(SERVO_MIN, min(SERVO_MAX, angle)))
    return angle, target_x


# ===========================================================================
# LIDAR STEERING
# ===========================================================================
def compute_lidar_steering(scan_data):
    """
    Returns (servo_angle, emergency_stop_bool).
    Uses your unmodified calculate_steering_error().
    """
    if not scan_data:
        return SERVO_CENTER, False

    err = calculate_steering_error(
        scan_data,
        target_distance_mm=LIDAR_TARGET_MM,
        safety_distance_mm=LIDAR_SAFETY_MM,
        clockwise=LIDAR_CLOCKWISE,
    )

    if err == 9999.0:
        return SERVO_CENTER, True   # emergency stop

    correction = LIDAR_PID.update(err)
    angle = int(max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + correction)))
    return angle, False


# ===========================================================================
# ARBITRATION
# ===========================================================================
def arbitration(lidar_angle, vision_angle, lidar_estop):
    """
    Decide the final servo angle from LiDAR + vision.

    === NEED FROM ORIGINAL: your real arbitration policy. ===
    PROPOSED DEFAULT (safe, simple priority):
        1. LiDAR emergency stop overrides everything.
        2. If an obstacle is seen (vision_angle is not None), vision wins,
           because the WRO obstacle challenge is about passing pillars.
        3. Otherwise fall back to LiDAR wall-following.

    Swap this for a weighted blend if that's what your original did.
    """
    if lidar_estop:
        return SERVO_CENTER, True            # caller will command STOP_SPEED
    if vision_angle is not None:
        return vision_angle, False
    return lidar_angle, False


# ===========================================================================
# ESP32 COMMUNICATION
# ===========================================================================
# Protocol expected by the ESP32 firmware:
#     "STR:<angle>,SPD:<speed>\n"
#   angle: 0-180  (firmware maps to servo pulse; 90 is its nominal center,
#                  but we use SERVO_CENTER=102 as the mechanical center)
#   speed: 0-255  (0 = stop; firmware drives AIN1=LOW, AIN2=HIGH otherwise)
#
# The firmware reads from Serial2 (GPIO16/17) OR USB Serial. Over USB the Pi
# talks to the ESP32 on a /dev/tty* port. Confirm which port your Pi enumerates
# (commonly /dev/ttyUSB0 or /dev/ttyACM0). Set ESP32_PORT accordingly.
import serial

ESP32_PORT = "/dev/ttyACM0"   # === CONFIRM: /dev/ttyACM0 vs /dev/ttyUSB0 ===
ESP32_BAUD = 115200           # matches firmware Serial.begin(115200)

try:
    _esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.1)
    time.sleep(2.0)  # allow ESP32 reset + servo self-test to finish
    print(f"[ESP32] connected on {ESP32_PORT} @ {ESP32_BAUD}")
except Exception as e:
    _esp32 = None
    print(f"[ESP32] WARNING: could not open {ESP32_PORT}: {e}. "
          f"Commands will be printed only.")


def send_to_esp32(servo_angle, speed):
    """
    Send a steering+speed command in the firmware's STR/SPD format.
    angle is clamped to 0-180, speed to 0-255, to match firmware expectations.
    """
    angle = int(max(0, min(180, servo_angle)))
    spd   = int(max(0, min(255, speed)))
    msg = f"STR:{angle},SPD:{spd}\n"
    if _esp32 is not None:
        try:
            _esp32.write(msg.encode("ascii"))
        except Exception as e:
            print(f"[ESP32] write failed: {e}")
    else:
        print(f"[ESP32-PRINT] {msg.strip()}")


# ===========================================================================
# BLUE-LINE / LAP COUNTING  — STUB (keep your existing logic)
# ===========================================================================
# === NEED FROM ORIGINAL ===
# You said: do NOT remove blue-line counting, keep current logic (HSV is fine).
# Paste your original counting function and I'll drop it in verbatim.
def update_blue_line_count(frame):
    """STUB: returns current lap state. Replace with your existing logic."""
    # === NEED FROM ORIGINAL: blue mask + crossing detection + lap increment ===
    return {"blue_crossings": 0, "laps": 0}


# ===========================================================================
# OBSTACLE / LAP STATE MACHINE  — STUB
# ===========================================================================
# === NEED FROM ORIGINAL ===
# Your original had an "obstacle state machine" (states + transitions) plus
# clockwise/anticlockwise mode handling and a stop-after-N-laps rule.
# Paste it and I'll integrate. For now this is a pass-through.
class RobotState:
    def __init__(self):
        self.running = True
        self.laps = 0
        # === NEED FROM ORIGINAL: real state fields ===

    def update(self, detections, lap_info):
        # === NEED FROM ORIGINAL: transitions, lap-stop, mode switches ===
        self.laps = lap_info.get("laps", self.laps)


# ===========================================================================
# SHARED FRAME BUFFERS (for Flask streaming)
# Four views mirror hsv_webpage.py: original, segmented, mask, debug.
# ===========================================================================
_frames = {"original": None, "segmented": None, "mask": None, "debug": None}
_frame_lock = threading.Lock()


def _set_frames(original=None, segmented=None, mask=None, debug=None):
    with _frame_lock:
        if original  is not None: _frames["original"]  = original
        if segmented is not None: _frames["segmented"] = segmented
        if mask      is not None: _frames["mask"]      = mask
        if debug     is not None: _frames["debug"]     = debug


def _encode(frame):
    if frame is None:
        return None
    ok, buf = cv2.imencode(".jpg", frame)
    return buf.tobytes() if ok else None


# ===========================================================================
# FLASK  (keep /video/original and /video/debug)
# ===========================================================================
app = Flask(__name__)


def _mjpeg_generator(which):
    while True:
        with _frame_lock:
            frame = _frames.get(which)
        jpg = _encode(frame)
        if jpg is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.03)


@app.route("/")
def index():
    return (
        "<h2>WRO Robot — live views</h2>"
        "<ul>"
        "<li><a href='/video/original'>original</a></li>"
        "<li><a href='/video/segmented'>segmented</a></li>"
        "<li><a href='/video/mask'>mask</a></li>"
        "<li><a href='/video/debug'>debug (boxes + steering target)</a></li>"
        "</ul>"
        "<p><a href='/reload_presets'>reload presets</a></p>"
    )


@app.route("/video/<mode>")
def video(mode):
    if mode not in _frames:
        return f"unknown mode '{mode}'", 404
    return Response(_mjpeg_generator(mode),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/reload_presets")
def reload_presets_route():
    """Hot-reload LAB thresholds after re-tuning in hsv_webpage.py."""
    keys = list(lab_detector.reload_presets().keys())
    return {"reloaded": keys}


def start_flask():
    # threaded so it doesn't block the control loop
    app.run(host="0.0.0.0", port=5000, threaded=True,
            debug=False, use_reloader=False)


# ===========================================================================
# MAIN CONTROL LOOP
# ===========================================================================
def main():
    picam2 = init_camera()

    lidar = LidarScanner()      # default port /dev/ttyUSB0, baud 230400
    lidar.connect()

    state = RobotState()

    # Flask in background thread.
    threading.Thread(target=start_flask, daemon=True).start()

    try:
        while state.running:
            # --- 1. capture ---
            frame = capture_bgr(picam2)

            # --- 2. vision: LAB detection ---
            detections = lab_detector.detect_lab_obstacles(frame)
            obstacle   = lab_detector.get_primary_obstacle(detections)
            vision_angle, target_x = compute_obstacle_steering(obstacle)

            # --- 3. blue-line / lap counting (existing logic) ---
            lap_info = update_blue_line_count(frame)
            state.update(detections, lap_info)

            # --- 4. lidar ---
            scan = lidar.get_scan_data()
            lidar_angle, estop = compute_lidar_steering(scan)

            # --- 5. arbitration ---
            final_angle, stop_now = arbitration(lidar_angle, vision_angle, estop)
            speed = STOP_SPEED if stop_now else DRIVE_SPEED

            # --- 6. command ---
            send_to_esp32(final_angle, speed)

            # --- 7. debug + auxiliary streams (mirror hsv_webpage views) ---
            debug = lab_detector.draw_debug(frame, detections,
                                            primary=obstacle, target_x=target_x)
            mask_bgr, segmented = lab_detector.build_views(frame, detections)
            _set_frames(original=frame, segmented=segmented,
                        mask=mask_bgr, debug=debug)

    except KeyboardInterrupt:
        print("Stopping (Ctrl-C).")
    finally:
        send_to_esp32(SERVO_CENTER, STOP_SPEED)
        lidar.disconnect()
        try:
            picam2.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()

