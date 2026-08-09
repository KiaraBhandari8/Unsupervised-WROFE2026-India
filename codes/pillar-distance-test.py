"""
pillar_distance_checker.py

Standalone diagnostic tool (separate from the main control loop):
  1. Detects the obstacle pillar via camera (LAB colour, same as your main script)
  2. Maps the pillar's pixel position to a LiDAR bearing angle
  3. Reports:
       - pillar distance (straight-line range to the pillar)
       - left clearance   (distance to whatever is just left of the pillar)
       - right clearance  (distance to whatever is just right of the pillar)
       - which side is "nearer" (i.e. more obstructed / less room)

Run directly on the Pi:  python3 pillar_distance_checker.py
Press Ctrl+C to stop.

NOTE: This uses the camera and LiDAR directly, so it can't run at the same
time as your main obs_main script (hardware ports/camera are single-owner).
Stop the main script first, then run this one to tune numbers.
"""

import time
import cv2
import numpy as np
from picamera2 import Picamera2
import libcamera

from lidar_steering4sept import LidarScanner

# ==================== CONFIG - match your robot ====================
CAMERA_RESOLUTION = (2304, 1296)
LAB_PROCESSING_WIDTH = CAMERA_RESOLUTION[0] // 3
LAB_PROCESSING_HEIGHT = CAMERA_RESOLUTION[1] // 3

CAMERA_HFOV_DEG = 66.0        # <-- SET THIS to your camera's real horizontal FOV
CAMERA_LIDAR_YAW_OFFSET_DEG = 0.0   # <-- fixed mounting offset if camera isn't
                                     #     boresight-aligned with LiDAR 0deg

SIDE_OFFSET_DEG = 12.0        # angular gap (left/right of pillar bearing) to sample
BEARING_WINDOW_DEG = 5        # sampling window for the pillar's own distance
SIDE_WINDOW_DEG = 6           # sampling window for the left/right clearance checks

LAB_MIN_CONTOUR_AREA = 900
LAB_MIN_WIDTH = 30

COLOR_PRESETS = {
    "red":   {"l_min": 0, "l_max": 255, "a_min": 146, "a_max": 255, "b_min": 100, "b_max": 255},
    "green": {"l_min": 0, "l_max": 255, "a_min": 0,   "a_max": 120, "b_min": 80,  "b_max": 200},
}
MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)

PRINT_EVERY_SEC = 0.3
# =====================================================================


def lab_mask(lab_frame, color):
    p = COLOR_PRESETS[color]
    mask = cv2.inRange(
        lab_frame,
        np.array([p["l_min"], p["a_min"], p["b_min"]]),
        np.array([p["l_max"], p["a_max"], p["b_max"]]),
    )
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)


def detect_pillars(lab_frame):
    dets = []
    for cls in ("red", "green"):
        mask = lab_mask(lab_frame, cls)
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in conts:
            ar = cv2.contourArea(c)
            if ar > LAB_MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                if w > LAB_MIN_WIDTH:
                    dets.append({
                        'class': cls, 'area': float(ar),
                        'bbox': [x, y, x + w, y + h], 'width': w,
                        'cx': x + w // 2,
                    })
    return dets


def pixel_to_bearing(cx, lab_w):
    norm = (cx / lab_w) - 0.5          # -0.5 (left edge) .. +0.5 (right edge)
    return norm * CAMERA_HFOV_DEG + CAMERA_LIDAR_YAW_OFFSET_DEG


def lidar_distance_at_bearing(scan_data, bearing_deg, window_deg):
    if not scan_data:
        return None
    lo, hi = bearing_deg - window_deg, bearing_deg + window_deg
    pts = [d for a, d in scan_data.items() if lo <= a <= hi and d > 0]
    return float(np.median(pts)) if pts else None


def get_pillar_geometry(target, scan_data, lab_w):
    bearing = pixel_to_bearing(target['cx'], lab_w)
    pillar_dist = lidar_distance_at_bearing(scan_data, bearing, BEARING_WINDOW_DEG)
    left_clear = lidar_distance_at_bearing(scan_data, bearing - SIDE_OFFSET_DEG, SIDE_WINDOW_DEG)
    right_clear = lidar_distance_at_bearing(scan_data, bearing + SIDE_OFFSET_DEG, SIDE_WINDOW_DEG)
    return bearing, pillar_dist, left_clear, right_clear


def fmt(v):
    return f"{v:.0f}mm" if v is not None else "N/A"


def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=False, hflip=False),
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    scanner = LidarScanner()
    scanner.connect()
    print("LiDAR + camera ready. Checking pillar clearance... (Ctrl+C to stop)\n")

    last_print = 0.0
    try:
        while True:
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if frame.shape[2] == 3 else frame
            small = cv2.resize(frame_bgr, (LAB_PROCESSING_WIDTH, LAB_PROCESSING_HEIGHT),
                                interpolation=cv2.INTER_AREA)
            lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)

            dets = detect_pillars(lab)
            scan_data = scanner.get_scan_data()

            now = time.time()
            if now - last_print >= PRINT_EVERY_SEC:
                last_print = now
                if not dets:
                    print("No pillar detected.")
                else:
                    # closest/lowest pillar in frame = the one to act on now
                    target = max(dets, key=lambda d: (d['bbox'][3], d['area']))
                    bearing, dist, lc, rc = get_pillar_geometry(target, scan_data, lab.shape[1])

                    if lc is not None and rc is not None:
                        nearer_side = "LEFT" if lc < rc else "RIGHT"
                    else:
                        nearer_side = "UNKNOWN"

                    print(f"[{target['class'].upper():5s}] "
                          f"bearing:{bearing:+5.1f}deg  "
                          f"pillar_dist:{fmt(dist):>7s}  "
                          f"left_clear:{fmt(lc):>7s}  "
                          f"right_clear:{fmt(rc):>7s}  "
                          f"more_obstructed_side:{nearer_side}")

            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        scanner.disconnect()
        picam2.stop()


if __name__ == "__main__":
    main()