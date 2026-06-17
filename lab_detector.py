"""
lab_detector.py
================
LAB color-segmentation obstacle detector. Drop-in replacement for the old
YOLO-based detection. NO neural network, NO NCNN, NO Ultralytics.

Pipeline:
    BGR frame -> LAB -> cv2.inRange(per preset) -> contours -> bboxes -> detections

Thresholds are loaded from vision_presets.json (written by your hsv_webpage.py
tuning dashboard). Only "red" and "green" are treated as obstacles here; any
other preset (e.g. "blue") is ignored by this module so it does NOT interfere
with your existing blue-line counting.

Detection dict format:
    {
        "class":  "red" | "green",
        "bbox":   [x1, y1, x2, y2],
        "cx":     int,    # bounding-box center x
        "cy":     int,    # bounding-box center y
        "width":  int,
        "height": int,
        "area":   int,    # contour area (not bbox area)
    }
"""

import json
import os
import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PRESETS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "vision_presets.json")

# Only these preset names are treated as steerable obstacles.
OBSTACLE_CLASSES = ("red", "green")

# Reject tiny noise blobs. Tune to your arena lighting / camera distance.
MIN_CONTOUR_AREA = 300      # px^2
MIN_BOX_WIDTH    = 12       # px
MIN_BOX_HEIGHT   = 20       # px

# Morphology kernel to clean up the mask (remove specks, close gaps).
_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# Debug-draw colors (BGR).
_DRAW_COLORS = {
    "red":   (0, 0, 255),
    "green": (0, 255, 0),
}


# ---------------------------------------------------------------------------
# Preset loading
# ---------------------------------------------------------------------------
def load_presets(path=PRESETS_PATH):
    """
    Load LAB thresholds from vision_presets.json.

    Returns a dict mapping class name -> (lower_lab, upper_lab) numpy arrays
    ready for cv2.inRange(). Only classes in OBSTACLE_CLASSES are returned.

    Expected JSON shape (per your dashboard):
        {
          "red":   {"l_min":0,"l_max":255,"a_min":140,"a_max":255,"b_min":100,"b_max":255},
          "green": {"l_min":0,"l_max":255,"a_min":0,"a_max":120,"b_min":80,"b_max":200},
          ...
        }
    """
    try:
        with open(path, "r") as f:
            raw = json.load(f)
    except FileNotFoundError:
        print(f"[lab_detector] WARNING: presets file not found at {path}. "
              f"Returning empty preset set.")
        return {}
    except json.JSONDecodeError as e:
        print(f"[lab_detector] WARNING: could not parse {path}: {e}. "
              f"Returning empty preset set.")
        return {}

    # Some dashboards wrap everything under a top-level "presets" key.
    if "presets" in raw and isinstance(raw["presets"], dict):
        raw = raw["presets"]

    presets = {}
    for cls in OBSTACLE_CLASSES:
        if cls not in raw:
            continue
        p = raw[cls]
        try:
            lower = np.array([p["l_min"], p["a_min"], p["b_min"]], dtype=np.uint8)
            upper = np.array([p["l_max"], p["a_max"], p["b_max"]], dtype=np.uint8)
        except KeyError as e:
            print(f"[lab_detector] WARNING: preset '{cls}' missing key {e}; skipping.")
            continue
        presets[cls] = (lower, upper)

    if not presets:
        print("[lab_detector] WARNING: no usable red/green presets loaded.")
    return presets


# Module-level cache so we don't re-read the file every frame.
# Call reload_presets() (e.g. from a Flask route) after re-tuning.
_PRESETS = load_presets()


def reload_presets(path=PRESETS_PATH):
    """Re-read presets from disk. Call after re-tuning in the dashboard."""
    global _PRESETS
    _PRESETS = load_presets(path)
    return _PRESETS


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------
def _mask_for_class(lab_frame, lower, upper):
    """Build a cleaned binary mask for one LAB threshold range."""
    mask = cv2.inRange(lab_frame, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
    return mask


def detect_lab_obstacles(frame, presets=None):
    """
    Run LAB segmentation on a BGR frame and return a list of detection dicts.

    Args:
        frame:   BGR image (numpy array) from Picamera2 / OpenCV.
        presets: optional override; defaults to the module-cached presets.

    Returns:
        list[dict] in the documented detection format. May be empty.
    """
    if frame is None:
        return []

    if presets is None:
        presets = _PRESETS

    detections = []
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    for cls, (lower, upper) in presets.items():
        mask = _mask_for_class(lab, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < MIN_BOX_WIDTH or h < MIN_BOX_HEIGHT:
                continue
            detections.append({
                "class":  cls,
                "bbox":   [int(x), int(y), int(x + w), int(y + h)],
                "cx":     int(x + w / 2),
                "cy":     int(y + h / 2),
                "width":  int(w),
                "height": int(h),
                "area":   int(area),
                "_contour": c,   # kept for debug drawing; ignore downstream
            })

    return detections


def get_primary_obstacle(detections):
    """
    Select the closest obstacle. Closeness is approximated by bounding-box
    width (wider = nearer), per the project spec.

    Returns the chosen detection dict, or None if the list is empty.
    """
    if not detections:
        return None
    return max(detections, key=lambda d: d["width"])


# ---------------------------------------------------------------------------
# Debug drawing
# ---------------------------------------------------------------------------
def build_views(frame, detections, presets=None):
    """
    Produce the same auxiliary views hsv_webpage.py shows:
      - combined binary mask (union of all obstacle-class masks)
      - segmented image (frame AND mask)
    Returns (mask_bgr, segmented_bgr).
    """
    if frame is None:
        return None, None
    if presets is None:
        presets = _PRESETS

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    combined = np.zeros(frame.shape[:2], dtype=np.uint8)
    for cls, (lower, upper) in presets.items():
        m = _mask_for_class(lab, lower, upper)
        combined = cv2.bitwise_or(combined, m)

    segmented = cv2.bitwise_and(frame, frame, mask=combined)
    mask_bgr = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)
    return mask_bgr, segmented


def draw_debug(frame, detections, primary=None, target_x=None):
    """
    Draw bounding boxes, contour outlines, and centers onto a COPY of frame.

    Args:
        frame:       BGR image.
        detections:  list from detect_lab_obstacles().
        primary:     optional chosen obstacle (highlighted thicker).
        target_x:    optional int; draws the current steering-target vertical line.

    Returns:
        annotated BGR frame (copy).
    """
    if frame is None:
        return frame
    out = frame.copy()

    for d in detections:
        color = _DRAW_COLORS.get(d["class"], (255, 255, 255))
        x1, y1, x2, y2 = d["bbox"]
        is_primary = primary is not None and d is primary
        thickness = 3 if is_primary else 1

        # contour outline
        if "_contour" in d:
            cv2.drawContours(out, [d["_contour"]], -1, color, 1)

        # bounding box
        cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)

        # center dot
        cv2.circle(out, (d["cx"], d["cy"]), 4, color, -1)

        # label
        label = f'{d["class"]} w{d["width"]}'
        cv2.putText(out, label, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # steering target line (where we WANT the obstacle center to end up)
    if target_x is not None:
        tx = int(target_x)
        cv2.line(out, (tx, 0), (tx, out.shape[0]), (0, 255, 255), 2)
        cv2.putText(out, "target", (tx + 4, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    return out


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    print("[lab_detector] loaded presets:", list(_PRESETS.keys()))
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        dets = detect_lab_obstacles(img)
        print(f"[lab_detector] {len(dets)} detection(s):")
        for d in dets:
            print({k: v for k, v in d.items() if k != "_contour"})
        prim = get_primary_obstacle(dets)
        dbg = draw_debug(img, dets, primary=prim)
        cv2.imwrite("lab_debug_out.png", dbg)
        print("[lab_detector] wrote lab_debug_out.png")

