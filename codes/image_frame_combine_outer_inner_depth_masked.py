import cv2
import json
import threading
import numpy as np

# ============================================================
# PID GAINS (unchanged from V1)
# ============================================================
KP_STEERING = 0.4
KP_LINE_CENTERING = 0.4

# ============================================================
# DEPTH SENSITIVITY CONTROL (unchanged from V1)
# ============================================================
DEPTH_IMPORTANCE_FACTOR = 1.1

# ============================================================
# SHAPE FILTERING CONSTANTS
# ============================================================
MIN_CONTOUR_AREA = 1500   # ignore tiny colour blobs
MIN_WIDTH = 30             # ignore thin blobs (ported from V2)

# ============================================================
# NEW: LAB COLOUR SPACE + LIVE-TUNABLE PRESETS (ported from V2)
# ============================================================
# LAB is far less sensitive to lighting/exposure swings than HSV, which is
# why V2 switched to it. Presets are loaded from a JSON file on disk so they
# can be tuned live from the web UI (see main.py's /tuning page) without a
# code redeploy -- exactly the workflow V2's hsv_webpage.py tooling enabled.
#
# NOTE ON PATH: this is intentionally a relative filename (resolved against
# the current working directory the main script is launched from), not an
# absolute path baked in for one specific Pi/user -- adjust PRESETS_FILE if
# you want it pinned to a specific location.
PRESETS_FILE = "vision_lab_presets.json"

DEFAULT_PRESETS = {
    "red":    {"l_min": 0,   "l_max": 255, "a_min": 146, "a_max": 255, "b_min": 100, "b_max": 255},
    "green":  {"l_min": 0,   "l_max": 255, "a_min": 0,   "a_max": 120, "b_min": 80,  "b_max": 200},
    "white":  {"l_min": 100, "l_max": 255, "a_min": 0,   "a_max": 255, "b_min": 0,   "b_max": 255},
}

STREAM_COLORS = ["red", "green", "white"]

COLOR_PRESETS = {k: dict(v) for k, v in DEFAULT_PRESETS.items()}
presets_lock = threading.Lock()

MORPH_KERNEL_5x5 = np.ones((5, 5), np.uint8)


def load_presets():
    """Load LAB presets from disk, falling back to built-in defaults on any error."""
    global COLOR_PRESETS
    presets = {k: dict(v) for k, v in DEFAULT_PRESETS.items()}
    try:
        with open(PRESETS_FILE) as f:
            loaded = json.load(f)
        for color in STREAM_COLORS:
            if color in loaded:
                presets[color].update(loaded[color])
        print(f"[VISION] Loaded LAB presets from {PRESETS_FILE}")
    except Exception as e:
        print(f"[VISION] Could not load {PRESETS_FILE} ({e}); using defaults.")
    with presets_lock:
        COLOR_PRESETS = presets
    return presets


def save_presets():
    """Persist the current in-memory presets to disk."""
    with presets_lock:
        snapshot = {k: dict(v) for k, v in COLOR_PRESETS.items()}
    try:
        with open(PRESETS_FILE, "w") as f:
            json.dump(snapshot, f, indent=4)
        return True
    except Exception as e:
        print(f"[VISION] Save failed: {e}")
        return False


def get_presets():
    with presets_lock:
        return {k: dict(v) for k, v in COLOR_PRESETS.items()}


def update_preset(color, values):
    """Update one colour's preset in memory (does NOT write to disk -- call
    save_presets() separately, same two-step flow as V2's /update + /save)."""
    with presets_lock:
        if color in COLOR_PRESETS:
            COLOR_PRESETS[color].update(values)
            return True
    return False


# Load from disk (or defaults) once at import time.
load_presets()


def lab_mask(lab_frame, color):
    """Build the LAB inRange mask for a colour preset, with a small open to
    clean up speckle noise (same pattern as V2's lab_mask)."""
    with presets_lock:
        p = dict(COLOR_PRESETS[color])
    mask = cv2.inRange(
        lab_frame,
        np.array([p["l_min"], p["a_min"], p["b_min"]]),
        np.array([p["l_max"], p["a_max"], p["b_max"]]),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL_5x5, iterations=2)
    return mask


# ============================================================
# NEW: DRIVABLE-AREA (TRACK-MASK) FALSE-POSITIVE FILTERING (ported from V2)
# ============================================================
# A pillar that is genuinely on the track sits on the white floor, so there is
# white directly below its base. Coloured things that are NOT on the track
# (spectators, banners, far walls reflecting colour) have no contiguous white
# floor beneath them -> filtered out here so they never trigger steering.
#
# Rather than trusting any nearby white pixels, build_track_mask() first finds
# the SINGLE connected white region that reaches the bottom of the frame (the
# floor the robot is actually standing on). Any other white patch in the scene
# -- ceiling, banners, a spectator's shirt -- is a separate connected component
# and gets discarded, so a pillar sitting in front of one of those does not
# count as "on track".
TRACK_MASK_CLOSE_PX = 25          # close small gaps (pillars/lines) so the floor is one blob
TRACK_MIN_AREA_FRAC = 0.02        # ignore white components smaller than this frac of frame
TRACK_BASE_DILATE_PX = 12         # tolerance band around the track edge for a pillar base
WHITE_FLOOR_BAND_PX = 24          # height of the band sampled below a pillar
WHITE_FLOOR_MIN_RATIO = 0.30      # min fraction of that band that must be on-track
LAB_ROI_TOP_FRAC = 0.35           # ignore blobs whose base sits above this frac of frame height
                                   # (horizon/background, not something in front of the robot)


def build_track_mask(white_mask):
    """Return a binary mask of the single drivable white region (the floor)."""
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
        reaches_bottom = (top + height) >= (h - 2)
        if reaches_bottom and area > best_area:
            best_label, best_area = lbl, area

    if best_label == 0:
        return np.zeros_like(white_mask)

    track = np.where(labels == best_label, 255, 0).astype(np.uint8)
    if TRACK_BASE_DILATE_PX > 0:
        dk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TRACK_BASE_DILATE_PX, TRACK_BASE_DILATE_PX))
        track = cv2.dilate(track, dk, iterations=1)
    return track


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
    if (by2 - by1) < 2 or (bx2 - bx1) < 2:
        cx = int(np.clip((x1 + x2) // 2, 0, w - 1))
        cy = int(np.clip(y2 - 1, 0, h - 1))
        return track_mask[cy, cx] != 0
    band = track_mask[by1:by2, bx1:bx2]
    ratio = cv2.countNonZero(band) / band.size
    return ratio >= WHITE_FLOOR_MIN_RATIO


def _find_valid_contours(mask, roi_mask, track_mask, frame_h):
    """Find contours for one colour mask that pass ALL of: ROI box, min
    area/width, not-in-the-horizon, and on-drivable-area filtering."""
    masked = cv2.bitwise_and(mask, mask, mask=roi_mask)
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_top = int(frame_h * LAB_ROI_TOP_FRAC)
    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area <= MIN_CONTOUR_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w <= MIN_WIDTH:
            continue
        if (y + h) < roi_top:
            continue  # base too high in frame -> horizon/background, not in front of us
        if not _on_drivable_area(track_mask, [x, y, x + w, y + h]):
            continue  # off-track colour -> ignore, don't steer for it
        valid.append(c)
    return valid


def segment_for_stream(frame_bgr, color):
    """Colour-segmented preview of the current frame for the web tuning UI
    (shows exactly what lab_mask() currently classifies as that colour)."""
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    mask = lab_mask(lab, color)
    return cv2.bitwise_and(frame_bgr, frame_bgr, mask=mask)


def segment_track_for_stream(frame_bgr):
    """Preview showing ONLY the drivable white track region; everything else
    (walls, banners, off-track colour) is blacked out -- lets you visually
    confirm the false-positive filter is seeing the floor correctly."""
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    track = build_track_mask(lab_mask(lab, "white"))
    return cv2.bitwise_and(frame_bgr, frame_bgr, mask=track)


def analyze_black_between_lines(frame, inner_start, inner_end):
    # Unchanged from V1
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x1, y1 = inner_start
    x2, y2 = inner_end
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
    if x1 >= x2 or y1 >= y2:
        return None
    roi = gray[y1:y2, x1:x2]
    _, black_mask = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)
    h, w = black_mask.shape
    left = black_mask[:, :w // 2]
    right = black_mask[:, w // 2:]
    black_left = np.sum(left) / 255
    black_right = np.sum(right) / 255
    total_black = black_left + black_right
    if total_black == 0:
        return None
    balance = (black_right - black_left) / total_black
    correction = KP_LINE_CENTERING * balance * 100
    return correction


def process_frame_for_steering(frame, use_outer_roi_and_bottom_point=False):
    """
    Processes a camera frame to determine steering.

    CHANGED vs V1:
      - Colour detection now runs in LAB space (lab_mask / COLOR_PRESETS)
        instead of fixed HSV thresholds -- more lighting-robust, and the
        presets are live-tunable from the web UI without a redeploy.
      - Every candidate contour must ALSO pass the drivable-area (track-mask)
        check -- a red/green blob that isn't standing on the connected white
        floor region is dropped before it ever reaches the steering decision.
        This is on top of (not instead of) the existing outer/inner ROI box,
        so "outside the field" is now filtered two ways: geometrically (ROI)
        and semantically (is there actually floor under it).

    UNCHANGED vs V1: return signature, the outer/inner ROI + target-line
    steering geometry, the depth_factor scaling, the "largest wins if both
    colours present" selection, and the black-line/dark-frame fallback logic.

    Return signature: (processed_frame, steering_angle, output_mask, logic_label, depth_factor)
    """
    if frame is None:
        return None, 0, None, "none", 0.0

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    h, w = frame.shape[:2]
    processed = frame.copy()
    steering_angle, logic_label = 0, "none"

    # --- NEW: drivable-area (track) mask, built once per frame ---
    white_mask = lab_mask(lab, "white")
    track_mask = build_track_mask(white_mask)

    # --- LAB colour masks (replaces the old HSV inRange calls) ---
    binary_red = lab_mask(lab, "red")
    binary_green = lab_mask(lab, "green")
    output_mask = binary_green

    # --- UI & LOGIC REGIONS (unchanged from V1) ---
    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))
    inner_start = (int(0.15 * w), int(0.25 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))

    if use_outer_roi_and_bottom_point:
        roi_start_pt, roi_end_pt = outer_start, outer_end
        detection_point_mode = "BOTTOM"
        green_target_x, red_target_x = outer_end[0], outer_start[0]
    else:
        roi_start_pt, roi_end_pt = inner_start, inner_end
        detection_point_mode = "CENTER"
        green_target_x, red_target_x = inner_end[0], inner_start[0]

    # --- ROI masking (unchanged geometry) ---
    roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(roi_mask, roi_start_pt, roi_end_pt, 255, -1)

    # --- Contour detection, now gated by ROI + area/width + track-mask ---
    valid_red_obstacles = _find_valid_contours(binary_red, roi_mask, track_mask, h)
    valid_green_obstacles = _find_valid_contours(binary_green, roi_mask, track_mask, h)

    chosen_contour = None
    largest_red = max(valid_red_obstacles, key=cv2.contourArea) if valid_red_obstacles else None
    largest_green = max(valid_green_obstacles, key=cv2.contourArea) if valid_green_obstacles else None

    if largest_red is not None and largest_green is not None:
        chosen_contour = largest_red if cv2.contourArea(largest_red) > cv2.contourArea(largest_green) else largest_green
    elif largest_red is not None:
        chosen_contour = largest_red
    elif largest_green is not None:
        chosen_contour = largest_green

    if chosen_contour is largest_red and largest_red is not None:
        logic_label = "red_obstacle"
    elif chosen_contour is largest_green and largest_green is not None:
        logic_label = "obstacle"
    else:
        logic_label = "none"

    # --- STEERING CALCULATION WITH DEPTH FACTOR (unchanged from V1) ---
    depth_factor = 0.0  # 0.0 = no obstacle / no proximity urgency
    obstacle_center_point, obstacle_target_x = None, None
    if chosen_contour is not None:
        x, y, wc, hc = cv2.boundingRect(chosen_contour)

        obstacle_bottom_y = y + hc
        normalized_depth = obstacle_bottom_y / h
        depth_factor = DEPTH_IMPORTANCE_FACTOR * normalized_depth

        pX = x + wc // 2
        pY = y + (hc if detection_point_mode == "BOTTOM" else hc // 2)
        obstacle_center_point = (pX, pY)

        if logic_label == "red_obstacle":
            output_mask = binary_red
            error = red_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor
            obstacle_target_x = red_target_x
        else:  # Green obstacle
            output_mask = binary_green
            error = green_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor
            obstacle_target_x = green_target_x

    # --- Fallback Logic (unchanged from V1) ---
    if logic_label == "none":
        correction = analyze_black_between_lines(frame, inner_start, inner_end)
        if correction is not None:
            steering_angle = correction
            logic_label = "line_centering"
        else:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if np.mean(gray_frame) < 50:
                steering_angle = -45
                logic_label = "corner_avoid"

    # --- FINAL UI OVERLAYS ---
    cv2.rectangle(processed, outer_start, outer_end, (0, 255, 0), 2)
    cv2.rectangle(processed, inner_start, inner_end, (0, 255, 0), 2)

    center_x = (roi_start_pt[0] + roi_end_pt[0]) // 2
    cv2.line(processed, (center_x, roi_start_pt[1]), (center_x, roi_end_pt[1]), (255, 255, 255), 1)

    cv2.line(processed, (green_target_x, inner_start[1]), (green_target_x, inner_end[1]), (255, 255, 0), 2)
    cv2.line(processed, (red_target_x, inner_start[1]), (red_target_x, inner_end[1]), (255, 0, 255), 2)

    # NEW: faint outline of the drivable-area mask, so you can see on the live
    # stream exactly what the false-positive filter currently considers "track".
    track_contours, _ = cv2.findContours(track_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(processed, track_contours, -1, (255, 200, 0), 1)

    if chosen_contour is not None and obstacle_center_point is not None:
        pX, pY = obstacle_center_point
        x, y, wc, hc = cv2.boundingRect(chosen_contour)
        cv2.rectangle(processed, (x, y), (x + wc, y + hc), (255, 255, 0), 2)
        cv2.drawContours(processed, [chosen_contour], -1, (0, 0, 255), 2)
        cv2.circle(processed, (pX, pY), 7, (0, 0, 255), -1)
        cv2.line(processed, (pX, pY), (obstacle_target_x, pY), (255, 0, 0), 3)

        cv2.putText(processed, f"Depth Factor: {depth_factor:.2f}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return processed, steering_angle, output_mask, logic_label, depth_factor