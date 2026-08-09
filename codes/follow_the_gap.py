"""
follow_the_gap.py

Reactive Follow-The-Gap (FGM) navigation core.

Replaces the discrete corner state machine (CORNER_DETECT_STOP ->
CORNER_APPROACH_ALIGN -> CORNER_ACTIVE_PIVOT -> CORNER_ALIGN_BACKWARD) and the
LiDAR side-panic override with a single continuous steering function that
runs every loop tick, in every part of the track -- straight corridors,
corners, and corner+obstacle combinations alike.

Core idea, each call:
    1. Build a distance-per-degree array from the raw LiDAR scan, over a
       forward angular window (e.g. -100..+100 deg, 0 = straight ahead,
       negative = left, positive = right -- matches your existing convention).
    2. "Bubble out" the single closest LiDAR point: mask an angular halo
       around it sized so the robot's physical half-width + a safety margin
       cannot fit past it. This is what keeps you off the wall curving in
       at a corner, and off any close obstacle the LiDAR itself sees.
    3. If the vision module reports a red/green obstacle, mask its actual
       angular footprint (computed from its bounding box edges via the
       camera's real FOV -- see pixel_offset_to_angle() in the vision file)
       plus the same safety margin. This is what keeps you off an obstacle
       sitting at/near the corner apex that the LiDAR alone might not
       separate cleanly from the wall.
    4. Find every remaining contiguous run of "free" angles (a "gap").
    5. Pick the best gap and steer toward a point inside it that balances
       "widest/deepest opening" against "closest to where you want to be
       making progress" (the wall-following side), so you don't zig-zag
       toward a wide-but-useless gap when a narrower direct one exists.

Nothing here knows or cares whether it is "in a corner." A corner is simply
a moment where the wall-ahead reading shrinks and the gap center angle
shifts off zero -- the function is identical in every case.
"""

import math

# ============================================================
# TUNING CONSTANTS -- bench-tune these on your actual chassis
# ============================================================

# Angular window scanned in front of the robot every tick.
# Wider = sees corners/obstacles earlier, but includes more side-wall noise.
FGM_SCAN_HALF_WIDTH_DEG = 100

# Physical robot half-width (mm) -- half the widest point of the chassis,
# including wheels/bumpers. Used to size the safety bubble angularly.
ROBOT_HALF_WIDTH_MM = 140

# Extra clearance added on top of the physical half-width (mm). Increase
# this if the robot is still clipping obstacles/walls; decrease if gaps
# are being rejected as "too narrow" when they are actually passable.
SAFETY_MARGIN_MM = 90

# Any LiDAR reading beyond this is treated as "open" -- also used to fill
# in missing/zero (no-return) readings, since those normally mean "too far
# for a valid return" rather than "an obstacle is here."
FGM_MAX_RANGE_MM = 2500.0

# A point is considered "too close to pass" (independent of the bubble)
# if its raw reading is below this threshold. Keeps thin/isolated single-
# degree occupied readings from being ignored.
FGM_OCCUPIED_THRESHOLD_MM = ROBOT_HALF_WIDTH_MM + SAFETY_MARGIN_MM

# Minimum gap width (degrees) to be considered drivable at all. Gaps
# narrower than this are discarded even if technically "free," since a
# single-degree sliver is almost always LiDAR noise, not real clearance.
FGM_MIN_GAP_WIDTH_DEG = 8

# Extra angular buffer (degrees) added around a vision-flagged obstacle's
# real angular footprint, since the vision bounding box can undershoot the
# obstacle's true physical edges slightly (motion blur, partial occlusion).
VISION_OBSTACLE_MARGIN_DEG = 6

# How strongly the final steering angle is pulled toward "progress" (the
# wall-following bias angle) vs. purely toward the widest/deepest gap.
# 0.0 = ignore bias entirely (pure FGM). 1.0 = ignore gap shape entirely.
PROGRESS_BIAS_WEIGHT = 0.30

# Exponential smoothing on the output angle, frame to frame, to prevent
# single-tick LiDAR noise from producing a visibly twitchy servo command.
# 0.0 = no smoothing (raw output every tick). Closer to 1.0 = smoother but
# slower to react -- do not push this high, it fights the whole point of FGM.
FGM_SMOOTHING_ALPHA = 0.25


def _bubble_half_angle_deg(distance_mm):
    """Angular half-width of the safety bubble at a given distance: the
    angle subtended by (robot half-width + margin) at that range. Closer
    obstacles get a wider angular bubble -- correct, since the same
    physical clearance covers more of your field of view up close."""
    distance_mm = max(distance_mm, 1.0)
    return math.degrees(math.atan2(ROBOT_HALF_WIDTH_MM + SAFETY_MARGIN_MM, distance_mm))


def _build_distance_array(scan_data, half_width_deg):
    """Returns (angles, distances) as parallel lists covering
    -half_width_deg..+half_width_deg inclusive, one entry per integer degree.
    Missing or zero (no-return) readings are filled with FGM_MAX_RANGE_MM,
    since a LiDAR no-return normally means "nothing detected in range,"
    i.e. open space, not an obstacle at distance zero."""
    angles = list(range(-half_width_deg, half_width_deg + 1))
    distances = []
    for a in angles:
        d = scan_data.get(a, 0.0)
        if d is None or d <= 0:
            d = FGM_MAX_RANGE_MM
        distances.append(min(d, FGM_MAX_RANGE_MM))
    return angles, distances


def _apply_bubble(angles, distances, occupied):
    """Finds the single closest point in the (already vision-independent)
    LiDAR array and marks the angular bubble around it as occupied. This is
    the step that keeps the robot off whatever is nearest -- a straight
    wall, a wall curving in at a corner, or a LiDAR-visible obstacle."""
    min_idx = min(range(len(distances)), key=lambda i: distances[i])
    min_dist = distances[min_idx]
    bubble_deg = _bubble_half_angle_deg(min_dist)

    center_angle = angles[min_idx]
    for i, a in enumerate(angles):
        if abs(a - center_angle) <= bubble_deg:
            occupied[i] = True
    return center_angle, min_dist, bubble_deg


def _apply_vision_mask(angles, occupied, vision_obstacle_info):
    """Marks the angular sector actually occupied by a vision-detected
    red/green obstacle, AND enforces the mandatory pass-direction rule by
    color. vision_obstacle_info is the dict produced by
    process_frame_for_steering()'s new return value:
        {"angle_deg": <center angle>, "half_width_deg": <half footprint>,
         "logic_label": "obstacle" (green) or "red_obstacle" (red)}
    angle_deg / half_width_deg come from real FOV geometry
    (pixel_offset_to_angle), not a guess.

    MANDATORY DIRECTION RULE (not just "avoid toward more space"):
        green ("obstacle")  -> robot MUST pass with the obstacle on its
                                right, i.e. steer LEFT (negative angles).
        red ("red_obstacle") -> robot MUST pass with the obstacle on its
                                left, i.e. steer RIGHT (positive angles).

    This is enforced by blocking the ENTIRE disallowed side out to the
    scan boundary -- not just the obstacle's own footprint -- so the
    gap-finder physically cannot select a gap on the wrong side even if
    it happens to be wider. Without this, FGM's generic "pick the widest
    gap" behavior can send the robot around the wrong side whenever that
    side happens to have more open space, which breaks the mandatory
    rule even though it looks like "successful avoidance."
    """
    if not vision_obstacle_info:
        return
    center = vision_obstacle_info["angle_deg"]
    half_width = vision_obstacle_info["half_width_deg"] + VISION_OBSTACLE_MARGIN_DEG
    label = vision_obstacle_info.get("logic_label")

    # Base footprint mask -- always applied regardless of color.
    for i, a in enumerate(angles):
        if abs(a - center) <= half_width:
            occupied[i] = True

    # Mandatory one-sided enforcement.
    if label == "obstacle":  # GREEN -> force robot LEFT of the obstacle
        cutoff = center + half_width
        for i, a in enumerate(angles):
            if a >= cutoff:
                occupied[i] = True
    elif label == "red_obstacle":  # RED -> force robot RIGHT of the obstacle
        cutoff = center - half_width
        for i, a in enumerate(angles):
            if a <= cutoff:
                occupied[i] = True


def _apply_occupied_threshold(distances, occupied):
    """Belt-and-suspenders: anything under the flat occupied-distance
    threshold is blocked even if it wasn't the single closest point (e.g.
    a second obstacle nearly as close as the first, which the bubble step
    alone would not necessarily cover)."""
    for i, d in enumerate(distances):
        if d < FGM_OCCUPIED_THRESHOLD_MM:
            occupied[i] = True


def _find_gaps(angles, occupied):
    """Returns a list of gaps, each as a dict:
        {"start_idx", "end_idx", "start_angle", "end_angle", "width_deg"}
    covering every contiguous run of non-occupied indices."""
    gaps = []
    n = len(angles)
    i = 0
    while i < n:
        if not occupied[i]:
            j = i
            while j + 1 < n and not occupied[j + 1]:
                j += 1
            gaps.append({
                "start_idx": i,
                "end_idx": j,
                "start_angle": angles[i],
                "end_angle": angles[j],
                "width_deg": angles[j] - angles[i],
            })
            i = j + 1
        else:
            i += 1
    return gaps


def _best_point_in_gap(angles, distances, gap):
    """Within a chosen gap, returns the angle of the single deepest
    (longest-range) reading. Steering toward this point instead of the
    flat gap center biases the robot toward the most open part of the
    corridor/turn, which matters most right at a corner where one edge of
    the gap is "just barely past the wall" and the other trails off into
    open track -- you want to aim well into the open side, not exactly
    between a close edge and a far edge."""
    lo, hi = gap["start_idx"], gap["end_idx"]
    best_i = max(range(lo, hi + 1), key=lambda i: distances[i])
    return angles[best_i]


def _select_gap(gaps, bias_angle_deg):
    """Scores every gap wide enough to be drivable and returns the best
    one. Score rewards width (more forgiving / safer) and penalizes how
    far the gap's center is from the progress-bias angle (so, all else
    equal, prefer the gap that keeps you tracking the wall-following side
    rather than one that technically has a hair more width somewhere
    irrelevant to progress)."""
    candidates = [g for g in gaps if g["width_deg"] >= FGM_MIN_GAP_WIDTH_DEG]
    if not candidates:
        return None

    def score(g):
        center = (g["start_angle"] + g["end_angle"]) / 2.0
        angular_penalty = abs(center - bias_angle_deg)
        return g["width_deg"] - (PROGRESS_BIAS_WEIGHT * angular_penalty)

    return max(candidates, key=score)


def compute_fgm_steering(scan_data, vision_obstacle_info=None, bias_angle_deg=0.0,
                          prev_angle_deg=0.0):
    """
    Main entry point, called once per loop tick.

    Args:
        scan_data: dict {angle_deg (int): distance_mm (float)} from the
            LiDAR, same format already produced by your LidarScanner.
        vision_obstacle_info: None, or a dict
            {"angle_deg": float, "half_width_deg": float}
            from the updated process_frame_for_steering(). Pass None when
            logic_label is "none" / "line_centering" / "corner_avoid" (no
            actual color obstacle in view this tick).
        bias_angle_deg: the "progress" direction -- e.g. 0.0 for straight
            corridors, or a small nudge toward your wall-following side if
            you want FGM to lean that way when multiple gaps are similar.
        prev_angle_deg: last tick's output angle, for smoothing.

    Returns:
        (steer_angle_deg, debug_info)
        steer_angle_deg: float, 0 = straight ahead, negative = steer left,
            positive = steer right (matches your existing LiDAR convention:
            LEFT_SCAN_ANGLES negative, RIGHT_SCAN_ANGLES positive).
        debug_info: dict with the raw gap list and chosen gap/bubble info,
            useful for drawing an overlay or logging during bench tests.
    """
    angles, distances = _build_distance_array(scan_data, FGM_SCAN_HALF_WIDTH_DEG)
    occupied = [False] * len(angles)

    bubble_center_angle, bubble_min_dist, bubble_deg = _apply_bubble(angles, distances, occupied)
    _apply_vision_mask(angles, occupied, vision_obstacle_info)
    _apply_occupied_threshold(distances, occupied)

    gaps = _find_gaps(angles, occupied)
    chosen_gap = _select_gap(gaps, bias_angle_deg)

    debug_info = {
        "angles": angles,
        "distances": distances,
        "occupied": occupied,
        "gaps": gaps,
        "chosen_gap": chosen_gap,
        "bubble_center_angle": bubble_center_angle,
        "bubble_min_dist": bubble_min_dist,
        "bubble_deg": bubble_deg,
        "vision_obstacle_info": vision_obstacle_info,
    }

    if chosen_gap is None:
        # No drivable gap at all this tick -- every angle in the window is
        # inside some safety bubble/threshold. This should be rare (it
        # means the robot is already too close to something on all sides).
        # Fall back to steering toward whichever single angle has the
        # longest raw range, and flag it so the caller can slow down hard.
        best_i = max(range(len(distances)), key=lambda i: distances[i])
        raw_angle = angles[best_i]
        debug_info["fallback"] = True
    else:
        raw_angle = _best_point_in_gap(angles, distances, chosen_gap)
        gap_center = (chosen_gap["start_angle"] + chosen_gap["end_angle"]) / 2.0
        # Blend the "deepest point" target with the plain gap center so a
        # single noisy far reading at the gap's edge doesn't yank steering
        # all the way to that edge.
        raw_angle = 0.5 * raw_angle + 0.5 * gap_center
        debug_info["fallback"] = False

    smoothed_angle = (FGM_SMOOTHING_ALPHA * prev_angle_deg) + ((1.0 - FGM_SMOOTHING_ALPHA) * raw_angle)
    debug_info["raw_angle"] = raw_angle
    debug_info["smoothed_angle"] = smoothed_angle

    return smoothed_angle, debug_info