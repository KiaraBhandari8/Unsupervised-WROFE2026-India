import cv2
import numpy as np
import math

# --- PID GAINS ---
KP_STEERING = 0.4
KP_LINE_CENTERING = 0.4


# --- DEPTH SENSITIVITY CONTROL ---
DEPTH_IMPORTANCE_FACTOR = 1.1

# --- SHAPE FILTERING CONSTANTS ---
MIN_CONTOUR_AREA = 2500

# --- MASK CLEANUP KERNEL ---
_MASK_CLEANUP_KERNEL = np.ones((5, 5), np.uint8)

# --- CAMERA GEOMETRY (Raspberry Pi Camera Module 3 WIDE) ---
# Confirmed horizontal FOV = 102 degrees. Capture is 2304x1296, exactly half
# the sensor's native 4608x2592 resolution at the same aspect ratio (a
# binned/scaled full-frame readout, not a crop), so the full 102 deg spans
# the entire processed frame width with no correction needed.
CAMERA_HORIZONTAL_FOV_DEG = 102.0


def pixel_offset_to_angle(pixel_x, frame_width, hfov_deg=CAMERA_HORIZONTAL_FOV_DEG):
    """
    Converts a pixel x-coordinate in the processed frame into a real-world
    angle relative to the camera's optical axis, using a tangent (pinhole
    projection) mapping rather than a plain linear one.

    Why tangent and not linear: a linear pixel->angle mapping assumes equal
    angular width per pixel across the whole frame, which is a reasonable
    approximation for narrow lenses but increasingly wrong as FOV widens --
    at 102 degrees the outer thirds of the frame cover noticeably more real
    angle per pixel than the center does under a true perspective (pinhole)
    projection. Since this angle is exactly what gets used to mask the
    LiDAR array in follow_the_gap.py, getting it right matters most right
    at the frame edges -- which is exactly where corner-apex obstacles tend
    to sit.

    Convention (matches the existing LiDAR convention in the main script):
        0 deg   = straight ahead (frame center)
        negative = left of center
        positive = right of center

    Args:
        pixel_x: x-coordinate in the processed frame (same frame
            process_frame_for_steering operates on).
        frame_width: width of that frame in pixels.
        hfov_deg: camera horizontal field of view in degrees.

    Returns:
        angle in degrees (float).
    """
    center_x = frame_width / 2.0
    normalized_offset = (pixel_x - center_x) / center_x  # -1.0 .. +1.0
    normalized_offset = max(-1.0, min(1.0, normalized_offset))

    half_fov_rad = math.radians(hfov_deg / 2.0)
    angle_rad = math.atan(normalized_offset * math.tan(half_fov_rad))
    return math.degrees(angle_rad)


def analyze_black_between_lines(frame, inner_start, inner_end):
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


def get_roi_geometry(frame):
    h, w = frame.shape[:2]
    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))
    inner_start = (int(0.15 * w), int(0.25 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))
    return outer_start, outer_end, inner_start, inner_end


def draw_debug_overlay(frame, use_outer_roi_and_bottom_point=False):
    outer_start, outer_end, inner_start, inner_end = get_roi_geometry(frame)

    if use_outer_roi_and_bottom_point:
        roi_start_pt, roi_end_pt = outer_start, outer_end
        green_target_x, red_target_x = outer_end[0], outer_start[0]
    else:
        roi_start_pt, roi_end_pt = inner_start, inner_end
        green_target_x, red_target_x = inner_end[0], inner_start[0]

    cv2.rectangle(frame, outer_start, outer_end, (0, 255, 0), 2)
    cv2.rectangle(frame, inner_start, inner_end, (0, 255, 0), 2)

    center_x = (roi_start_pt[0] + roi_end_pt[0]) // 2
    cv2.line(frame, (center_x, roi_start_pt[1]), (center_x, roi_end_pt[1]), (255, 255, 255), 1)

    cv2.line(frame, (green_target_x, inner_start[1]), (green_target_x, inner_end[1]), (255, 255, 0), 2)
    cv2.line(frame, (red_target_x, inner_start[1]), (red_target_x, inner_end[1]), (255, 0, 255), 2)

    return frame


def process_frame_for_steering(frame, use_outer_roi_and_bottom_point=False):
    """
    Processes a camera frame to determine steering AND (new) the obstacle's
    real angular position/footprint for use by follow_the_gap.py.

    Returns:
        (processed, steering_angle, output_mask, logic_label, vision_obstacle_info)

    vision_obstacle_info is None when logic_label is "none",
    "line_centering", or "corner_avoid" (no color obstacle in view this
    tick). When a red/green obstacle IS detected, it is a dict:
        {
            "angle_deg": <center angle of the obstacle, real FOV geometry>,
            "half_width_deg": <half the obstacle's real angular footprint>,
            "pixel_x": <pX, kept for any legacy/debug use>,
        }
    This replaces the old hardcoded 0 in the 5th return slot.
    """
    if frame is None:
        return None, 0, None, "none", None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    processed = frame.copy()
    steering_angle, logic_label = 0, "none"
    vision_obstacle_info = None

    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 150, 100])
    upper_red1 = np.array([7, 255, 255])
    lower_red2 = np.array([173, 150, 100])
    upper_red2 = np.array([180, 255, 255])

    binary_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    binary_red = cv2.bitwise_or(mask_red1, mask_red2)

    output_mask = binary_green

    outer_start, outer_end, inner_start, inner_end = get_roi_geometry(frame)

    if use_outer_roi_and_bottom_point:
        roi_start_pt, roi_end_pt = outer_start, outer_end
        detection_point_mode = "BOTTOM"
        green_target_x, red_target_x = outer_end[0], outer_start[0]
    else:
        roi_start_pt, roi_end_pt = inner_start, inner_end
        detection_point_mode = "CENTER"
        green_target_x, red_target_x = inner_end[0], inner_start[0]

    roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(roi_mask, roi_start_pt, roi_end_pt, 255, -1)
    masked_binary_red = cv2.bitwise_and(binary_red, binary_red, mask=roi_mask)
    masked_binary_green = cv2.bitwise_and(binary_green, binary_green, mask=roi_mask)

    masked_binary_red = cv2.morphologyEx(masked_binary_red, cv2.MORPH_CLOSE, _MASK_CLEANUP_KERNEL)
    masked_binary_green = cv2.morphologyEx(masked_binary_green, cv2.MORPH_CLOSE, _MASK_CLEANUP_KERNEL)

    chosen_contour, obstacle_center_point, obstacle_target_x = None, None, None
    red_contours, _ = cv2.findContours(masked_binary_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(masked_binary_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_red_obstacles = [cnt for cnt in red_contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
    valid_green_obstacles = [cnt for cnt in green_contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
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

    depth_factor = 1.0
    if chosen_contour is not None:
        x, y, wc, hc = cv2.boundingRect(chosen_contour)

        obstacle_bottom_y = y + hc
        normalized_depth = obstacle_bottom_y / h
        depth_factor = DEPTH_IMPORTANCE_FACTOR * normalized_depth

        pX = x + wc // 2
        pY = y + (hc if detection_point_mode == "BOTTOM" else hc // 2)
        obstacle_center_point = (pX, pY)

        # --- NEW: real angular position + footprint for FGM masking ---
        # Uses the bounding box's actual left/right edges (not just the
        # center point + an assumed width), so the angular footprint
        # reflects the obstacle's real size in the frame, via true FOV
        # geometry rather than a flat pixel-per-degree guess.
        angle_left = pixel_offset_to_angle(x, w)
        angle_right = pixel_offset_to_angle(x + wc, w)
        obstacle_angle_center = (angle_left + angle_right) / 2.0
        obstacle_half_width_deg = abs(angle_right - angle_left) / 2.0
        vision_obstacle_info = {
            "angle_deg": obstacle_angle_center,
            "half_width_deg": obstacle_half_width_deg,
            "pixel_x": pX,
        }

        if logic_label == "red_obstacle":
            output_mask = binary_red
            error = red_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor
            obstacle_target_x = red_target_x
        else:
            output_mask = binary_green
            error = green_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor
            obstacle_target_x = green_target_x

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

    processed = draw_debug_overlay(processed, use_outer_roi_and_bottom_point)

    if chosen_contour is not None and obstacle_center_point is not None:
        pX, pY = obstacle_center_point
        x, y, wc, hc = cv2.boundingRect(chosen_contour)
        cv2.rectangle(processed, (x, y), (x + wc, y + hc), (255, 255, 0), 2)
        cv2.drawContours(processed, [chosen_contour], -1, (0, 0, 255), 2)
        cv2.circle(processed, (pX, pY), 7, (0, 0, 255), -1)
        cv2.line(processed, (pX, pY), (obstacle_target_x, pY), (255, 0, 0), 3)

        cv2.putText(processed, f"Depth Factor: {depth_factor:.2f}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(processed, f"Obs Angle: {vision_obstacle_info['angle_deg']:.1f}deg "
                                f"(+/-{vision_obstacle_info['half_width_deg']:.1f})",
                    (x, y + hc + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    return processed, steering_angle, output_mask, logic_label, vision_obstacle_info