import cv2
import numpy as np

# --- PID GAINS ---
KP_STEERING = 0.4
KP_LINE_CENTERING = 0.4

# --- NEW: PD STEERING FOR PILLAR AVOIDANCE ---
# Replaces the old multiplicative DEPTH_IMPORTANCE_FACTOR approach with a
# proper PD controller on the pillar x-error, plus a separate additive
# y-offset correction - modeled directly on the asparagus team's
# `angle = straight + error*cKp + (error-prevError)*cKd` plus their
# `cy * (cPillar.y - ROI3[1])` nudge in find_pillar.py.
KD_STEERING = 0.15        # derivative gain on x-error (their cKd)
Y_OFFSET_GAIN = 0.06       # additive y-based nudge gain (their cy)

# --- SHAPE FILTERING CONSTANTS ---
MIN_CONTOUR_AREA = 1000

# --- PERSISTENT PD STATE ---
# process_frame_for_steering() is called once per control-loop iteration
# with no object to hold state across calls (unlike asparagus's Pillar/
# prevError, which live at module/script scope in their main loop). We
# track prevError per obstacle color here at module scope instead, so the
# derivative term has something real to diff against frame-to-frame.
_prev_error = {"red_obstacle": 0.0, "obstacle": 0.0}


def analyze_black_between_lines(frame, inner_start, inner_end):
    # This function remains unchanged
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


def process_frame_for_steering(frame, draw_overlays=True):
    """
    Processes a camera frame to determine steering.

    FIX: the near-field/far-field ROI switch (outer rectangle + bottom
    detection point vs. inner rectangle + center detection point) has been
    removed entirely. The inner rectangle is now the single ROI used for
    obstacle detection regardless of LiDAR-reported distance, and the
    detection point for BOTH red and green obstacles is always the bottom
    of the bounding box (previously only the near-field/outer-ROI path used
    the bottom point; the far-field/inner-ROI path used the box center).

    UPDATE: pillar steering no longer uses a multiplicative depth_factor
    (KP_STEERING * error * depth_factor). It now uses PD control on the
    x-error (KP_STEERING + KD_STEERING against a persisted prevError, one
    per obstacle color) plus a separate additive y-offset correction based
    on how far down the ROI the obstacle's bottom point sits - this mirrors
    the asparagus team's find_pillar.py steering logic:
        angle = straight + error*cKp + (error - prevError)*cKd
        angle -= cy*(y - ROI_top) if error <= 0 else -cy*(y - ROI_top)
    translated into this frame's steering_angle convention.

    draw_overlays: when False, skips ALL cv2 drawing calls (rectangles, lines,
    contours, text) and simply returns the untouched frame. This is the
    single biggest per-frame cost after the color masking/contour work, so
    callers that aren't streaming/displaying the debug view (or that want
    max control-loop throughput) should pass draw_overlays=False.
    """
    if frame is None:
        return None, 0, None, "none", 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    # Only copy the frame if we're actually going to draw on it - copying a
    # ~1150x650 BGR array every call is not free.
    processed = frame.copy() if draw_overlays else frame
    steering_angle, logic_label = 0, "none"

    # Color definitions are unchanged
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 150, 100])
    upper_red1 = np.array([7, 255, 255])
    lower_red2 = np.array([173, 150, 100])
    upper_red2 = np.array([180, 255, 255])

    # Get full-frame color masks
    binary_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    binary_red = cv2.bitwise_or(mask_red1, mask_red2)

    output_mask = binary_green

    # FIX: top edge raised from 0.25h to 0.12h. At 0.25h, a distant obstacle
    # (which appears small and high in the frame) had most of its height
    # clipped away by the ROI mask before contourArea() ever saw it - only a
    # sliver near the object's bottom survived, often landing well under
    # MIN_CONTOUR_AREA even after lowering the threshold. Raising the top
    # edge gives distant obstacles more vertical room to fall fully inside
    # the ROI, so the area filter is judging the obstacle's real size again
    # instead of an accidental sliver of it.
    inner_start = (int(0.15 * w), int(0.12 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))
    # Outer rectangle is kept only as a drawn visual reference line on the
    # debug stream now - it plays no role in ROI masking or detection.
    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))

    # --- Inner rectangle is the only ROI; detection point is always
    # the bottom of the bounding box for both colors. ---
    roi_start_pt, roi_end_pt = inner_start, inner_end
    green_target_x, red_target_x = inner_end[0], inner_start[0]

    # --- ROI Masking and Contour Detection (Unchanged) ---
    roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(roi_mask, roi_start_pt, roi_end_pt, 255, -1)
    masked_binary_red = cv2.bitwise_and(binary_red, binary_red, mask=roi_mask)
    masked_binary_green = cv2.bitwise_and(binary_green, binary_green, mask=roi_mask)

    # --- OBSTACLE DETECTION (Unchanged) ---
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

    # DEBUG AID: when nothing clears MIN_CONTOUR_AREA, grab the single
    # largest raw contour (red or green, un-filtered) so its real pixel
    # area can be shown on the debug stream. This turns "why isn't it
    # detecting" into a number you can read off the stream instead of
    # guessing at a new MIN_CONTOUR_AREA value blind.
    rejected_contour = None
    if chosen_contour is None:
        all_raw_contours = list(red_contours) + list(green_contours)
        if all_raw_contours:
            rejected_contour = max(all_raw_contours, key=cv2.contourArea)

    # --- STEERING CALCULATION: PD control + y-offset correction ---
    # (depth_factor / DEPTH_IMPORTANCE_FACTOR removed)
    error = 0.0
    y_offset_term = 0.0
    if chosen_contour is not None:
        x, y, wc, hc = cv2.boundingRect(chosen_contour)

        pX = x + wc // 2
        pY = y + hc  # Bottom of bounding box - always used now, for both colors.
        obstacle_center_point = (pX, pY)

        if logic_label == "red_obstacle":
            output_mask = binary_red
            target_x = red_target_x
        else:  # Green obstacle
            output_mask = binary_green
            target_x = green_target_x

        error = target_x - pX
        obstacle_target_x = target_x

        # PD term on x-error, mirroring asparagus's
        # `angle = straight + error*cKp + (error-prevError)*cKd`.
        prev_error = _prev_error.get(logic_label, 0.0)
        steering_angle = KP_STEERING * error + KD_STEERING * (error - prev_error)
        _prev_error[logic_label] = error

        # print(f"Steering Angle : {int(steering_angle)}")


        # Additive y-offset correction, mirroring asparagus's
        # `cy * (cPillar.y - ROI3[1])` nudge: the further down the ROI the
        # obstacle's bottom point sits (closer to the car), the bigger the
        # extra push, applied in the direction indicated by the sign of the
        # x-error rather than folded into a multiplicative gain on the
        # whole term.
        y_offset_term = Y_OFFSET_GAIN * (pY - inner_start[1])
        steering_angle += -y_offset_term if error <= 0 else y_offset_term

        # print(f"Final Steering Angle : {int(steering_angle)}, Y Offset : {int(y_offset_term)}")

    # Fallback Logic (Unchanged)
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
        # No pillar tracked this frame - reset both colors' prevError so a
        # stale error from several frames ago doesn't spike the D-term the
        # next time a pillar reappears.
        _prev_error["red_obstacle"] = 0.0
        _prev_error["obstacle"] = 0.0

    # --- FINAL UI OVERLAYS (now skipped entirely when draw_overlays=False) ---
    if draw_overlays:
        cv2.rectangle(processed, outer_start, outer_end, (0, 255, 0), 2)
        cv2.rectangle(processed, inner_start, inner_end, (0, 255, 0), 2)

        center_x = (roi_start_pt[0] + roi_end_pt[0]) // 2
        cv2.line(processed, (center_x, roi_start_pt[1]), (center_x, roi_end_pt[1]), (255, 255, 255), 1)

        cv2.line(processed, (green_target_x, inner_start[1]), (green_target_x, inner_end[1]), (255, 255, 0), 2)
        cv2.line(processed, (red_target_x, inner_start[1]), (red_target_x, inner_end[1]), (255, 0, 255), 2)

        if chosen_contour is not None and obstacle_center_point is not None:
            pX, pY = obstacle_center_point
            x, y, wc, hc = cv2.boundingRect(chosen_contour)
            cv2.rectangle(processed, (x, y), (x + wc, y + hc), (255, 255, 0), 2)
            cv2.drawContours(processed, [chosen_contour], -1, (0, 0, 255), 2)
            cv2.circle(processed, (pX, pY), 7, (0, 0, 255), -1)
            cv2.line(processed, (pX, pY), (obstacle_target_x, pY), (255, 0, 0), 3)

            contour_area_px = cv2.contourArea(chosen_contour)
            cv2.putText(processed, f"Area: {int(contour_area_px)}px", (x, max(0, y - 30)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(processed, f"PD err:{error:.0f} yOff:{y_offset_term:.1f}", (x, max(0, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        elif rejected_contour is not None:
            # Nothing cleared MIN_CONTOUR_AREA, but something was seen inside
            # the ROI - draw it in orange with its real area so you can read
            # the actual number off the stream instead of guessing at a new
            # threshold value blind.
            rx, ry, rwc, rhc = cv2.boundingRect(rejected_contour)
            rejected_area_px = cv2.contourArea(rejected_contour)
            cv2.rectangle(processed, (rx, ry), (rx + rwc, ry + rhc), (0, 165, 255), 2)
            cv2.putText(processed, f"Rejected: {int(rejected_area_px)}px < {MIN_CONTOUR_AREA}",
                        (rx, max(0, ry - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    return processed, steering_angle, output_mask, logic_label, 0