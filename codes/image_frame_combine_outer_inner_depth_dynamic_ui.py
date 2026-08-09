import cv2
import numpy as np
import time

# --- PID GAINS ---
KP_STEERING = 0.4  
KD_STEERING = 0.15   
KP_LINE_CENTERING = 0.4
KD_LINE_CENTERING = 0.05  

# --- DEPTH SENSITIVITY CONTROL ---
DEPTH_IMPORTANCE_FACTOR = 1.8

# --- SHAPE FILTERING CONSTANTS ---
MIN_CONTOUR_AREA = 2000

# --- DERIVATIVE STATE REGISTERS ---
_prev_obstacle_error = 0.0
_prev_obstacle_time = None
_prev_balance = 0.0
_prev_balance_time = None

def analyze_black_between_lines(frame, inner_start, inner_end):
    global _prev_balance, _prev_balance_time

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
        _prev_balance = 0.0
        _prev_balance_time = None
        return None

    balance = (black_right - black_left) / total_black

    now = time.time()
    if _prev_balance_time is None:
        derivative = 0.0
    else:
        dt = now - _prev_balance_time
        derivative = (balance - _prev_balance) / dt if dt > 0 else 0.0

    _prev_balance = balance
    _prev_balance_time = now

    correction = (KP_LINE_CENTERING * balance + KD_LINE_CENTERING * derivative) * 100
    return correction

def process_frame_for_steering(frame, use_outer_roi_and_bottom_point=False):
    """
    Processes a camera frame to determine track lines and navigate straightaways.
    Uses highly stable, centered rectangular boundaries to prevent visual jitter.
    """
    global _prev_obstacle_error, _prev_obstacle_time

    if frame is None:
        return None, 0, None, "none", 9999.0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    processed = frame.copy()
    steering_angle, logic_label = 0, "none"
    
    # Color Calibration Windows
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

    # --- UI & LOGIC REGIONS ---
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

    roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(roi_mask, roi_start_pt, roi_end_pt, 255, -1)
    masked_binary_red = cv2.bitwise_and(binary_red, binary_red, mask=roi_mask)
    masked_binary_green = cv2.bitwise_and(binary_green, binary_green, mask=roi_mask)
    
    # --- OBSTACLE TRACKING ---
    chosen_contour, obstacle_center_point, obstacle_target_x = None, None, None
    red_contours, _ = cv2.findContours(masked_binary_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(masked_binary_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_red = [cnt for cnt in red_contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
    valid_green = [cnt for cnt in green_contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
    largest_red = max(valid_red, key=cv2.contourArea) if valid_red else None
    largest_green = max(valid_green, key=cv2.contourArea) if valid_green else None
    
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

    # --- TRACKING GEOMETRY AND STEERING MATH ---
    depth_factor = 1.0 
    vertical_distance_to_outer_bottom = 9999.0 
    
    if chosen_contour is not None:
        x, y, wc, hc = cv2.boundingRect(chosen_contour)
        obstacle_bottom_y = y + hc

        # Calculate exact Y-axis horizontal gap to outer baseline boundary line
        vertical_distance_to_outer_bottom = float(outer_end[1] - obstacle_bottom_y)
        normalized_depth = obstacle_bottom_y / h
        depth_factor = DEPTH_IMPORTANCE_FACTOR * normalized_depth

        pX = x + wc // 2
        pY = y + (hc if detection_point_mode == "BOTTOM" else hc // 2)
        obstacle_center_point = (pX, pY)

        if logic_label == "red_obstacle":
            output_mask = binary_red
            error = red_target_x - pX
            obstacle_target_x = red_target_x
        else: 
            output_mask = binary_green
            error = green_target_x - pX
            obstacle_target_x = green_target_x

        now = time.time()
        dt = now - _prev_obstacle_time if _prev_obstacle_time else 0.05
        derivative = (error - _prev_obstacle_error) / dt if dt > 0 else 0.0

        _prev_obstacle_error = error
        _prev_obstacle_time = now

        pd_output = KP_STEERING * error + KD_STEERING * derivative
        steering_angle = pd_output * depth_factor
    else:
        _prev_obstacle_error = 0.0
        _prev_obstacle_time = None

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

    # --- UI RENDERING PANELS ---
    cv2.rectangle(processed, outer_start, outer_end, (0, 255, 0), 2)
    cv2.rectangle(processed, inner_start, inner_end, (0, 255, 0), 2)
    center_x = (roi_start_pt[0] + roi_end_pt[0]) // 2
    cv2.line(processed, (center_x, roi_start_pt[1]), (center_x, roi_end_pt[1]), (255, 255, 255), 1)
    cv2.line(processed, (green_target_x, inner_start[1]), (green_target_x, inner_end[1]), (255, 255, 0), 2)
    cv2.line(processed, (red_target_x, inner_start[1]), (red_target_x, inner_end[1]), (255, 0, 255), 2)

    if chosen_contour is not None and obstacle_center_point is not None:
        pX, pY = obstacle_center_point
        cv2.rectangle(processed, (x, y), (x + wc, y + hc), (255, 255, 0), 2)
        cv2.drawContours(processed, [chosen_contour], -1, (0, 0, 255), 2)
        cv2.circle(processed, (pX, pY), 7, (0, 0, 255), -1)
        cv2.line(processed, (pX, pY), (obstacle_target_x, pY), (255, 0, 0), 3)
        cv2.putText(processed, f"Depth: {depth_factor:.2f} | Y-Dist: {int(vertical_distance_to_outer_bottom)}px", 
                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    return processed, steering_angle, output_mask, logic_label, vertical_distance_to_outer_bottom