import cv2
import numpy as np

# --- PID GAINS ---
KP_STEERING = 0.4
KP_LINE_CENTERING = 0.4


# --- NEW: DEPTH SENSITIVITY CONTROL ---
# This controls how much the steering is amplified for closer objects.
# 1.0 = No depth adjustment.
# 2.0 = An object at the bottom of the frame has its error doubled.
# Higher values mean more aggressive steering for nearby obstacles.
DEPTH_IMPORTANCE_FACTOR = 1.1

# --- SHAPE FILTERING CONSTANTS ---
MIN_CONTOUR_AREA = 1500

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


def process_frame_for_steering(frame, use_outer_roi_and_bottom_point=False):
    """
    Processes a camera frame to determine steering.
    Switches between two modes based on LiDAR input:
    - Far-Field (default): Detects obstacles and sets steering targets using the inner rectangle.
    - Near-Field (LiDAR-triggered): Detects obstacles and sets steering targets using the outer rectangle.
    """
    if frame is None:
        return None, 0, None, "none", 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    processed = frame.copy()
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

    # --- UI & LOGIC REGIONS (Unchanged) ---
    # outer_start = (int(0.15 * w), int(0.15 * h)) 
    # outer_end = (int(0.85 * w), int(0.85 * h))
    # inner_start = (int(0.15 * w), int(0.25 * h))
    # inner_end = (int(0.85 * w), int(0.75 * h))

    outer_start = (int(0.25 * w), int(0.15 * h)) 
    outer_end = (int(0.75 * w), int(0.85 * h))
    inner_start = (int(0.25 * w), int(0.25 * h))
    inner_end = (int(0.75 * w), int(0.75 * h))

    


    # --- Dynamically set ROI, Reference Lines, and Detection Point (Unchanged) ---
    if use_outer_roi_and_bottom_point:
        roi_start_pt, roi_end_pt = outer_start, outer_end
        detection_point_mode = "BOTTOM"
        green_target_x, red_target_x = outer_end[0], outer_start[0]
    else:
        roi_start_pt, roi_end_pt = inner_start, inner_end
        detection_point_mode = "CENTER"
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

    # --- MODIFIED: STEERING CALCULATION WITH DEPTH FACTOR ---
    depth_factor = 1.0 # Default to 1.0 (no effect)
    if chosen_contour is not None:
        x, y, wc, hc = cv2.boundingRect(chosen_contour)
        
        # --- START OF NEW CODE ---
        # 1. Calculate the bottom Y-coordinate of the obstacle.
        obstacle_bottom_y = y + hc

        # 2. Normalize this position based on the frame height (0.0 to 1.0).
        # A value near 1.0 means the obstacle is at the bottom (close).
        # A value near 0.0 means the obstacle is at the top (far).
        normalized_depth = obstacle_bottom_y / h
        
        # 3. Calculate the depth factor.
        # This scales linearly from 1.0 (for distant objects) up to DEPTH_IMPORTANCE_FACTOR.
        depth_factor = DEPTH_IMPORTANCE_FACTOR * normalized_depth
        # --- END OF NEW CODE ---

        pX = x + wc // 2
        pY = y + (hc if detection_point_mode == "BOTTOM" else hc // 2)
        obstacle_center_point = (pX, pY)

        if logic_label == "red_obstacle":
            output_mask = binary_red
            error = red_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor # Apply the factor
            obstacle_target_x = red_target_x
        else: # Green obstacle
            output_mask = binary_green
            error = green_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor # Apply the factor
            obstacle_target_x = green_target_x

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

    # --- FINAL UI OVERLAYS (Unchanged) ---
    # Draw the ROIs
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

        # --- Optional: Display the calculated depth factor on screen for debugging ---
        cv2.putText(processed, f"Depth Factor: {depth_factor:.2f}", (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return processed, steering_angle, output_mask, logic_label, 0