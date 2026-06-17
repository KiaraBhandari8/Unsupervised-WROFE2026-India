import cv2
import numpy as np
import os
import glob

# --- PID GAINS ---
KP_STEERING = 0.4
KP_LINE_CENTERING = 0.4

# --- DEPTH SENSITIVITY CONTROL ---
DEPTH_IMPORTANCE_FACTOR = 1.1

# --- SHAPE FILTERING CONSTANTS ---
MIN_CONTOUR_AREA = 1500
# Obstacle must have 75% of its area on the track
OBSTACLE_ON_TRACK_THRESHOLD = 0.75

def create_track_mask(image):
    """
    Generates a binary mask of the track/drivable area based on color.
    This identifies white, blue, and orange segments as the track.
    """
    height, width, _ = image.shape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for the track (white, blue, orange)
    lower_white = np.array([0, 0, 100])
    upper_white = np.array([180, 60, 255])
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])

    # Create and combine color masks
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
    color_mask = cv2.bitwise_or(white_mask, blue_mask)
    color_mask = cv2.bitwise_or(color_mask, orange_mask)

    # Define a mandatory base region (bottom-center) to ensure connectivity
    base_mask = np.zeros((height, width), dtype=np.uint8)
    y_start = int(height * 0.80)
    x_start = int(width * 0.35)
    x_end = int(width * 0.65)
    base_mask[y_start:height, x_start:x_end] = 255
    
    combined_mask = cv2.bitwise_or(color_mask, base_mask)

    # Clean the mask to remove noise and fill gaps
    kernel = np.ones((7, 7), np.uint8)
    cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    # Keep only the largest contiguous area, which is assumed to be the track
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    final_mask = np.zeros_like(cleaned_mask)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(final_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

    return final_mask

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
    Processes a camera frame to determine steering. It now filters obstacles
    to only consider those located on the detected track area.
    """
    if frame is None:
        return None, 0, None, "none", 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    processed = frame.copy()
    steering_angle, logic_label = 0, "none"
    
    # STEP 1: Generate the track mask for the current frame.
    track_mask = create_track_mask(frame)

    # Color definitions (unchanged)
    lower_green = np.array([35, 60, 30])
    upper_green = np.array([110, 255, 160])
    lower_red1 = np.array([0, 100, 50])
    upper_red1 = np.array([5, 255, 255])
    lower_red2 = np.array([173, 100, 50])
    upper_red2 = np.array([180, 255, 255])
    
    binary_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    binary_red = cv2.bitwise_or(mask_red1, mask_red2)
    output_mask = binary_green

    # --- UI & LOGIC REGIONS (Unchanged) ---
    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))
    inner_start = (int(0.15 * w), int(0.25 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))

    # --- Dynamic ROI Setting (Unchanged) ---
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
    
    # --- MODIFIED OBSTACLE DETECTION & FILTERING ---
    
    # STEP 2: Find all potential obstacles by color and size.
    red_contours, _ = cv2.findContours(masked_binary_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(masked_binary_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    potential_red = [cnt for cnt in red_contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
    potential_green = [cnt for cnt in green_contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]

    # STEP 3: Filter potential obstacles based on their overlap with the track mask.
    on_track_red_obstacles = []
    for contour in potential_red:
        total_area = cv2.contourArea(contour)
        if total_area == 0: continue
        
        obstacle_mask = np.zeros_like(track_mask)
        cv2.drawContours(obstacle_mask, [contour], -1, 255, thickness=cv2.FILLED)
        intersection_area = cv2.countNonZero(cv2.bitwise_and(obstacle_mask, track_mask))
        
        if (intersection_area / total_area) >= OBSTACLE_ON_TRACK_THRESHOLD:
            on_track_red_obstacles.append(contour)

    on_track_green_obstacles = []
    for contour in potential_green:
        total_area = cv2.contourArea(contour)
        if total_area == 0: continue
        
        obstacle_mask = np.zeros_like(track_mask)
        cv2.drawContours(obstacle_mask, [contour], -1, 255, thickness=cv2.FILLED)
        intersection_area = cv2.countNonZero(cv2.bitwise_and(obstacle_mask, track_mask))

        if (intersection_area / total_area) >= OBSTACLE_ON_TRACK_THRESHOLD:
            on_track_green_obstacles.append(contour)
            
    # STEP 4: Select the largest valid 'on-track' obstacle.
    chosen_contour, obstacle_center_point, obstacle_target_x = None, None, None
    largest_red = max(on_track_red_obstacles, key=cv2.contourArea) if on_track_red_obstacles else None
    largest_green = max(on_track_green_obstacles, key=cv2.contourArea) if on_track_green_obstacles else None
    
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

    # --- STEERING CALCULATION WITH DEPTH FACTOR (Unchanged) ---
    depth_factor = 1.0 
    if chosen_contour is not None:
        x, y, wc, hc = cv2.boundingRect(chosen_contour)
        obstacle_bottom_y = y + hc
        normalized_depth = obstacle_bottom_y / h
        depth_factor = 1.0 + (DEPTH_IMPORTANCE_FACTOR - 1.0) * normalized_depth

        pX = x + wc // 2
        pY = y + (hc if detection_point_mode == "BOTTOM" else hc // 2)
        obstacle_center_point = (pX, pY)

        if logic_label == "red_obstacle":
            output_mask = binary_red
            error = red_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor
            obstacle_target_x = red_target_x
        else: # Green obstacle
            output_mask = binary_green
            error = green_target_x - pX
            steering_angle = KP_STEERING * error * depth_factor
            obstacle_target_x = green_target_x

    # --- Fallback Logic (Unchanged) ---
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

    # --- FINAL UI OVERLAYS (MODIFIED TO SHOW TRACK) ---
    track_contours_viz, _ = cv2.findContours(track_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(processed, track_contours_viz, -1, (255, 200, 0), 2) # Light blue outline for the track
    
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
        cv2.putText(processed, f"Depth Factor: {depth_factor:.2f}", (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return processed, steering_angle, output_mask, logic_label, 0

# --- MAIN EXECUTION BLOCK ---
if __name__ == "__main__":
    # IMPORTANT: Change this to the path of your image folder
    INPUT_DIR = "WRO_Images" 
    OUTPUT_DIR = "obstacles_on_track"

    # Create the output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output will be saved in: {OUTPUT_DIR}")

    # Find all JPG and PNG images in the input directory
    image_paths = glob.glob(os.path.join(INPUT_DIR, '*.jpg')) + \
                  glob.glob(os.path.join(INPUT_DIR, '*.png'))

    if not image_paths:
        print(f"❌ Error: No .jpg or .png images found in '{INPUT_DIR}'.")
        print("Please make sure the INPUT_DIR variable is set correctly.")
    else:
        print(f"✅ Found {len(image_paths)} images. Starting processing...")

        # Loop over each image, process it, and save the result
        for image_path in image_paths:
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"⚠️ Warning: Could not read image {image_path}. Skipping.")
                continue

            print(f"Processing {os.path.basename(image_path)}...")

            # Process the frame using the main function
            processed_frame, _, _, _, _ = process_frame_for_steering(frame)

            # Construct the output path and save the processed frame
            base_filename = os.path.basename(image_path)
            output_path = os.path.join(OUTPUT_DIR, base_filename)
            cv2.imwrite(output_path, processed_frame)
        
        print("\n🎉 Processing complete!")