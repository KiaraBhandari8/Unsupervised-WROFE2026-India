#          CALIBRATED RED HSV RESULTS          
#==========================================
#LOWER BOUND array format: np.array([0, 169, 73])
#UPPER BOUND array format: np.array([179, 255, 255])
#==========================================
#Raw Channel Breakdown -> H: 0-179 | S: 7-209 | V: 103-204


#           CALIBRATED GREEN HSV RESULTS          
# ==========================================
# LOWER BOUND array format: np.array([32, 146, 40])
# UPPER BOUND array format: np.array([95, 255, 255])
# ==========================================
# Raw Channel Breakdown -> H: 32-95 | S: 12-186 | V: 28-204




import time
import cv2
import numpy as np
from picamera2 import Picamera2

def main():
    print("=== WRO Headless HSV Color Calibration Tool ===")
    print("[1] Place your pillar directly in front of the camera lens.")
    print("[2] The script will sample a 50x50 pixel square in the dead center.")
    
    # Initialize your Pi Camera Module 3 Wide
    picam2 = Picamera2()
    camera_config = picam2.create_video_configuration(main={"size": (768, 432)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Allow automatic white balance and exposure to lock down
    time.sleep(2.5)
    
    try:
        # Capture a crisp test frame
        raw_frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)
        
        # Convert to HSV color layout space
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        # Define a 50x50 target window at the exact center of your 768x432 matrix
        center_x, center_y = 384, 216
        box_radius = 25
        
        y_start, y_end = center_y - box_radius, center_y + box_radius
        x_start, x_end = center_x - box_radius, center_x + box_radius
        
        # Extract the pixels inside the target container
        sample_zone = hsv_frame[y_start:y_end, x_start:x_end]
        
        # Compute the absolute mathematical minimum and maximum ranges present
        h_channel = sample_zone[:, :, 0]
        s_channel = sample_zone[:, :, 1]
        v_channel = sample_zone[:, :, 2]
        
        min_h, max_h = np.min(h_channel), np.max(h_channel)
        min_s, max_s = np.min(s_channel), np.max(s_channel)
        min_v, max_v = np.min(v_channel), np.max(v_channel)
        
        # Draw the target box on the BGR frame for physical alignment verification
        cv2.rectangle(frame_bgr, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
        cv2.circle(frame_bgr, (center_x, center_y), 3, (0, 0, 255), -1)
        cv2.imwrite("hsv_calibration_alignment.jpg", frame_bgr)
        
        print("\n==========================================")
        print("          CALIBRATED HSV RESULTS          ")
        print("==========================================")
        print(f"LOWER BOUND array format: np.array([{min_h}, {max_s - 40 if max_s > 100 else 60}, {min_v - 30 if min_v > 50 else 40}])")
        print(f"UPPER BOUND array format: np.array([{max_h}, 255, 255])")
        print("==========================================")
        print(f"Raw Channel Breakdown -> H: {min_h}-{max_h} | S: {min_s}-{max_s} | V: {min_v}-{max_v}")
        print("[SUCCESS] Check 'hsv_calibration_alignment.jpg' to verify alignment correctness.")

    except Exception as e:
        print(f"[CRITICAL ERROR] Extraction thread failed: {e}")
    finally:
        picam2.close()

if __name__ == "__main__":
    main()