#!/usr/bin/env python3
import cv2, numpy as np, time
from picamera2 import Picamera2

print("[INIT] Camera sensor...", flush=True)
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(2)
print("[OK] Camera ready\n", flush=True)

blue_low = np.array([80, 110, 50])
blue_high = np.array([130, 255, 255])
orange_low = np.array([5, 100, 20])
orange_high = np.array([15, 255, 255])

def find_contour_center(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    area = cv2.contourArea(largest)
    if M["m00"] == 0:
        return None, area
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy), area

print("=== LIVE COLOR TRACKING ===")
print("Frame: 640x480 | Center x=320\n")
print(f"{'Color':<8} {'Area%':<8} {'PosX':<8} {'PosY':<8} {'Steer':<8}")
print("-" * 45)

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        b_mask = cv2.inRange(hsv, blue_low, blue_high)
        o_mask = cv2.inRange(hsv, orange_low, orange_high)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        b_mask = cv2.morphologyEx(b_mask, cv2.MORPH_OPEN, kernel)
        o_mask = cv2.morphologyEx(o_mask, cv2.MORPH_OPEN, kernel)

        b_pos, b_area = find_contour_center(b_mask)
        o_pos, o_area = find_contour_center(o_mask)

        b_pct = b_area / (640 * 480) * 100
        o_pct = o_area / (640 * 480) * 100

        if b_pos:
            b_steer = (b_pos[0] - 320) / 320 * 45
        else:
            b_steer = 0
        if o_pos:
            o_steer = (o_pos[0] - 320) / 320 * 45
        else:
            o_steer = 0

        print(f"{'BLUE':<8} {b_pct:6.2f}%  {str(b_pos[0] if b_pos else 0):<8} {str(b_pos[1] if b_pos else 0):<8} {b_steer:>+5.0f} deg")
        print(f"{'ORANGE':<8} {o_pct:6.2f}%  {str(o_pos[0] if o_pos else 0):<8} {str(o_pos[1] if o_pos else 0):<8} {o_steer:>+5.0f} deg")
        print("-" * 45)

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n[STOP] Camera off")
    picam2.stop()
