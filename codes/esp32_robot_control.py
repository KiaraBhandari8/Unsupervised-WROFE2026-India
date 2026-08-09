#!/usr/bin/env python3
"""
Robot Control with ESP32 Communication
- Color detection runs on Raspberry Pi
- Motor/servo commands sent to ESP32 via serial
"""

import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, Response
import threading
import time
import os
import serial

# ==================== ESP32 COMMUNICATION ====================
ESP32_PORT = '/dev/ttyUSB0'  # Check with 'ls /dev/ttyUSB*' or 'ls /dev/ttyACM*'
ESP32_BAUDRATE = 115200

# Try to connect to ESP32
try:
    ser = serial.Serial(ESP32_PORT, ESP32_BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to reset
    print(f"Connected to ESP32 on {ESP32_PORT}")
    ESP32_CONNECTED = True
except Exception as e:
    print(f"ESP32 not connected: {e}")
    print("Will run in simulation mode")
    ser = None
    ESP32_CONNECTED = False

def send_to_esp32(command):
    """Send command to ESP32"""
    if ser and ESP32_CONNECTED:
        try:
            ser.write(f"{command}\n".encode())
            ser.flush()
            return True
        except Exception as e:
            print(f"Serial error: {e}")
            return False
    return False

# ESP32 Command functions
def esp32_servo(angle):
    """Set servo angle on ESP32"""
    send_to_esp32(f"SERVO:{angle}")

def esp32_forward(speed):
    """Set forward speed on ESP32"""
    send_to_esp32(f"FORWARD:{int(speed*100)}")

def esp32_stop():
    """Stop motors on ESP32"""
    send_to_esp32("STOP")

def esp32_init():
    """Initialize ESP32"""
    send_to_esp32("INIT")

# Fallback functions if no ESP32
if not ESP32_CONNECTED:
    def esp32_servo(angle):
        print(f"[SIM] Servo: {angle}°")
    def esp32_forward(speed):
        print(f"[SIM] Forward: {speed}")
    def esp32_stop():
        print(f"[SIM] Stop")
    def esp32_init():
        print(f"[SIM] Init")

# ==================== ORIGINAL IMPORTS (optional) ====================
try:
    from image_frame_combine_outer_inner_depth_sept06 import process_frame_for_steering
except ImportError:
    process_frame_for_steering = None
    print("Note: image_frame_combine module not found")

# ==================== MAIN SETUP ====================
if __name__ == '__main__':
    print("--- Starting Robot Control System (ESP32) ---")
    esp32_init()
    time.sleep(0.5)

# ==================== GLOBAL VARIABLES ====================
output_frame = None
output_frame_lock = threading.Lock()
camera_thread_stop_event = threading.Event()

latest_lidar_data = {}
lidar_data_lock = threading.Lock()
latest_detections = []

app = Flask(__name__)

# ==================== CONFIGURATION ====================
SERVO_CENTER_ANGLE = 90  # Adjust for your ESP32 setup
STEERING_GAIN = 0.1
ROBOT_MANEUVER_SPEED = 0.65
ROBOT_CRUISE_SPEED = 0.65
ROBOT_SPEED_MAX = 0.65

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

LIDAR_SERVO_MIN_ANGLE = 70
LIDAR_SERVO_MAX_ANGLE = 110

STREAM_VIDEO = True
DEBUG_UI_OVERLAYS = True

class RobotState:
    RED_AVOIDANCE = "RED_AVOIDANCE"
    GREEN_AVOIDANCE = "GREEN_AVOIDANCE"
    STRAIGHT = "STRAIGHT"
    STOP = "STOP"

current_robot_state = RobotState.STRAIGHT

# ==================== COLOR DETECTION ====================
def detect_color(frame):
    """Detect red and green colors"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red detection
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask1 + red_mask2
    
    # Green detection
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    dets = []
    
    # Red contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > 200:
            x, y, w, h = cv2.boundingRect(c)
            roi = red_mask[y:y+h, x:x+w]
            matching_pixels = cv2.countNonZero(roi)
            total_pixels = w * h
            conf = matching_pixels / total_pixels if total_pixels > 0 else 0
            dets.append({'class': 'red', 'conf': conf, 'bbox': [x, y, x+w, y+h]})
    
    # Green contours
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > 200:
            x, y, w, h = cv2.boundingRect(c)
            roi = green_mask[y:y+h, x:x+w]
            matching_pixels = cv2.countNonZero(roi)
            total_pixels = w * h
            conf = matching_pixels / total_pixels if total_pixels > 0 else 0
            dets.append({'class': 'green', 'conf': conf, 'bbox': [x, y, x+w, y+h]})
    
    return dets

# ==================== CAMERA THREAD ====================
def camera_acquisition_thread_func(picam2_instance, stop_event):
    global output_frame, latest_detections
    print("Camera thread started.")
    
    last_det_time = 0
    
    try:
        while not stop_event.is_set():
            captured_frame_rgb = picam2_instance.capture_array()
            frame_bgr = cv2.cvtColor(captured_frame_rgb, cv2.COLOR_RGB2BGR)
            
            # Run color detection
            if time.time() - last_det_time > 0.2:
                latest_detections = detect_color(frame_bgr)
                last_det_time = time.time()
            
            # Draw detection boxes
            for d in latest_detections:
                x1, y1, x2, y2 = d['bbox']
                color = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
                cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), color, 2)
                conf_pct = int(d['conf'] * 100)
                cv2.putText(frame_bgr, f"{d['class']}:{conf_pct}%", (x1, y1-5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
            
            with output_frame_lock:
                output_frame = frame_bgr.copy()
            
            time.sleep(0.03)
            
    except Exception as e:
        print(f"Camera error: {e}")
    finally:
        print("Camera thread stopped.")

# ==================== MAIN CONTROL LOOP ====================
def robot_control_loop():
    global output_frame, current_robot_state

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
        transform=libcamera.Transform(vflip=False, hflip=False)
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
    
    time.sleep(1)

    camera_acquisition_thread = threading.Thread(
        target=camera_acquisition_thread_func,
        args=(picam2, camera_thread_stop_event)
    )
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    print("Starting main control loop...")

    try:
        loop_counter = 0
        
        while True:
            loop_start_time = time.monotonic()
            loop_counter += 1
            
            with output_frame_lock:
                if output_frame is None:
                    time.sleep(0.01)
                    continue
                frame_bgr = output_frame.copy()
            
            # Get detections
            detections = latest_detections.copy()
            
            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = "GO STRAIGHT"
            
            # === COLOR DETECTION LOGIC ===
            # Red → pass on RIGHT (steer right)
            # Green → pass on LEFT (steer left)
            # None → go straight
            
            red_near = None
            green_near = None
            
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                box_width = x2 - x1
                if box_width > 20:  # Close enough
                    if det['class'] == 'red':
                        red_near = det
                    elif det['class'] == 'green':
                        green_near = det
            
            if red_near is not None:
                # RED: steer RIGHT to pass on right
                current_robot_state = RobotState.RED_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE + 25
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = f"RED >> STEER RIGHT"
                print(f">>> RED DETECTED! Steering RIGHT")
                
            elif green_near is not None:
                # GREEN: steer LEFT to pass on left
                current_robot_state = RobotState.GREEN_AVOIDANCE
                target_servo_angle = SERVO_CENTER_ANGLE - 25
                robot_speed_current = ROBOT_MANEUVER_SPEED
                display_text = f"GREEN << STEER LEFT"
                print(f">>> GREEN DETECTED! Steering LEFT")
                
            else:
                # No detection - go straight
                current_robot_state = RobotState.STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                robot_speed_current = ROBOT_CRUISE_SPEED
                display_text = "GO STRAIGHT"

            # === APPLY TO ESP32 ===
            final_angle = int(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE))
            esp32_servo(final_angle)
            esp32_forward(robot_speed_current)
            
            # === UI OVERLAY ===
            cv2.putText(frame_bgr, display_text, (10, 15), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            if STREAM_VIDEO:
                with output_frame_lock:
                    output_frame = frame_bgr.copy()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        esp32_stop()
        camera_thread_stop_event.set()
        camera_acquisition_thread.join()
        picam2.stop()
        
        if ser:
            ser.close()
        print("Done.")

# ==================== FLASK STREAMS ====================
def generate_frames():
    global output_frame
    while True:
        if not STREAM_VIDEO:
            time.sleep(0.5)
            continue
        with output_frame_lock:
            if output_frame is None:
                time.sleep(0.01)
                continue
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not flag:
                continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

@app.route("/")
def index():
    return '''<!DOCTYPE html>
<html>
<head>
    <title>Robot Control (ESP32)</title>
    <style>
        body { margin: 0; background: #111; display: flex; flex-direction: column; align-items: center; font-family: monospace; color: #0f0; }
        h1 { margin: 10px; }
        img { max-width: 100%; max-height: 70vh; border: 2px solid #0f0; }
        .info { margin: 10px; padding: 10px; background: #222; border-radius: 5px; }
    </style>
</head>
<body>
    <h1>ESP32 Robot Control</h1>
    <img src="/video_feed">
    <div class="info">
        Red → Steer RIGHT | Green → Steer LEFT | None → Straight<br>
        ESP32: /dev/ttyUSB0 | Port 5000
    </div>
</body>
</html>'''

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ==================== MAIN ====================
if __name__ == '__main__':
    print("="*50)
    print("Robot Control with ESP32")
    print("="*50)
    
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()
    
    print(f"Starting web server on http://192.168.1.100:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)