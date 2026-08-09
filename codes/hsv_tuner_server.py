import cv2
import numpy as np
import threading
import time
import os
from flask import Flask, Response, jsonify, request, send_file
from picamera2 import Picamera2
import libcamera

CAMERA_RESOLUTION = (1640, 1232)
CAMERA_FRAMERATE = 30.0
CAMERA_BUFFER_COUNT = 4
PROCESSING_WIDTH = 640
PROCESSING_HEIGHT = 480

RED_LOWER = np.array([175, 70, 50])
RED_UPPER = np.array([192, 210, 255])
GREEN_LOWER = np.array([50, 80, 90])
GREEN_UPPER = np.array([70, 255, 205])
BLUE_LOWER = np.array([100, 50, 50])
BLUE_UPPER = np.array([130, 255, 255])
ORANGE_LOWER = np.array([13, 50, 50])
ORANGE_UPPER = np.array([37, 255, 255])
WHITE_LOWER = np.array([0, 0, 100])
WHITE_UPPER = np.array([179, 30, 255])

hsv_lock = threading.Lock()
output_frame = None
output_frame_lock = threading.Lock()
stop_event = threading.Event()

app = Flask(__name__)

def capture_loop():
    global output_frame, output_frame_lock
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    processing_size = (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    print("Camera started.")
    try:
        while not stop_event.is_set():
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(cv2.resize(frame_rgb, processing_size, interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2BGR)
            with output_frame_lock:
                output_frame = frame_bgr.copy()
    except Exception as e:
        print(f"Capture error: {e}")
    finally:
        picam2.stop()
        print("Camera stopped.")

def generate_frames():
    global output_frame, output_frame_lock
    while not stop_event.is_set():
        with output_frame_lock:
            if output_frame is None:
                time.sleep(0.01)
                continue
            flag, encoded = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + bytearray(encoded) + b'\r\n')

@app.route("/")
def index():
    return send_file(os.path.join(os.path.dirname(os.path.abspath(__file__)), "index_test.html"), mimetype='text/html')

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/hsv", methods=["GET"])
def get_hsv():
    with hsv_lock:
        data = {
            "red_lower": RED_LOWER.tolist(),
            "red_upper": RED_UPPER.tolist(),
            "green_lower": GREEN_LOWER.tolist(),
            "green_upper": GREEN_UPPER.tolist(),
            "blue_lower": BLUE_LOWER.tolist(),
            "blue_upper": BLUE_UPPER.tolist(),
            "orange_lower": ORANGE_LOWER.tolist(),
            "orange_upper": ORANGE_UPPER.tolist(),
            "white_lower": WHITE_LOWER.tolist(),
            "white_upper": WHITE_UPPER.tolist(),
        }
    return jsonify(data)

@app.route("/api/hsv", methods=["POST"])
def set_hsv():
    data = request.get_json()
    if not data:
        return jsonify({"error": "no data"}), 400
    with hsv_lock:
        for key, arr in [("red_lower", RED_LOWER), ("red_upper", RED_UPPER),
                          ("green_lower", GREEN_LOWER), ("green_upper", GREEN_UPPER),
                          ("blue_lower", BLUE_LOWER), ("blue_upper", BLUE_UPPER),
                          ("orange_lower", ORANGE_LOWER), ("orange_upper", ORANGE_UPPER),
                          ("white_lower", WHITE_LOWER), ("white_upper", WHITE_UPPER)]:
            if key in data:
                arr[:] = np.clip(data[key], 0, 255).astype(np.uint8)
    return jsonify({"status": "ok"})

if __name__ == '__main__':
    cap_thread = threading.Thread(target=capture_loop, daemon=True)
    cap_thread.start()
    print("HSV Tuner Server starting on http://0.0.0.0:8000")
    try:
        app.run(host='0.0.0.0', port=8000, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_event.set()
        cap_thread.join(timeout=3)
