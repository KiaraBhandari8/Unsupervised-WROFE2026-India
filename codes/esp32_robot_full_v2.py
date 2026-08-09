#!/usr/bin/env python3
"""
Robot Control with ESP32 + YOLO Pillar Detection + LIDAR Weighted Steering
Single-threaded architecture - no thread contention
v2: Fixed serial port (hardcoded /dev/ttyAMA0), removed silent failures,
    added startup servo/motor test, fixed CAM resolution to 320x240
"""
import cv2
import numpy as np
from flask import Flask, Response
import ncnn
import threading
import time
import os

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from picamera2 import Picamera2
    import libcamera
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

# ==================== CONFIGURATION ====================
SERVO_CENTER = 102
SERVO_MIN = 77
SERVO_MAX = 127
CRUISE_SPEED = 0.60
MANEUVER_SPEED = 0.80
CAM_W = 320
CAM_H = 240
STREAM_FPS = 8
CMD_HZ = 20

LIDAR_SAFETY_MM = 300
LIDAR_CAUTION_MM = 600
LIDAR_MIN_ANGLE = -60
LIDAR_MAX_ANGLE = 60
DIST_WEIGHT = 1.5
ANGLE_WEIGHT = 0.8
STEER_SMOOTH = 0.3

# ==================== YOLO MODEL CONFIG ====================
YOLO_PARAM = '/home/pi8/yolo_model/finetune_ncnn/model_480.param'
YOLO_BIN = '/home/pi8/yolo_model/finetune_ncnn/model_480.bin'
YOLO_INPUT_SIZE = 480
YOLO_CONF_THRESHOLD = 0.15
YOLO_CLASSES = {0: 'red', 1: 'green'}

# ==================== SERIAL SETUP ====================
# Hardcoded to /dev/ttyAMA0 — the Pi's hardware UART wired to ESP32 Serial2 (GPIO16/17)
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ser = None
ESP32_OK = False

if SERIAL_AVAILABLE:
    try:
        ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.1)
        time.sleep(2)
        ESP32_OK = True
        print(f"[SERIAL] ESP32 connected on {PI_TO_ESP_PORT}")
    except Exception as e:
        print(f"[SERIAL ERROR] Failed to open {PI_TO_ESP_PORT}: {e}")
        print("[SERIAL] Motor and servo commands will NOT be sent.")
else:
    print("[SERIAL] pyserial not available — running in simulation mode.")

def cmd(angle, speed):
    if ser and ESP32_OK:
        packet = f"STR:{angle},SPD:{int(speed * 255)}\n"
        ser.write(packet.encode('utf-8'))
        ser.flush()
    # no silent sim — if serial is down the user already saw the error above

def stop():
    if ser and ESP32_OK:
        packet = f"STR:{SERVO_CENTER},SPD:0\n"
        ser.write(packet.encode('utf-8'))
        ser.flush()

def startup_test():
    """Sweep servo and pulse motor briefly to confirm hardware is alive."""
    if not (ser and ESP32_OK):
        print("[STARTUP TEST] Skipped — serial not connected.")
        return
    print("[STARTUP TEST] Sweeping servo left -> center -> right ...")
    for angle in [SERVO_CENTER - 20, SERVO_CENTER, SERVO_CENTER + 20, SERVO_CENTER]:
        packet = f"STR:{angle},SPD:0\n"
        ser.write(packet.encode('utf-8'))
        ser.flush()
        time.sleep(0.3)
    print("[STARTUP TEST] Brief motor pulse ...")
    ser.write(f"STR:{SERVO_CENTER},SPD:{int(0.4 * 255)}\n".encode('utf-8'))
    ser.flush()
    time.sleep(0.4)
    stop()
    print("[STARTUP TEST] Done — hardware confirmed alive.")

# ==================== YOLO DETECTION ====================
yolo_net = None

def load_yolo_model():
    global yolo_net
    print("Loading YOLO model...")
    yolo_net = ncnn.Net()
    yolo_net.opt.use_vulkan_compute = False
    yolo_net.opt.num_threads = 4
    yolo_net.load_param(YOLO_PARAM)
    yolo_net.load_model(YOLO_BIN)
    print("YOLO model loaded!")

def detect_yolo(img):
    if yolo_net is None:
        return []
    h, w = img.shape[:2]
    scale = YOLO_INPUT_SIZE / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    padded = np.zeros((YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, 3), dtype=np.uint8)
    dw, dh = (YOLO_INPUT_SIZE - new_w) // 2, (YOLO_INPUT_SIZE - new_h) // 2
    padded[dh:dh + new_h, dw:dw + new_w] = resized
    mat = ncnn.Mat.from_pixels(padded, ncnn.Mat.PixelType.PIXEL_BGR2RGB, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE)
    mat.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
    ex = yolo_net.create_extractor()
    ex.set_light_mode(True)
    ex.input("in0", mat)
    _, output = ex.extract('out0')
    output = np.array(output).reshape(6, 8400).T
    class_scores = output[:, 4:]
    max_scores = np.max(class_scores, axis=1)
    scale_inv = w / (YOLO_INPUT_SIZE - 2 * dw)
    dets = []
    for i in np.where(max_scores >= YOLO_CONF_THRESHOLD)[0]:
        conf = float(max_scores[i])
        cls_idx = int(np.argmax(class_scores[i]))
        cls = YOLO_CLASSES[cls_idx]
        x1 = int((output[i, 0] - output[i, 2]/2 - dw) * scale_inv)
        y1 = int((output[i, 1] - output[i, 3]/2 - dh) * scale_inv)
        x2 = int((output[i, 0] + output[i, 2]/2 - dw) * scale_inv)
        y2 = int((output[i, 1] + output[i, 3]/2 - dh) * scale_inv)
        dets.append({'class': cls, 'confidence': conf, 'bbox': [x1, y1, x2, y2], 'center_x': (x1 + x2) // 2})
    return dets

def weighted_steer(scan):
    if not scan:
        return 0, True
    obs = []
    for a, d in scan.items():
        if LIDAR_MIN_ANGLE <= a <= LIDAR_MAX_ANGLE and 0 < d < LIDAR_CAUTION_MM:
            obs.append({'angle': a, 'dist': d, 'sev': (LIDAR_CAUTION_MM - d) / LIDAR_CAUTION_MM})
    if not obs:
        return 0, True
    tw, wa = 0, 0
    for o in obs:
        w = o['sev'] * DIST_WEIGHT + abs(o['angle']) * ANGLE_WEIGHT
        wa += o['angle'] * w
        tw += w
    wa = wa / tw if tw > 0 else 0
    safe = min(o['dist'] for o in obs) > LIDAR_SAFETY_MM
    off = int(np.clip(wa * 2, -40, 40))
    return off, safe

# ==================== GLOBALS ====================
frame_lock = threading.Lock()
latest_frame = None
gyro = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}
state_text = "INIT"
stop_event = threading.Event()

app = Flask(__name__)

def gen_frames():
    while not stop_event.is_set():
        t0 = time.time()
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
        if frame is None:
            time.sleep(0.05)
            continue
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            time.sleep(0.05)
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        el = time.time() - t0
        if el < 1.0 / STREAM_FPS:
            time.sleep(1.0 / STREAM_FPS - el)

@app.route('/')
def index():
    return '''<!DOCTYPE html><html><head><title>ESP32 Robot</title>
<style>body{margin:0;background:#111;display:flex;flex-direction:column;align-items:center;font-family:monospace;color:#0f0}
img{max-width:100%;max-height:70vh;border:2px solid #0f0}.info{margin:10px;padding:10px;background:#222;border-radius:5px;max-width:600px;font-size:12px;color:#ff0}</style>
</head><body><h1>Robot Control</h1><img src="/video_feed"><div class="info">
<b>PRIORITY:</b><br>1. RED -> Steer RIGHT<br>2. GREEN -> Steer LEFT<br>3. LIDAR -> Weighted steering<br>4. NONE -> Straight</div></body></html>'''

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    return f'{{"state":"{state_text}","gyro_z":{gyro["gz"]}}}'

# ==================== SIMULATED LIDAR ====================
def sim_lidar():
    import random
    return {a: random.randint(800, 2000) for a in range(-60, 61, 5)}

# ==================== MAIN LOOP ====================
def main():
    global latest_frame, gyro, state_text
    
    # --- Camera init ---
    cap = None
    picam = None
    if PICAMERA_AVAILABLE:
        try:
            picam = Picamera2()
            cfg = picam.create_preview_configuration(main={"size": (CAM_W, CAM_H)},
                transform=libcamera.Transform())
            picam.configure(cfg)
            picam.start()
            print(f"Pi Camera {CAM_W}x{CAM_H}")
        except Exception as e:
            print(f"Pi Camera fail: {e}")
            picam = None
    if picam is None:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            print(f"Webcam {CAM_W}x{CAM_H}")
        else:
            print("No camera!")
    
    # --- Serial read buffer ---
    def read_ser():
        global gyro
        while ser and ESP32_OK and ser.in_waiting:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if line.startswith('DATA:'):
                    parts = line[5:].split(',')
                    if len(parts) >= 6:
                        gyro = {k: float(v) for k, v in zip(['ax','ay','az','gx','gy','gz'], parts[:6])}
            except:
                pass
    
    load_yolo_model()
    startup_test()
    print("[MAIN] Loop running (single-threaded)")
    prev_steer = 0
    last_det = 0.0
    last_cmd = 0.0
    last_stream = 0.0
    detections = []
    
    try:
        while not stop_event.is_set():
            t = time.time()

            # 1. Capture frame (non-blocking — motor runs even if camera fails)
            frame = None
            if picam:
                try:
                    rgb = picam.capture_array()
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                except Exception as e:
                    print(f"[CAM] Capture error: {e}")
            elif cap and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    frame = None

            # 2. Read serial (gyro feedback from ESP32)
            read_ser()

            # 3. YOLO detection (~5 Hz) — only if we have a frame
            if frame is not None and t - last_det > 0.2:
                det_t0 = time.time()
                detections = detect_yolo(frame)
                det_elapsed = time.time() - det_t0
                last_det = t
                large = [d for d in detections if (d['bbox'][2] - d['bbox'][0]) > 25]
                if large:
                    for d in large:
                        w = d['bbox'][2] - d['bbox'][0]
                        print(f"[YOLO] {d['class']} conf={d['confidence']:.2f} w={w}px")
                if det_elapsed > 0.6:
                    print(f"[YOLO] Slow inference: {det_elapsed:.2f}s")

            # 4. Decision: motor always runs forward, steer based on pillar colour
            target_angle = SERVO_CENTER
            speed = CRUISE_SPEED
            txt = "STRAIGHT"

            reds   = [d for d in detections if d['class'] == 'red'   and (d['bbox'][2] - d['bbox'][0]) > 25]
            greens = [d for d in detections if d['class'] == 'green' and (d['bbox'][2] - d['bbox'][0]) > 25]

            if reds:
                target_angle = SERVO_CENTER + 30   # steer right to pass red on the right
                speed = MANEUVER_SPEED
                txt = "RED>>RIGHT"
            elif greens:
                target_angle = SERVO_CENTER - 30   # steer left to pass green on the left
                speed = MANEUVER_SPEED
                txt = "GREEN<<LEFT"

            final_angle = int(np.clip(target_angle, SERVO_MIN, SERVO_MAX))
            state_text = txt

            # 5. Send command at CMD_HZ
            if t - last_cmd > 1.0 / CMD_HZ:
                cmd(final_angle, speed)
                last_cmd = t
                print(f"[CMD] STR:{final_angle} SPD:{int(speed*255)} | {txt}")

            # 6. Draw overlay and update stream (only when camera is working)
            if frame is not None:
                cv2.putText(frame, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.putText(frame, f"GyroZ:{gyro['gz']:.1f}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
                cv2.putText(frame, f"Angle:{final_angle}", (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
                for d in detections:
                    x1, y1, x2, y2 = d['bbox']
                    clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
                    cv2.putText(frame, f"{d['class']}:{int(d['confidence']*100)}%", (x1, y1-3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, clr, 1)
                with frame_lock:
                    latest_frame = frame.copy()
            else:
                time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        stop()
        stop_event.set()
        if picam:
            picam.stop()
        if cap:
            cap.release()
        if ser:
            ser.close()
        print("Done")

# ==================== START ====================
if __name__ == '__main__':
    print("="*50)
    print("ESP32 Robot Control (single-thread)")
    print("="*50)
    t = threading.Thread(target=main, daemon=True)
    t.start()
    print(f"Web: http://192.168.1.100:8080  |  Status: http://192.168.1.100:8080/status")
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
