#!/usr/bin/env python3
"""
Robot Control with ESP32 + YOLO Pillar Detection + HSV Fallback + LIDAR
Single-threaded architecture - no thread contention
"""
import cv2
import numpy as np
from flask import Flask, Response
import threading
import time
import os

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from picamera2 import Picamera2
    import libcamera
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

try:
    import ncnn
    NCNN_AVAILABLE = True
except ImportError:
    NCNN_AVAILABLE = False

SERVO_CENTER = 102
SERVO_MIN = 77
SERVO_MAX = 127
CRUISE_SPEED = 0.8
MANEUVER_SPEED = 0.9
CAM_W = 640
CAM_H = 480
STREAM_FPS = 8
CMD_HZ = 20

LIDAR_SAFETY_MM = 300
LIDAR_CAUTION_MM = 600
LIDAR_MIN_ANGLE = -60
LIDAR_MAX_ANGLE = 60
DIST_WEIGHT = 1.5
ANGLE_WEIGHT = 0.8
STEER_SMOOTH = 0.3

YOLO_PARAM = '/home/pi8/yolo_model/pillar_640_v2/model.ncnn.param'
YOLO_BIN = '/home/pi8/yolo_model/pillar_640_v2/model.ncnn.bin'
YOLO_INPUT_SIZE = 640
YOLO_CONF_THRESHOLD = 0.20
YOLO_CLASSES = {0: 'red', 1: 'green'}

LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])
LOWER_GREEN = np.array([40, 50, 50])
UPPER_GREEN = np.array([80, 255, 255])

PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

ser = None
ESP32_OK = False

if SERIAL_AVAILABLE:
    try:
        ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.1)
        time.sleep(2)
        ESP32_OK = True
        print(f"[SERIAL] ESP32 on {PI_TO_ESP_PORT}")
    except Exception as e:
        print(f"[SERIAL] Fail: {e}")
else:
    print("[SERIAL] pyserial not available")

def cmd(angle, speed):
    if ser and ESP32_OK:
        packet = f"STR:{angle},SPD:{int(speed * 255)}\n"
        ser.write(packet.encode())
        ser.flush()

def stop():
    if ser and ESP32_OK:
        ser.write(f"STR:{SERVO_CENTER},SPD:0\n".encode())
        ser.flush()

def startup_test():
    if not (ser and ESP32_OK):
        print("[STARTUP] Skipped - no serial")
        return
    print("[STARTUP] Sweeping servo...")
    for angle in [SERVO_CENTER - 20, SERVO_CENTER, SERVO_CENTER + 20, SERVO_CENTER]:
        ser.write(f"STR:{angle},SPD:0\n".encode())
        ser.flush()
        time.sleep(0.3)
    print("[STARTUP] Motor pulse...")
    ser.write(f"STR:{SERVO_CENTER},SPD:{int(0.4 * 255)}\n".encode())
    ser.flush()
    time.sleep(0.4)
    stop()
    print("[STARTUP] Done")

def detect_hsv(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1) + cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    dets = []
    for mask, cls in [(red, 'red'), (green, 'green')]:
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in conts:
            if cv2.contourArea(c) > 150:
                x, y, w, h = cv2.boundingRect(c)
                roi = mask[y:y+h, x:x+w]
                conf = cv2.countNonZero(roi) / (w * h) if w * h > 0 else 0
                dets.append({'class': cls, 'conf': conf, 'bbox': [x, y, x+w, y+h], 'width': w})
    return dets

yolo_net = None

def load_yolo():
    global yolo_net
    if not NCNN_AVAILABLE:
        print("[YOLO] ncnn not available")
        return False
    if not os.path.exists(YOLO_PARAM):
        print(f"[YOLO] Model not found at {YOLO_PARAM}")
        return False
    print("[YOLO] Loading model...")
    yolo_net = ncnn.Net()
    yolo_net.opt.num_threads = 4
    yolo_net.load_param(YOLO_PARAM)
    yolo_net.load_model(YOLO_BIN)
    print("[YOLO] Loaded!")
    return True

def detect_yolo(img):
    if yolo_net is None:
        return None
    h, w = img.shape[:2]
    scale = YOLO_INPUT_SIZE / max(h, w)
    nw, nh = int(w * scale), int(h * scale)
    r = cv2.resize(img, (nw, nh))
    p = np.zeros((YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, 3), dtype=np.uint8)
    dw = (YOLO_INPUT_SIZE - nw) // 2
    dh = (YOLO_INPUT_SIZE - nh) // 2
    p[dh:dh+nh, dw:dw+nw] = r
    mat = ncnn.Mat.from_pixels(p, ncnn.Mat.PixelType.PIXEL_BGR2RGB, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE)
    mat.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
    ex = yolo_net.create_extractor()
    ex.set_light_mode(True)
    ex.input("in0", mat)
    _, out = ex.extract("out0")
    output = np.array(out).T
    n_cells = output.shape[0]
    if n_cells == 6:
        output = output.T
    n_cells = output.shape[0]
    class_cols = output[:, 4:]
    max_scores = np.max(class_cols, axis=1)
    scale_inv = w / (YOLO_INPUT_SIZE - 2 * dw)
    dets = []
    for i in np.where(max_scores >= YOLO_CONF_THRESHOLD)[0]:
        conf = float(max_scores[i])
        cls_idx = int(np.argmax(class_cols[i]))
        cls = YOLO_CLASSES[cls_idx]
        x1 = int((output[i, 0] - output[i, 2]/2 - dw) * scale_inv)
        y1 = int((output[i, 1] - output[i, 3]/2 - dh) * scale_inv)
        x2 = int((output[i, 0] + output[i, 2]/2 - dw) * scale_inv)
        y2 = int((output[i, 1] + output[i, 3]/2 - dh) * scale_inv)
        dets.append({'class': cls, 'confidence': conf, 'bbox': [x1, y1, x2, y2], 'width': x2 - x1, 'center_x': (x1 + x2) // 2})
    if len(dets) > 1:
        keep = cv2.dnn.NMSBoxes(
            [d['bbox'] for d in dets],
            [d['confidence'] for d in dets],
            0.1, 0.5
        )
        if len(keep) > 0:
            dets = [dets[i] for i in keep.flatten()]
    return dets if dets else []

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
            if latest_frame is None:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
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
<b>PRIORITY:</b><br>1. YOLO RED -> Steer RIGHT<br>2. YOLO GREEN -> Steer LEFT<br>3. HSV Fallback<br>4. LIDAR -> Weighted steering<br>5. NONE -> Straight</div></body></html>'''

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    return f'{{"state":"{state_text}","gyro_z":{gyro["gz"]}}}'

def sim_lidar():
    import random
    return {a: random.randint(800, 2000) for a in range(-60, 61, 5)}

def main():
    global latest_frame, gyro, state_text

    cap = None
    picam = None
    if PICAMERA_AVAILABLE:
        try:
            picam = Picamera2()
            cfg = picam.create_preview_configuration(main={"size": (CAM_W, CAM_H)},
                transform=libcamera.Transform())
            picam.configure(cfg)
            picam.start()
            print(f"[CAM] Pi Camera {CAM_W}x{CAM_H}")
        except Exception as e:
            print(f"[CAM] Pi Camera fail: {e}")
            picam = None
    if picam is None:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            print(f"[CAM] Webcam {CAM_W}x{CAM_H}")
        else:
            print("[CAM] No camera!")

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

    yolo_ok = load_yolo()
    startup_test()
    print("[MAIN] Loop running")
    prev_steer = 0
    last_det = 0.0
    last_cmd = 0.0
    detections = []
    yolo_active = True

    try:
        while not stop_event.is_set():
            t = time.time()

            frame = None
            if picam:
                try:
                    rgb = picam.capture_array()
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                except:
                    pass
            elif cap and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    frame = None

            read_ser()

            if frame is not None and t - last_det > 0.2:
                det_t0 = time.time()
                if yolo_ok:
                    yolo_dets = detect_yolo(frame)
                    if yolo_dets is not None and len(yolo_dets) > 0:
                        detections = yolo_dets
                        yolo_active = True
                    elif yolo_dets is not None and len(yolo_dets) == 0:
                        hsv_dets = detect_hsv(frame)
                        hsv_large = [d for d in hsv_dets if d['width'] > 25]
                        if hsv_large:
                            detections = [{'class': d['class'], 'confidence': d['conf'],
                                           'bbox': d['bbox'], 'width': d['width'],
                                           'center_x': (d['bbox'][0] + d['bbox'][2]) // 2} for d in hsv_large]
                            yolo_active = False
                        else:
                            detections = []
                    else:
                        detections = []
                else:
                    detections = detect_hsv(frame)
                    detections = [d for d in detections if d['width'] > 25]
                    yolo_active = False
                last_det = t
                det_elapsed = time.time() - det_t0
                for d in detections:
                    print(f"[DET] {d['class']} conf={d.get('confidence', d.get('conf', 0)):.2f} w={d.get('width',0)}")
                if det_elapsed > 0.3:
                    print(f"[SLOW] detection: {det_elapsed:.2f}s")

            target_angle = SERVO_CENTER
            speed = CRUISE_SPEED
            txt = "STRAIGHT"

            lidar = sim_lidar()
            reds = [d for d in detections if d['class'] == 'red' and d.get('width', 0) > 25]
            greens = [d for d in detections if d['class'] == 'green' and d.get('width', 0) > 25]

            if reds:
                target_angle = SERVO_CENTER + 30
                speed = MANEUVER_SPEED
                txt = "RED>>RIGHT"
            elif greens:
                target_angle = SERVO_CENTER - 30
                speed = MANEUVER_SPEED
                txt = "GREEN<<LEFT"
            else:
                off, safe = weighted_steer(lidar)
                if not safe:
                    speed = 0
                    txt = "STOP"
                    stop()
                else:
                    off = int(prev_steer * (1 - STEER_SMOOTH) + off * STEER_SMOOTH)
                    prev_steer = off
                    target_angle = SERVO_CENTER + off
                    if off != 0:
                        txt = f"LIDAR {'RIGHT' if off>0 else 'LEFT'} {abs(off)}deg"

            final_angle = int(np.clip(target_angle, SERVO_MIN, SERVO_MAX))
            state_text = txt

            if t - last_cmd > 1.0 / CMD_HZ:
                cmd(final_angle, speed)
                last_cmd = t
                if txt != "STRAIGHT" or int(t) % 2 == 0:
                    print(f"[CMD] STR:{final_angle} SPD:{int(speed*255)} | {txt}")

            if frame is not None:
                method = "YOLO" if yolo_active else "HSV"
                cv2.putText(frame, f"{txt} [{method}]", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.putText(frame, f"GyroZ:{gyro['gz']:.1f}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
                cv2.putText(frame, f"Angle:{final_angle}", (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
                for d in detections:
                    x1, y1, x2, y2 = d['bbox'][0], d['bbox'][1], d['bbox'][2], d['bbox'][3]
                    clr = (0, 0, 255) if d['class'] == 'red' else (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
                    conf = d.get('confidence', d.get('conf', 0))
                    cv2.putText(frame, f"{d['class']}:{int(conf*100)}%", (x1, y1-3),
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

if __name__ == '__main__':
    print("="*50)
    print("ESP32 Robot Control v3 (YOLO+HSV+LIDAR)")
    print("="*50)
    t = threading.Thread(target=main, daemon=True)
    t.start()
    print(f"Web: http://192.168.1.100:5000  |  Status: http://192.168.1.100:5000/status")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
