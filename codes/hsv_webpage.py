from flask import Flask, render_template, Response, request, jsonify
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import json
import time

app = Flask(__name__)

# =========================
# CAMERA
# =========================

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

latest_frame = None
frame_lock = threading.Lock()

def camera_worker():
    global latest_frame
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with frame_lock:
            latest_frame = frame

threading.Thread(target=camera_worker, daemon=True).start()

# =========================
# TUNING DATA
# =========================

active_mode = "track"

presets = {
    "track": {"h_min":0,"h_max":179,"s_min":0,"s_max":80,"v_min":0,"v_max":80},
    "red": {"l_min":0,"l_max":255,"a_min":140,"a_max":255,"b_min":100,"b_max":255},
    "blue": {"l_min":0,"l_max":255,"a_min":0,"a_max":140,"b_min":0,"b_max":120},
    "green": {"l_min":0,"l_max":255,"a_min":0,"a_max":120,"b_min":80,"b_max":200},
    "orange": {"l_min":0,"l_max":255,"a_min":140,"a_max":255,"b_min":140,"b_max":255},
    "white": {"l_min":180,"l_max":255,"a_min":0,"a_max":255,"b_min":0,"b_max":255}
}

# =========================
# PROCESSING
# =========================

def build_mask(frame):
    global active_mode

    if active_mode == "track":
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        p = presets["track"]
        lower = np.array([p["h_min"], p["s_min"], p["v_min"]])
        upper = np.array([p["h_max"], p["s_max"], p["v_max"]])
        return cv2.inRange(hsv, lower, upper)

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l,a,b = cv2.split(lab)

    p = presets[active_mode]

    mask = cv2.inRange(
        lab,
        np.array([p["l_min"], p["a_min"], p["b_min"]]),
        np.array([p["l_max"], p["a_max"], p["b_max"]])
    )
    return mask

def process_frame(frame):
    mask = build_mask(frame)

    segmented = cv2.bitwise_and(frame, frame, mask=mask)

    debug = frame.copy()

    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        if area < 300:
            continue

        x,y,w,h = cv2.boundingRect(c)

        cv2.drawContours(debug,[c],-1,(0,255,0),2)
        cv2.rectangle(debug,(x,y),(x+w,y+h),(255,0,0),2)

    return segmented, mask, debug

def stream(mode):

    while True:

        with frame_lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()

        segmented, mask, debug = process_frame(frame)

        if mode == "original":
            img = frame
        elif mode == "segmented":
            img = segmented
        elif mode == "mask":
            img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            img = debug

        ok, buffer = cv2.imencode(".jpg", img)
        if not ok:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buffer.tobytes() +
            b'\r\n'
        )

# =========================
# ROUTES
# =========================

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video/<mode>")
def video(mode):
    return Response(
        stream(mode),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/set_mode", methods=["POST"])
def set_mode():
    global active_mode
    active_mode = request.json["mode"]
    return jsonify({"ok": True})

@app.route("/update", methods=["POST"])
def update():
    global presets

    mode = request.json["mode"]
    values = request.json["values"]

    if mode in presets:
        presets[mode].update(values)

    return jsonify({"ok": True})

@app.route("/get_presets")
def get_presets():
    return jsonify(presets)

@app.route("/save")
def save():
    with open("vision_presets.json", "w") as f:
        json.dump(presets, f, indent=4)
    return jsonify({"saved": True})

@app.route("/load")
def load():
    global presets

    try:
        with open("vision_presets.json") as f:
            presets = json.load(f)
        return jsonify({"loaded": True})
    except:
        return jsonify({"loaded": False})

@app.route("/pixel", methods=["POST"])
def pixel():
    x = int(request.json["x"])
    y = int(request.json["y"])

    with frame_lock:
        if latest_frame is None:
            return jsonify({})

        frame = latest_frame.copy()

    if x >= frame.shape[1] or y >= frame.shape[0]:
        return jsonify({})

    bgr = frame[y, x]

    hsv = cv2.cvtColor(
        np.uint8([[bgr]]),
        cv2.COLOR_BGR2HSV
    )[0][0]

    lab = cv2.cvtColor(
        np.uint8([[bgr]]),
        cv2.COLOR_BGR2LAB
    )[0][0]

    return jsonify({
        "rgb":[int(bgr[2]),int(bgr[1]),int(bgr[0])],
        "hsv":[int(hsv[0]),int(hsv[1]),int(hsv[2])],
        "lab":[int(lab[0]),int(lab[1]),int(lab[2])]
    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)

