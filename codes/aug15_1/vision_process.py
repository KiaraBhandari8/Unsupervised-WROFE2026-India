"""
Vision process.

Owns the Picamera2 camera exclusively, exactly like lidar_process.py owns
the LidarScanner. Per the design note in HANDOFF_SUMMARY.md: camera frames
never cross a process boundary. This process does capture -> BGR convert
-> process_frame_for_steering() -> gain scaling -> publish, all locally,
and only a tiny VisionResult struct (a float and a short label) crosses
into the nav process via multiprocessing.Value -- same pattern as
LidarResult.

The Flask MJPEG debug stream (STREAM_VIDEO/DEBUG_UI_OVERLAYS, ported as-is
from the old single-file main.py) also lives entirely in this process,
since it only ever needs the locally-held processed frame.

Gain scaling (STEERING_GAIN_RED / STEERING_GAIN_GREEN / RED_CLEARANCE_OFFSET)
happens HERE, not in nav_process.py -- process_frame_for_steering() itself
returns only the raw PD steering_angle. This mirrors exactly where the old
single-file main loop applied these gains, just relocated into this process
so nav_process.py only ever has to read a ready-to-use servo_adjust float.
"""

import time

import cv2
import numpy as np

from config import (
    PI_TO_ESP_PORT,  # noqa: F401  (not used here; kept for reference/symmetry)
    CAMERA_RESOLUTION, LORES_RESOLUTION, CAMERA_FRAMERATE, CAMERA_BUFFER_COUNT,
    PROCESSING_WIDTH, PROCESSING_HEIGHT,
    STEERING_GAIN_GREEN, STEERING_GAIN_RED, RED_CLEARANCE_OFFSET,
    STREAM_VIDEO, DEBUG_UI_OVERLAYS, STREAM_FRAME_INTERVAL_S, VISION_FLASK_PORT,
    LOG_EVERY_N_LOOPS,
)
from image_frame_combine_outer_inner_depthfps_aspVision import process_frame_for_steering
from profiling import profile_section, print_profile_summary
from shared_state import SharedRobotState


def _compute_servo_adjust(vision_angle, logic_label):
    """
    Ported unchanged from the old main loop's Tier 4 block:
        vision_angle = -1 * vision_angle
        red:   servo_adjust = -vision_angle * STEERING_GAIN_RED; + RED_CLEARANCE_OFFSET
        green: servo_adjust = -vision_angle * STEERING_GAIN_GREEN
    Returns (servo_adjust, is_obstacle) where servo_adjust is what nav
    should do: target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust.
    """
    vision_angle = -1 * vision_angle
    if logic_label == "red_obstacle":
        servo_adjust = -vision_angle * STEERING_GAIN_RED + RED_CLEARANCE_OFFSET
        return servo_adjust, True
    elif logic_label == "obstacle":
        servo_adjust = -vision_angle * STEERING_GAIN_GREEN
        return servo_adjust, True
    else:
        return 0.0, False


def _make_flask_app(get_frame_fn, shared: SharedRobotState):
    from flask import Flask, Response

    app = Flask(__name__)

    def generate_frames():
        last_emit_time = 0.0
        while not shared.shutdown_event.is_set():
            if not STREAM_VIDEO:
                time.sleep(0.2)
                continue

            now = time.monotonic()
            wait = STREAM_FRAME_INTERVAL_S - (now - last_emit_time)
            if wait > 0:
                time.sleep(wait)

            local_frame = get_frame_fn()
            if local_frame is None:
                time.sleep(0.03)
                continue

            flag, encoded_image = cv2.imencode(".jpg", local_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not flag:
                continue

            last_emit_time = time.monotonic()
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

    @app.route("/")
    def index():
        return "<h3>WRO Live Camera Server Active</h3><img src='/video_feed' width='100%'/>"

    @app.route("/video_feed")
    def video_feed():
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    return app


def run_vision_process(shared: SharedRobotState):
    print("[VISION PROC] Starting up.")

    # Import Picamera2/libcamera here (not at module top) so any process
    # that merely imports this file for constants (none currently do, but
    # keeps the pattern consistent) doesn't require camera hardware/libs.
    from picamera2 import Picamera2
    import libcamera

    with profile_section("vision.init.camera_setup"):
        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration(
            main={"size": CAMERA_RESOLUTION},
            lores={"size": LORES_RESOLUTION, "format": "YUV420"},
            transform=libcamera.Transform(vflip=False, hflip=False),
            controls={"FrameRate": CAMERA_FRAMERATE},
            buffer_count=CAMERA_BUFFER_COUNT,
        )
        picam2.configure(camera_config)
        picam2.start()
        time.sleep(1)

    processing_size = (PROCESSING_WIDTH, PROCESSING_HEIGHT)

    # Local-only state -- never crosses the process boundary. The Flask
    # thread (started below) reads output_frame_holder via the closure
    # get_frame(); nav_process.py never sees this.
    output_frame_holder = {"frame": None}

    def get_frame():
        return output_frame_holder["frame"]

    flask_thread = None
    if STREAM_VIDEO:
        import threading
        app = _make_flask_app(get_frame, shared)
        flask_thread = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=VISION_FLASK_PORT,
                                    debug=False, threaded=True, use_reloader=False),
            daemon=True,
        )
        flask_thread.start()
        print(f"[VISION PROC] Debug MJPEG stream on http://0.0.0.0:{VISION_FLASK_PORT}/video_feed")

    loop_iteration = 0

    try:
        while not shared.shutdown_event.is_set():
            with profile_section("vision.camera_capture_array"):
                lores_yuv = picam2.capture_array("lores")

            with profile_section("vision.camera_colorconvert_resize"):
                frame_bgr = cv2.cvtColor(lores_yuv, cv2.COLOR_YUV2BGR_I420)
                frame_size = (frame_bgr.shape[1], frame_bgr.shape[0])
                # if frame_size != processing_size:
                #     frame_bgr = cv2.resize(frame_bgr, processing_size, interpolation=cv2.INTER_AREA)

            with profile_section("vision.process_frame_for_steering"):
                processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                    frame_bgr, draw_overlays=DEBUG_UI_OVERLAYS
                )

            with profile_section("vision.gain_scale"):
                servo_adjust, is_obstacle = _compute_servo_adjust(vision_angle, logic_label)

            with profile_section("vision.publish_result"):
                with shared.vision_result.get_lock():
                    v = shared.vision_result
                    v.timestamp = time.monotonic()
                    v.valid = True
                    v.obstacle_label = logic_label.encode('utf-8')[:15]
                    v.servo_adjust = servo_adjust
                    v.raw_steering_angle = vision_angle

            if STREAM_VIDEO:
                if DEBUG_UI_OVERLAYS:
                    cv2.putText(processed_frame, f"logic: {logic_label} | adj: {servo_adjust:.1f}",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                output_frame_holder["frame"] = processed_frame

            loop_iteration += 1
            if loop_iteration % LOG_EVERY_N_LOOPS == 0:
                print_profile_summary(tag="vision")

    except Exception as e:
        print(f"[VISION PROC][FAILURE] {e}")
    finally:
        print("[VISION PROC] Shutting down, stopping camera.")
        try:
            picam2.stop()
        except Exception:
            pass
