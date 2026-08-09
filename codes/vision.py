# vision.py

import cv2
import json
import time
import threading
import numpy as np

from picamera2 import Picamera2


class VisionSystem:

    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480

    SERVO_CENTER = 90
    SERVO_MIN = 60
    SERVO_MAX = 120

    RED_TARGET_X = int(0.80 * FRAME_WIDTH)
    GREEN_TARGET_X = int(0.20 * FRAME_WIDTH)

    def __init__(self):

        self.running = False

        self.frame_lock = threading.Lock()
        self.data_lock = threading.Lock()

        self.latest_frame = None

        self.data = {
            "primary_obstacle": None,
            "vision_angle": self.SERVO_CENTER,
            "blue_detected": False,
            "debug_frame": None
        }

        self.presets = self.load_presets()

        self.picam2 = Picamera2()

    # ==================================================
    # PRESETS
    # ==================================================

    def load_presets(self):

        try:
            with open("vision_presets.json", "r") as f:
                return json.load(f)

        except Exception:

            print("[VISION] No vision_presets.json found.")

            return {
                "red": {
                    "l_min": 0,
                    "l_max": 255,
                    "a_min": 140,
                    "a_max": 255,
                    "b_min": 100,
                    "b_max": 255
                },

                "green": {
                    "l_min": 0,
                    "l_max": 255,
                    "a_min": 0,
                    "a_max": 120,
                    "b_min": 80,
                    "b_max": 200
                },

                "blue": {
                    "l_min": 0,
                    "l_max": 255,
                    "a_min": 0,
                    "a_max": 140,
                    "b_min": 0,
                    "b_max": 120
                }
            }

    # ==================================================
    # CAMERA
    # ==================================================

    def start(self):

        config = self.picam2.create_preview_configuration(
            main={
                "size": (
                    self.FRAME_WIDTH,
                    self.FRAME_HEIGHT
                ),
                "format": "RGB888"
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

        time.sleep(1)

        self.running = True

        threading.Thread(
            target=self.camera_thread,
            daemon=True
        ).start()

        threading.Thread(
            target=self.processing_thread,
            daemon=True
        ).start()

        print("[VISION] Started")

    def stop(self):

        self.running = False

        try:
            self.picam2.stop()
        except:
            pass

    def camera_thread(self):

        while self.running:

            frame = self.picam2.capture_array()

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

            with self.frame_lock:
                self.latest_frame = frame

    # ==================================================
    # LAB MASK
    # ==================================================

    def build_lab_mask(self, lab, preset):

        lower = np.array([
            preset["l_min"],
            preset["a_min"],
            preset["b_min"]
        ])

        upper = np.array([
            preset["l_max"],
            preset["a_max"],
            preset["b_max"]
        ])

        return cv2.inRange(
            lab,
            lower,
            upper
        )

    # ==================================================
    # DETECTIONS
    # ==================================================

    def get_obstacles(
        self,
        mask,
        obstacle_class
    ):

        detections = []

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for c in contours:

            area = cv2.contourArea(c)

            if area < 500:
                continue

            x, y, w, h = cv2.boundingRect(c)

            detections.append({

                "class": obstacle_class,

                "x": x,
                "y": y,

                "width": w,
                "height": h,

                "cx": x + w // 2,
                "cy": y + h // 2,

                "area": area,

                "contour": c
            })

        return detections

    # ==================================================
    # PRIMARY OBSTACLE
    # ==================================================

def select_primary_obstacle(obstacles):

    if not obstacles:
        return None

    best = None
    best_score = -1

    for o in obstacles:

        score = (
            o["area"] * 0.7 +
            o["cy"] * 0.3
        )

        if score > best_score:
            best_score = score
            best = o

    return best

    # ==================================================
    # STEERING
    # ==================================================

    def compute_vision_angle(
        self,
        obstacle
    ):

        if obstacle is None:
            return self.SERVO_CENTER

        if obstacle["class"] == "red":
            target_x = self.RED_TARGET_X

        else:
            target_x = self.GREEN_TARGET_X

        error = target_x - obstacle["cx"]

        gain = 0.12

        angle = (
            self.SERVO_CENTER +
            error * gain
        )

        angle = int(max(
            self.SERVO_MIN,
            min(self.SERVO_MAX, angle)
        ))

        return angle

    # ==================================================
    # BLUE LINE
    # ==================================================

    def detect_blue_line(
        self,
        blue_mask
    ):

        h, w = blue_mask.shape

        roi = blue_mask[
            int(h * 0.75):h,
            :
        ]

        blue_pixels = cv2.countNonZero(roi)

        return blue_pixels > 4000

    # ==================================================
    # DEBUG
    # ==================================================

    def build_debug_frame(
        self,
        frame,
        obstacle,
        angle
    ):

        debug = frame.copy()

        cv2.line(
            debug,
            (self.RED_TARGET_X, 0),
            (self.RED_TARGET_X,
             self.FRAME_HEIGHT),
            (0, 0, 255),
            2
        )

        cv2.line(
            debug,
            (self.GREEN_TARGET_X, 0),
            (self.GREEN_TARGET_X,
             self.FRAME_HEIGHT),
            (0, 255, 0),
            2
        )

        if obstacle is not None:

            x = obstacle["x"]
            y = obstacle["y"]

            w = obstacle["width"]
            h = obstacle["height"]

            cv2.rectangle(
                debug,
                (x, y),
                (x + w, y + h),
                (255, 0, 0),
                2
            )

            cv2.circle(
                debug,
                (
                    obstacle["cx"],
                    obstacle["cy"]
                ),
                5,
                (0, 255, 255),
                -1
            )

            cv2.putText(
                debug,
                obstacle["class"].upper(),
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        cv2.putText(
            debug,
            f"ANGLE: {angle}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        return debug

    # ==================================================
    # PROCESSING THREAD
    # ==================================================

    def processing_thread(self):

        while self.running:

            with self.frame_lock:

                if self.latest_frame is None:
                    continue

                frame = self.latest_frame.copy()

            lab = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2LAB
            )

            red_mask = self.build_lab_mask(
                lab,
                self.presets["red"]
            )

            green_mask = self.build_lab_mask(
                lab,
                self.presets["green"]
            )

            blue_mask = self.build_lab_mask(
                lab,
                self.presets["blue"]
            )

            red_obs = self.get_obstacles(
                red_mask,
                "red"
            )

            green_obs = self.get_obstacles(
                green_mask,
                "green"
            )

            obstacles = (
                red_obs +
                green_obs
            )

            primary = self.select_primary_obstacle(
                obstacles
            )

            angle = self.compute_vision_angle(
                primary
            )

            blue_detected = self.detect_blue_line(
                blue_mask
            )

            debug = self.build_debug_frame(
                frame,
                primary,
                angle
            )

            with self.data_lock:

                self.data = {

                    "primary_obstacle": primary,

                    "vision_angle": angle,

                    "blue_detected": blue_detected,

                    "debug_frame": debug
                }

            time.sleep(0.01)

    # ==================================================
    # API
    # ==================================================

    def get_data(self):

        with self.data_lock:
            return self.data.copy()