# lidar.py

import threading
import time

from lidar_steering4sept import (
    LidarScanner,
    PIDController,
    calculate_steering_error
)


class LidarSystem:

    SERVO_CENTER = 90
    SERVO_MIN = 60
    SERVO_MAX = 120

    TARGET_DISTANCE_MM = 750
    SAFETY_DISTANCE_MM = 150

    def __init__(self, clockwise=True):

        self.clockwise = clockwise

        self.running = False

        self.data_lock = threading.Lock()

        self.lidar = LidarScanner()

        self.pid = PIDController(
            Kp=0.05,
            Ki=0.0,
            Kd=0.01
        )

        self.data = {
            "angle": self.SERVO_CENTER,
            "estop": False,
            "scan": None
        }

    # ==========================================
    # START / STOP
    # ==========================================

    def start(self):

        self.lidar.connect()

        self.running = True

        threading.Thread(
            target=self.lidar_thread,
            daemon=True
        ).start()

        print("[LIDAR] Started")

    def stop(self):

        self.running = False

        try:
            self.lidar.disconnect()
        except:
            pass

    # ==========================================
    # THREAD
    # ==========================================

    def lidar_thread(self):

        while self.running:

            scan = self.lidar.get_scan_data()

            if scan is None:

                time.sleep(0.01)
                continue

            error = calculate_steering_error(
                scan,
                target_distance_mm=self.TARGET_DISTANCE_MM,
                safety_distance_mm=self.SAFETY_DISTANCE_MM,
                clockwise=self.clockwise
            )

            if error == 9999.0:

                with self.data_lock:

                    self.data = {
                        "angle": self.SERVO_CENTER,
                        "estop": True,
                        "scan": scan
                    }

                continue

            correction = self.pid.update(error)

            angle = int(
                self.SERVO_CENTER +
                correction
            )

            angle = max(
                self.SERVO_MIN,
                min(self.SERVO_MAX, angle)
            )

            with self.data_lock:

                self.data = {
                    "angle": angle,
                    "estop": False,
                    "scan": scan
                }

            time.sleep(0.02)

    # ==========================================
    # API
    # ==========================================

    def get_data(self):

        with self.data_lock:
            return self.data.copy()

    def get_angle(self):

        with self.data_lock:
            return self.data["angle"]

    def emergency_stop(self):

        with self.data_lock:
            return self.data["estop"]

    def set_clockwise(self, value):

        self.clockwise = value