#!/usr/bin/env python3
"""
WRO Future Engineers - Obstacle Challenge Navigation Stack
Replaces: earlier left-wall-following / vision-gain obstacle scripts.

Architecture (each layer independently testable):

    Sensor Drivers  -> State Estimator -> Perception Fusion
                     -> Behavior Arbitrator -> Local Planner
                     -> Controller -> ESP32 (STR:/SPD: protocol)

Design decisions (see prior discussion for rationale):
  - No SLAM / occupancy grid: track topology is known and re-randomized
    every round, so a persistent map buys nothing. Local geometry + a
    strong structural prior (known corridor width, known turn direction)
    is what actually helps.
  - Dead-reckoning state estimate (encoder distance + trusted gyro
    heading), not a full EKF. Simpler, and sufficient to gate the corner
    turn on measured heading instead of a blind timer.
  - LiDAR finds *where* an obstacle is (range + bearing discontinuity
    vs. expected wall profile). Camera only answers *what color*. They
    are fused by bearing agreement, not run as two competing systems.
  - Turn direction is a single pre-race constant (known in advance per
    the rules), not re-decided at every corner from a noisy reading.
  - Every shared sensor value is timestamped; stale data degrades to a
    safe fallback instead of being silently trusted forever.

Works against your EXISTING, UNCHANGED ESP32 firmware - the one that streams
only "YAW:<deg>\n" (no encoder telemetry) and accepts "STR:<deg>,SPD:<speed>\n".
Two consequences of not touching the firmware, both handled here rather than
on the ESP32:

  1. No encoder ticks are available, so distance-traveled is estimated from
     commanded-speed * dt instead of true wheel odometry. This is coarser
     (battery sag / wheel slip aren't visible to it) but is only used for the
     "don't re-trigger a corner too soon" travel guard, not for the actual
     turn-exit decision - that's still gated on measured yaw, which stays
     accurate regardless.
  2. Your firmware's RESET_YAW handling is unreliable (dead code path /
     string mismatch in the version currently on the board), so instead of
     depending on it, this script captures a yaw BASELINE at start-of-run
     and treats (raw_yaw - baseline) as the effective heading. This works
     correctly even if RESET_YAW never fires on the ESP32 side at all.
"""

import time
import math
import signal
import sys
import threading
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

import numpy as np
import cv2
import serial

try:
    import ydlidar
except ImportError:
    ydlidar = None

try:
    from picamera2 import Picamera2
    import libcamera
except ImportError:
    Picamera2 = None

try:
    from gpiozero import Button
except ImportError:
    Button = None


# =====================================================================
# CONFIG
# =====================================================================

class Cfg:
    # --- Serial / hardware ---
    ESP_PORT = "/dev/ttyAMA0"
    ESP_BAUD = 115200
    LIDAR_PORT = "/dev/ttyUSB0"
    LIDAR_BAUD = 230400
    START_BUTTON_PIN = 17  # gpiozero BCM pin; None to auto-start after a delay

    # --- Vehicle geometry (measure these on your actual chassis) ---
    WHEELBASE_MM = 160.0
    WHEEL_CIRCUMFERENCE_MM = 65.0
    ENCODER_TICKS_PER_REV = 700

    # --- Track priors (rules: Obstacle Challenge corridor is fixed) ---
    CORRIDOR_WIDTH_MM = 1000.0
    CORRIDOR_TOLERANCE_MM = 50.0

    # --- Round-fixed knowledge (SET THESE BEFORE EACH ROUND, per the rules
    #     the direction is announced in advance, not something to detect) ---
    TURN_DIRECTION = "LEFT"     # "LEFT" or "RIGHT" - fixed for the whole round
    TOTAL_CORNERS = 12          # 3 laps x 4 corners

    # --- Steering / speed ---
    STEER_CENTER = 95
    STEER_MAX_DELTA = 20        # max deviation from center the planner may request
    SPEED_CRUISE = 160
    SPEED_CORNER = 160
    SPEED_AVOID = 150
    LOOKAHEAD_MM = 450.0

    # If your bench test shows the wheels spin backward for a positive SPD
    # value (known ambiguity in the current firmware's polarity), flip this
    # to -1. Do NOT touch the .ino - fix it here instead.
    SPEED_POLARITY = 1

    # --- Corner detection (debounced) ---
    CORNER_FRONT_TRIGGER_MM = 1000.0
    CORNER_SIDE_OPEN_MM = 1300.0
    CORNER_SIDE_CLOSE_MM = 900.0
    CORNER_CONFIRM_FRAMES = 4
    CORNER_MIN_TRAVEL_MM = 1200.0   # refuse to re-trigger sooner than this
    CORNER_TARGET_DEG = 88.0
    CORNER_TIMEOUT_S = 2.5          # safety ceiling, not the primary trigger

    # --- Pillar fusion ---
    PILLAR_TRIGGER_RANGE_MM = 900.0
    PILLAR_CLEARANCE_MM = 160.0
    PILLAR_BEARING_MATCH_DEG = 12.0
    CAMERA_HFOV_DEG = 102           # adjust to your actual lens

    # --- Watchdogs ---
    MAX_LIDAR_AGE_S = 0.30
    MAX_CAMERA_AGE_S = 0.40
    MAX_YAW_AGE_S = 0.25

    # --- Loop ---
    LOOP_PERIOD_S = 0.02


# =====================================================================
# TIMESTAMPED SAMPLE
# =====================================================================

@dataclass
class Sample:
    value: object = None
    ts: float = 0.0

    def is_stale(self, max_age_s: float) -> bool:
        return (self.value is None) or (time.monotonic() - self.ts > max_age_s)


# =====================================================================
# SERIAL BRIDGE (commands out, YAW/ENC telemetry in)
# =====================================================================

class SerialBridge:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.yaw = Sample(0.0, time.monotonic())
        self.enc = Sample(0, time.monotonic())
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("YAW:"):
                    continue
                now = time.monotonic()
                parts = line.split(",")
                yaw_val = float(parts[0].split(":")[1])
                self.yaw = Sample(yaw_val, now)
                if len(parts) > 1 and parts[1].startswith("ENC:"):
                    self.enc = Sample(int(parts[1].split(":")[1]), now)
            except Exception:
                continue

    def send(self, steer_deg: int, speed: int):
        steer_deg = int(np.clip(steer_deg, 0, 180))
        speed = int(np.clip(speed * Cfg.SPEED_POLARITY, -255, 255))
        try:
            self.ser.write(f"STR:{steer_deg},SPD:{speed}\n".encode("utf-8"))
        except Exception:
            pass

    def reset_yaw(self):
        # Best-effort only: the currently-flashed firmware may not act on
        # this (dead code / string mismatch). Do not rely on it for
        # correctness - see StateEstimator's software yaw baseline instead.
        try:
            self.ser.write(b"RESET_YAW\n")
        except Exception:
            pass

    def stop_hardware(self):
        for _ in range(3):
            self.send(Cfg.STEER_CENTER, 0)
            time.sleep(0.03)

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


# =====================================================================
# STATE ESTIMATOR (dead reckoning: encoder distance + trusted gyro heading)
# =====================================================================

class StateEstimator:
    """
    theta comes from the ESP32's already-filtered gyro integration (trusted).
    x, y, and distance-traveled come from encoder ticks (or a speed*dt
    fallback if encoder telemetry isn't flowing yet).
    """

    def __init__(self, bridge: SerialBridge):
        self.bridge = bridge
        self.x = 0.0
        self.y = 0.0
        self.theta_deg = 0.0
        self.yaw_offset_deg = 0.0
        self.total_dist_mm = 0.0
        self.dist_since_last_corner_mm = 0.0
        self._last_enc_ticks: Optional[int] = None
        self._last_speed_cmd = 0
        self._last_update_t = time.monotonic()

    def capture_yaw_baseline(self, samples: int = 10, delay_s: float = 0.05):
        """Call once at start-of-run. Works around the firmware's unreliable
        RESET_YAW by zeroing heading in software instead."""
        readings = []
        for _ in range(samples):
            if not self.bridge.yaw.is_stale(1.0):
                readings.append(self.bridge.yaw.value)
            time.sleep(delay_s)
        self.yaw_offset_deg = (sum(readings) / len(readings)) if readings else 0.0
        print(f"[ESTIMATOR] Yaw baseline captured: {self.yaw_offset_deg:.2f} deg")

    def note_speed_command(self, speed_cmd: int):
        self._last_speed_cmd = speed_cmd

    def update(self):
        now = time.monotonic()
        dt = now - self._last_update_t
        self._last_update_t = now

        if not self.bridge.yaw.is_stale(Cfg.MAX_YAW_AGE_S):
            self.theta_deg = self.bridge.yaw.value - self.yaw_offset_deg

        ds_mm = 0.0
        if not self.bridge.enc.is_stale(Cfg.MAX_YAW_AGE_S) and self._last_enc_ticks is not None:
            # Only used if a future firmware revision happens to add this;
            # current unmodified firmware never sends ENC, so this path is
            # inert and the speed*dt fallback below is what actually runs.
            delta_ticks = self.bridge.enc.value - self._last_enc_ticks
            ds_mm = (delta_ticks / Cfg.ENCODER_TICKS_PER_REV) * Cfg.WHEEL_CIRCUMFERENCE_MM
        else:
            # Primary distance source given the unmodified firmware: coarse,
            # but sufficient for the corner re-trigger travel guard, since
            # actual turn-exit timing is gated on measured yaw, not this.
            ds_mm = (self._last_speed_cmd / 255.0) * 900.0 * dt

        if self.bridge.enc.value is not None:
            self._last_enc_ticks = self.bridge.enc.value

        theta_rad = math.radians(self.theta_deg)
        self.x += ds_mm * math.cos(theta_rad)
        self.y += ds_mm * math.sin(theta_rad)
        self.total_dist_mm += abs(ds_mm)
        self.dist_since_last_corner_mm += abs(ds_mm)

    def mark_corner_completed(self):
        self.dist_since_last_corner_mm = 0.0


# =====================================================================
# LIDAR PERCEPTION
# =====================================================================

class LidarPerception:
    def __init__(self, port: str, baud: int):
        self.scan = Sample({}, time.monotonic())
        self._stop = threading.Event()
        self._laser = None
        self._ok = False
        if ydlidar is None:
            print("[LIDAR] ydlidar module not available - running without LiDAR.")
            return
        try:
            ydlidar.os_init()
            self._laser = ydlidar.CYdLidar()
            self._laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
            self._laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baud)
            self._laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self._laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
            self._laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
            self._laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self._laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
            self._laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
            self._laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
            self._laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
            self._laser.initialize()
            self._laser.turnOn()
            self._ok = True
            threading.Thread(target=self._loop, daemon=True).start()
            print(f"[LIDAR] Online on {port}")
        except Exception as e:
            print(f"[LIDAR] Init failed: {e}")

    def _loop(self):
        scan_obj = ydlidar.LaserScan()
        while not self._stop.is_set():
            try:
                if self._laser.doProcessSimple(scan_obj):
                    data = {}
                    for p in scan_obj.points:
                        if 0.02 <= p.range <= 16.0:
                            deg = round(math.degrees(p.angle))
                            data[deg] = p.range * 1000.0
                    self.scan = Sample(data, time.monotonic())
            except Exception:
                pass
            time.sleep(0.01)

    def sector_avg(self, deg_lo: int, deg_hi: int, min_mm: float = 40.0) -> float:
        data = self.scan.value or {}
        pts = [data[a] for a in range(deg_lo, deg_hi) if a in data and data[a] > min_mm]
        return (sum(pts) / len(pts)) if pts else 2000.0

    def find_pillar_candidates(self, expected_wall_mm: float) -> List[Tuple[float, float]]:
        """Return (bearing_deg, range_mm) for returns notably closer than the
        expected corridor wall profile - i.e. something sitting inside the lane."""
        data = self.scan.value or {}
        candidates = []
        for deg in range(-70, 71):
            r = data.get(deg)
            if r is None:
                continue
            if r < min(expected_wall_mm - 150.0, Cfg.PILLAR_TRIGGER_RANGE_MM):
                candidates.append((float(deg), r))
        return candidates

    def is_stale(self) -> bool:
        return self.scan.is_stale(Cfg.MAX_LIDAR_AGE_S)


# =====================================================================
# CAMERA PERCEPTION (color + bearing only - no distance/steering-gain here)
# =====================================================================

class CameraPerception:
    RED_LO1, RED_HI1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    RED_LO2, RED_HI2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    GREEN_LO, GREEN_HI = np.array([40, 80, 60]), np.array([85, 255, 255])
    MIN_AREA = 250

    def __init__(self):
        self.frame = Sample(None, time.monotonic())
        self._stop = threading.Event()
        self._cam = None
        if Picamera2 is None:
            print("[CAM] picamera2 not available - running without camera.")
            return
        try:
            self._cam = Picamera2()
            config = self._cam.create_preview_configuration(
                main={"size": (1152, 648), "format": "RGB888"},
                transform=libcamera.Transform(vflip=False, hflip=False),
            )
            self._cam.configure(config)
            self._cam.start()
            time.sleep(1.0)
            threading.Thread(target=self._loop, daemon=True).start()
            print("[CAM] Online.")
        except Exception as e:
            print(f"[CAM] Init failed: {e}")

    def _loop(self):
        while not self._stop.is_set():
            try:
                rgb = self._cam.capture_array()
                self.frame = Sample(rgb, time.monotonic())
            except Exception:
                pass

    def detect_color_blobs(self) -> List[Tuple[str, float]]:
        """Returns list of (color, bearing_deg) using actual camera HFOV,
        not an empirically-fit pixel->steering gain."""
        if self.frame.value is None:
            return []
        rgb = self.frame.value
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        width = rgb.shape[1]
        half_fov = Cfg.CAMERA_HFOV_DEG / 2.0

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.RED_LO1, self.RED_HI1),
            cv2.inRange(hsv, self.RED_LO2, self.RED_HI2),
        )
        green_mask = cv2.inRange(hsv, self.GREEN_LO, self.GREEN_HI)

        results = []
        for color, mask in (("RED", red_mask), ("GREEN", green_mask)):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < self.MIN_AREA:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                if h < w:  # pillars are taller than wide
                    continue
                cx = x + w / 2.0
                ratio = (cx - width / 2.0) / (width / 2.0)  # -1..+1
                bearing_deg = ratio * half_fov
                results.append((color, bearing_deg))
        return results

    def is_stale(self) -> bool:
        return self.frame.is_stale(Cfg.MAX_CAMERA_AGE_S)


# =====================================================================
# PILLAR FUSION (LiDAR gives WHERE, camera gives WHAT COLOR)
# =====================================================================

@dataclass
class ConfirmedPillar:
    color: str
    bearing_deg: float
    range_mm: float
    ts: float


class PillarFusion:
    def __init__(self, lidar: LidarPerception, cam: CameraPerception):
        self.lidar = lidar
        self.cam = cam

    def get_confirmed_pillars(self, expected_wall_mm: float) -> List[ConfirmedPillar]:
        if self.lidar.is_stale() or self.cam.is_stale():
            return []
        candidates = self.lidar.find_pillar_candidates(expected_wall_mm)
        color_bearings = self.cam.detect_color_blobs()
        confirmed = []
        now = time.monotonic()
        for cbearing, crange in candidates:
            best = None
            best_diff = Cfg.PILLAR_BEARING_MATCH_DEG
            for color, vbearing in color_bearings:
                diff = abs(vbearing - cbearing)
                if diff < best_diff:
                    best_diff = diff
                    best = color
            if best is not None:
                confirmed.append(ConfirmedPillar(best, cbearing, crange, now))
        return confirmed


# =====================================================================
# CORNER DETECTOR (debounced, direction fixed by config not re-detected)
# =====================================================================

class CornerDetector:
    def __init__(self):
        self._confirm_count = 0

    def check(self, lidar: LidarPerception, estimator: StateEstimator) -> bool:
        if estimator.dist_since_last_corner_mm < Cfg.CORNER_MIN_TRAVEL_MM:
            self._confirm_count = 0
            return False

        front = lidar.sector_avg(-15, 16)
        left = lidar.sector_avg(-90, -39)
        right = lidar.sector_avg(40, 91)

        signature = (front <= Cfg.CORNER_FRONT_TRIGGER_MM) and (
            (left < Cfg.CORNER_SIDE_CLOSE_MM and right > Cfg.CORNER_SIDE_OPEN_MM) or
            (right < Cfg.CORNER_SIDE_CLOSE_MM and left > Cfg.CORNER_SIDE_OPEN_MM)
        )

        if signature:
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        return self._confirm_count >= Cfg.CORNER_CONFIRM_FRAMES


# =====================================================================
# LOCAL PLANNER (centerline pure pursuit + metric pillar-offset injection)
# =====================================================================

class PurePursuitPlanner:
    def compute_steering(self, lidar: LidarPerception, pillars: List[ConfirmedPillar],
                          fallback_heading_pid_output: float) -> Tuple[float, str]:
        left = lidar.sector_avg(-90, -39)
        right = lidar.sector_avg(40, 91)
        have_left = left < 1500.0
        have_right = right < 1500.0

        if have_left and have_right:
            lateral_error_mm = ((left + right) / 2.0) - (left)  # +ve => drift right of center
            lateral_error_mm = (right - left) / 2.0
            mode = "centerline(both walls)"
        elif have_left:
            lateral_error_mm = (Cfg.CORRIDOR_WIDTH_MM / 2.0) - left
            mode = "centerline(left+prior)"
        elif have_right:
            lateral_error_mm = right - (Cfg.CORRIDOR_WIDTH_MM / 2.0)
            mode = "centerline(right+prior)"
        else:
            return fallback_heading_pid_output, "gyro-straight(no walls)"

        active_pillar = min(pillars, key=lambda p: p.range_mm) if pillars else None
        if active_pillar:
            push = Cfg.PILLAR_CLEARANCE_MM
            if active_pillar.color == "RED":
                lateral_error_mm -= push   # red -> keep right -> bias target left-of-pillar error
            else:
                lateral_error_mm += push   # green -> keep left
            mode = f"avoid({active_pillar.color})"

        curvature = 2.0 * lateral_error_mm / (Cfg.LOOKAHEAD_MM ** 2)
        steer_delta_rad = math.atan(Cfg.WHEELBASE_MM * curvature)
        steer_delta_deg = math.degrees(steer_delta_rad)
        steer_delta_deg = float(np.clip(steer_delta_deg, -Cfg.STEER_MAX_DELTA, Cfg.STEER_MAX_DELTA))
        return steer_delta_deg, mode


# =====================================================================
# SIMPLE HEADING PID (fallback when no walls are visible)
# =====================================================================

class HeadingPID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral = 0.0
        self._prev_err = 0.0

    def update(self, target_deg: float, current_deg: float, dt: float) -> float:
        err = target_deg - current_deg
        self._integral += err * dt
        deriv = (err - self._prev_err) / dt if dt > 0 else 0.0
        self._prev_err = err
        out = self.kp * err + self.ki * self._integral + self.kd * deriv
        return float(np.clip(out, -Cfg.STEER_MAX_DELTA, Cfg.STEER_MAX_DELTA))


# =====================================================================
# MAIN CONTROLLER / BEHAVIOR ARBITRATOR
# =====================================================================

class RobotController:
    STATE_INIT = "INIT"
    STATE_DRIVE = "DRIVE"
    STATE_CORNER = "CORNER"
    STATE_DONE = "DONE"

    def __init__(self):
        self.bridge = SerialBridge(Cfg.ESP_PORT, Cfg.ESP_BAUD)
        self.lidar = LidarPerception(Cfg.LIDAR_PORT, Cfg.LIDAR_BAUD)
        self.cam = CameraPerception()
        self.fusion = PillarFusion(self.lidar, self.cam)
        self.corner_detector = CornerDetector()
        self.planner = PurePursuitPlanner()
        self.estimator = StateEstimator(self.bridge)
        self.heading_pid = HeadingPID()

        self.state = self.STATE_INIT
        self.corners_done = 0
        self.heading_target_deg = 0.0
        self._corner_start_time = 0.0
        self._corner_direction_sign = 1 if Cfg.TURN_DIRECTION == "LEFT" else -1

        self._shutdown = threading.Event()
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, *_):
        self._shutdown.set()

    # --- start gating per rules 9.10/9.11: wait for a single start button ---
    def _wait_for_start(self):
        if Button is not None and Cfg.START_BUTTON_PIN is not None:
            print("[INIT] Waiting for start button...")
            btn = Button(Cfg.START_BUTTON_PIN)
            btn.wait_for_press()
        else:
            print("[INIT] No start button wired - auto-starting in 3s.")
            time.sleep(3.0)

        # Keep the vehicle still through this - it sets the software yaw
        # zero point. Firmware RESET_YAW is sent too (harmless either way)
        # but correctness does not depend on it taking effect.
        self.bridge.reset_yaw()
        self.estimator.capture_yaw_baseline()

    def run(self):
        self._wait_for_start()
        self.state = self.STATE_DRIVE
        print("[RUN] Starting main loop.")

        while not self._shutdown.is_set():
            loop_start = time.monotonic()
            self.estimator.update()

            if self.state == self.STATE_DONE:
                break

            if self.state == self.STATE_CORNER:
                self._run_corner_state()
            else:
                self._run_drive_state()

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, Cfg.LOOP_PERIOD_S - elapsed))

        self._shutdown_hardware()

    def _run_drive_state(self):
        # Watchdog: if core sensors are stale, fail safe rather than trust ghosts
        if self.lidar.is_stale():
            self.bridge.send(Cfg.STEER_CENTER, 0)
            self.estimator.note_speed_command(0)
            print("[WATCHDOG] LiDAR stale - holding.")
            return

        if self.corners_done >= Cfg.TOTAL_CORNERS:
            self._finish_run()
            return

        if self.corner_detector.check(self.lidar, self.estimator):
            print(f"[CORNER] Confirmed after {self.estimator.dist_since_last_corner_mm:.0f}mm travel.")
            self.state = self.STATE_CORNER
            self._corner_start_time = time.monotonic()
            self.heading_target_deg = self.estimator.theta_deg + (
                self._corner_direction_sign * Cfg.CORNER_TARGET_DEG
            )
            return

        pillars = self.fusion.get_confirmed_pillars(Cfg.CORRIDOR_WIDTH_MM)
        fallback = self.heading_pid.update(0.0, self.estimator.theta_deg, Cfg.LOOP_PERIOD_S)
        steer_delta, mode = self.planner.compute_steering(self.lidar, pillars, fallback)

        speed = Cfg.SPEED_AVOID if pillars else Cfg.SPEED_CRUISE
        steer_cmd = int(round(Cfg.STEER_CENTER + steer_delta))
        self.bridge.send(steer_cmd, speed)
        self.estimator.note_speed_command(speed)
        print(f"[DRIVE] mode={mode} steer={steer_cmd} speed={speed} "
              f"pillars={len(pillars)} yaw={self.estimator.theta_deg:.1f}")

    def _run_corner_state(self):
        elapsed = time.monotonic() - self._corner_start_time
        heading_error = abs(self.estimator.theta_deg - self.heading_target_deg)
        turned_enough = heading_error <= 5.0
        timed_out = elapsed >= Cfg.CORNER_TIMEOUT_S

        if turned_enough or timed_out:
            reason = "yaw target reached" if turned_enough else "SAFETY TIMEOUT (check gyro/servo)"
            print(f"[CORNER] Exiting - {reason}. yaw={self.estimator.theta_deg:.1f} "
                  f"target={self.heading_target_deg:.1f}")
            self.bridge.send(Cfg.STEER_CENTER, 0)
            time.sleep(0.15)
            self.corners_done += 1
            self.estimator.mark_corner_completed()
            self.corner_detector = CornerDetector()  # reset debounce state
            self.state = self.STATE_DRIVE
            return

        lock_angle = Cfg.STEER_CENTER + (self._corner_direction_sign * Cfg.STEER_MAX_DELTA)
        self.bridge.send(int(lock_angle), Cfg.SPEED_CORNER)
        self.estimator.note_speed_command(Cfg.SPEED_CORNER)
        print(f"[CORNER] turning... yaw={self.estimator.theta_deg:.1f} "
              f"target={self.heading_target_deg:.1f} t={elapsed:.2f}s")

    def _finish_run(self):
        print("[FINISH] All corners completed - stopping.")
        self.bridge.send(Cfg.STEER_CENTER, Cfg.SPEED_CRUISE)
        time.sleep(1.0)  # brief coast; replace with a front-distance stop check
        self.bridge.send(Cfg.STEER_CENTER, 0)
        self.state = self.STATE_DONE

    def _shutdown_hardware(self):
        print("[SHUTDOWN] Stopping hardware.")
        self.bridge.stop_hardware()
        self.bridge.close()


# =====================================================================
# ENTRY POINT
# =====================================================================

if __name__ == "__main__":
    controller = RobotController()
    try:
        controller.run()
    except Exception as e:
        print(f"[FATAL] {e}")
    finally:
        controller._shutdown_hardware()
        sys.exit(0)
