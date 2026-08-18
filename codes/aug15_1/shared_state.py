"""
Cross-process shared state for the WRO robot.

Two things live here:

1. LidarResult — a tiny ctypes struct (a handful of floats/bools) that the
   LiDAR process writes every scan and the nav process reads every loop.
   Backed by multiprocessing.Value with its own lock. This is the ONLY
   thing that crosses the process boundary on a normal loop iteration —
   microseconds of lock overhead, nothing frame-sized.

2. A 361-slot shared scan array (index = angle_deg + 180, sentinel 0.0 =
   "no reading at this angle") that mirrors the LidarScanner's own
   {angle: distance_mm} dict format. This exists for exactly one reason:
   execute_cornering() calls scanner_instance.get_scan_data() *repeatedly*
   during a blocking maneuver, not just once. Since the real LidarScanner
   object now lives exclusively in the LiDAR process, the nav process
   hands execute_cornering a SharedScanReader instead — same
   .get_scan_data() interface, backed by this shared array instead of the
   serial port. The LiDAR process keeps refreshing the array in the
   background at its normal ~10-12Hz scan rate throughout the maneuver.
"""

import ctypes
import multiprocessing as mp
import time

SCAN_ARRAY_SIZE = 361  # angles -180..180 inclusive
SCAN_ANGLE_OFFSET = 180


def angle_to_index(angle_deg):
    idx = int(angle_deg) + SCAN_ANGLE_OFFSET
    if 0 <= idx < SCAN_ARRAY_SIZE:
        return idx
    return None


class LidarResult(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_double),          # time.monotonic() at last write
        ("avg_front_dist", ctypes.c_float),

        ("corner_flag", ctypes.c_bool),
        ("corner_distance_mm", ctypes.c_float),  # -1.0 if None
        ("corner_front_mm", ctypes.c_float),
        ("corner_sum_avg_side_mm", ctypes.c_float),

        ("wall_error_valid", ctypes.c_bool),
        ("wall_combined_error", ctypes.c_float),

        ("right_min_dist", ctypes.c_float),       # -1.0 if no points in zone
        ("left_min_dist", ctypes.c_float),        # -1.0 if no points in zone
        ("right_raw_trigger", ctypes.c_bool),
        ("left_raw_trigger", ctypes.c_bool),
    ]


class VisionResult(ctypes.Structure):
    """
    Published by vision_process.py after every processed frame, read by
    nav_process.py once per loop. Same "tiny struct over mp.Value" pattern
    as LidarResult — camera frames themselves never cross the process
    boundary, only this handful of floats/bools/short string.

    obstacle_label is fixed-size bytes rather than a Python str because
    ctypes.Structure fields backing an mp.Value must be plain C types.
    Values used: b"none", b"red_obstacle", b"obstacle" (green) — same
    logic_label strings process_frame_for_steering() already returns.
    """
    _fields_ = [
        ("timestamp", ctypes.c_double),         # time.monotonic() at last publish
        ("valid", ctypes.c_bool),               # False until first frame processed
        ("obstacle_label", ctypes.c_char * 16),
        ("servo_adjust", ctypes.c_float),        # final gain-scaled adjust, ready to subtract from SERVO_CENTER_ANGLE
        ("raw_steering_angle", ctypes.c_float),  # pre-gain angle from process_frame_for_steering, for logging only
    ]


class SharedRobotState:
    """Bundles everything that crosses the nav <-> lidar <-> vision process boundaries."""

    def __init__(self):
        self.lidar_result = mp.Value(LidarResult, lock=True)
        self.scan_array = mp.Array(ctypes.c_float, SCAN_ARRAY_SIZE, lock=True)
        self.vision_result = mp.Value(VisionResult, lock=True)
        self.shutdown_event = mp.Event()

    def write_scan(self, scan_data):
        """Called by the LiDAR process after every successful get_scan_data()."""
        with self.scan_array.get_lock():
            arr = self.scan_array
            # Clear previous scan first so stale angles from a prior frame
            # (e.g. the LiDAR briefly missed a return there) don't linger.
            for i in range(SCAN_ARRAY_SIZE):
                arr[i] = 0.0
            for angle, dist in scan_data.items():
                idx = angle_to_index(angle)
                if idx is not None and dist and dist > 0:
                    arr[idx] = float(dist)

    def read_scan(self):
        """Reconstructs the {angle_deg: distance_mm} dict from the shared array."""
        with self.scan_array.get_lock():
            snapshot = list(self.scan_array)
        return {
            i - SCAN_ANGLE_OFFSET: snapshot[i]
            for i in range(SCAN_ARRAY_SIZE)
            if snapshot[i] > 0.0
        }


class SharedScanReader:
    """
    Drop-in replacement for a live LidarScanner, for code paths (namely
    execute_cornering and everything it calls) that only ever call
    .get_scan_data() on the scanner instance they're handed. Backed by the
    shared scan array instead of the serial port — the LiDAR process is
    the only thing that actually talks to the hardware.
    """

    def __init__(self, shared_state: SharedRobotState):
        self._shared = shared_state

    def get_scan_data(self):
        data = self._shared.read_scan()
        return data if data else None
