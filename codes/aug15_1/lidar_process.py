"""
LiDAR process.

Owns the physical LidarScanner exclusively. Every scan:
  1. get_scan_data()                       (~89ms, blocking serial read)
  2. evaluate_corner_detection(scan_data)  (split-and-merge L-corner detect)
  3. get_wall_parallel_error(scan_data)    (wall-follow PID input)
  4. side-zone raw trigger + min-dist      (side-avoidance input)
  5. publish everything to shared_state

This is exactly the corner_evaluate_detection + wall_follow_pid input +
side_zone_math work that used to run on the nav thread in the single-
process version — moving here is what stops it from blocking/GIL-starving
the camera acquisition thread today, and what lets the nav loop run at a
much higher, jitter-free rate once the vision process is added too.

Nothing in evaluate_corner_detection or get_wall_parallel_error changes —
same functions, same signatures, just called from here instead.
"""

import time

from config import (
    LIDAR_PORT, LIDAR_BAUD,
    FRONT_SCAN_ANGLE_DEG,
    WALL_FOLLOW_SIDE, WALL_FOLLOW_TARGET_DISTANCE_MM,
    SIDE_RIGHT_ZONE_ANGLES, SIDE_LEFT_ZONE_ANGLES,
    SIDE_TRIGGER_DISTANCE_MM, MIN_TRIGGER_POINTS,
    LOG_EVERY_N_LOOPS,
)
from lidar_steering_new_parallel import LidarScanner, get_wall_parallel_error, PARALLEL_DISTANCE_WEIGHT
from corner_lidar_13aug_working_2 import evaluate_corner_detection
from profiling import profile_section, print_profile_summary
from shared_state import SharedRobotState


def _front_sector_distance(scan_data):
    pts = [scan_data[a] for a in range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
           if a in scan_data and scan_data[a] > 0]
    return sum(pts) / len(pts) if pts else 2000.0


def _side_zone_raw_triggers(scan_data):
    right_zone_points = [scan_data[a] for a in SIDE_RIGHT_ZONE_ANGLES if a in scan_data and scan_data[a] > 0]
    left_zone_points = [scan_data[a] for a in SIDE_LEFT_ZONE_ANGLES if a in scan_data and scan_data[a] > 0]

    right_under = [d for d in right_zone_points if d < SIDE_TRIGGER_DISTANCE_MM]
    left_under = [d for d in left_zone_points if d < SIDE_TRIGGER_DISTANCE_MM]

    right_raw_trigger = len(right_under) >= MIN_TRIGGER_POINTS
    left_raw_trigger = len(left_under) >= MIN_TRIGGER_POINTS

    right_min_dist = (sum(sorted(right_under)[:3]) / 3) if right_raw_trigger else -1.0
    left_min_dist = (sum(sorted(left_under)[:3]) / 3) if left_raw_trigger else -1.0

    return right_raw_trigger, left_raw_trigger, right_min_dist, left_min_dist


def run_lidar_process(shared: SharedRobotState):
    print("[LIDAR PROC] Starting up.")

    scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
    try:
        scanner.connect()
    except Exception as e:
        print(f"[LIDAR PROC][FATAL] Could not connect to LiDAR: {e}")
        return

    loop_iteration = 0

    try:
        while not shared.shutdown_event.is_set():
            with profile_section("lidar.get_scan_data"):
                scan_data = scanner.get_scan_data()

            if not scan_data:
                time.sleep(0.005)
                continue

            with profile_section("lidar.write_scan_array"):
                shared.write_scan(scan_data)

            with profile_section("lidar.corner_evaluate_detection"):
                corner_flag, metadata = evaluate_corner_detection(scan_data)

            with profile_section("lidar.wall_follow_error"):
                combined_error = get_wall_parallel_error(
                    scan_data, WALL_FOLLOW_SIDE,
                    WALL_FOLLOW_TARGET_DISTANCE_MM, PARALLEL_DISTANCE_WEIGHT,
                )

            with profile_section("lidar.side_zone_math"):
                right_trig, left_trig, right_min, left_min = _side_zone_raw_triggers(scan_data)
                avg_front = _front_sector_distance(scan_data)

            with profile_section("lidar.publish_result"):
                with shared.lidar_result.get_lock():
                    r = shared.lidar_result
                    r.timestamp = time.monotonic()
                    r.avg_front_dist = avg_front

                    r.corner_flag = corner_flag
                    r.corner_distance_mm = metadata.get("corner_distance_mm") or -1.0
                    r.corner_front_mm = metadata.get("front") or -1.0
                    r.corner_sum_avg_side_mm = metadata.get("sum_avg_side") or -1.0

                    r.wall_error_valid = combined_error is not None
                    r.wall_combined_error = combined_error if combined_error is not None else 0.0

                    r.right_min_dist = right_min
                    r.left_min_dist = left_min
                    r.right_raw_trigger = right_trig
                    r.left_raw_trigger = left_trig

            loop_iteration += 1
            if loop_iteration % LOG_EVERY_N_LOOPS == 0:
                print_profile_summary(tag="lidar")

    except Exception as e:
        print(f"[LIDAR PROC][FAILURE] {e}")
    finally:
        print("[LIDAR PROC] Shutting down, disconnecting scanner.")
        try:
            scanner.disconnect()
        except Exception:
            pass
