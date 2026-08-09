"""
Standalone LiDAR diagnostic tool.

Purpose: let you set a left/right/front angle RANGE below, then continuously
print the live top 20 points average distance (mm) inside each range, plus how 
many valid points were found. Walk/drive the robot toward a real corner while 
this runs so you can see what values actually show up at your track's corners, 
and pick correct thresholds instead of guessing.

Run directly: python3 lidar_range_check.py
Stop with Ctrl+C (safely disconnects the LiDAR).
"""

import time
import signal
import sys

from lidar_steering_new import LidarScanner

# ============================================================
# EDIT THESE RANGES TO TEST DIFFERENT ANGLE WINDOWS
# ============================================================
FRONT_ANGLES = range(-15, 16)     # straight ahead cone
LEFT_ANGLES  = range(-95, -70)   # lateral-left window (matches this file's own convention)
RIGHT_ANGLES = range(70, 95)     # lateral-right window (matches this file's own convention)

# Only count points inside this range as "valid" (filters out 0 / bad reads)
MIN_VALID_MM = 1.0
MAX_VALID_MM = 6000.0

PRINT_INTERVAL_SEC = 0.2   # how often to print a fresh line

TOP_N_FURTHEST = 20   # matches calculate_steering_error()'s noise-filtering approach

# ============================================================

shutdown_flag = False


def handle_sigint(signum, frame):
    global shutdown_flag
    shutdown_flag = True


signal.signal(signal.SIGINT, handle_sigint)


def sector_avg_top_n_furthest(scan_data, angles, top_n=TOP_N_FURTHEST):
    """Sort valid points farthest-first, keep only the top_n furthest, average those.
    Meant to reduce noise from stray close reflections (e.g. wheels, small clutter)."""
    values = [scan_data[a] for a in angles if a in scan_data and MIN_VALID_MM <= scan_data[a] <= MAX_VALID_MM]
    if not values:
        return None, 0
    values.sort(reverse=True)
    top_values = values[:top_n]
    return sum(top_values) / len(top_values), len(top_values)


def main():
    print("[INIT] Connecting to LiDAR...")
    scanner = LidarScanner(port='/dev/ttyUSB0', baudrate=230400)
    scanner.connect()
    print("[INIT] Connected. Reading live scan data. Ctrl+C to stop.\n")
    print(f"Front angles: {FRONT_ANGLES.start} to {FRONT_ANGLES.stop - 1}")
    print(f"Left angles:  {LEFT_ANGLES.start} to {LEFT_ANGLES.stop - 1}")
    print(f"Right angles: {RIGHT_ANGLES.start} to {RIGHT_ANGLES.stop - 1}\n")

    last_print_time = 0.0

    try:
        while not shutdown_flag:
            scan_data = scanner.get_scan_data()
            if not scan_data:
                time.sleep(0.02)
                continue

            now = time.monotonic()
            if now - last_print_time < PRINT_INTERVAL_SEC:
                continue
            last_print_time = now

            # Top N furthest averages for Front, Left, and Right
            front_top_avg, front_top_n = sector_avg_top_n_furthest(scan_data, FRONT_ANGLES)
            left_top_avg, left_top_n = sector_avg_top_n_furthest(scan_data, LEFT_ANGLES)
            right_top_avg, right_top_n = sector_avg_top_n_furthest(scan_data, RIGHT_ANGLES)

            # Format outputs
            front_top_str = f"{front_top_avg:.0f}mm (n={front_top_n})" if front_top_avg is not None else "N/A (n=0)"
            left_top_str = f"{left_top_avg:.0f}mm (n={left_top_n})" if left_top_avg is not None else "N/A (n=0)"
            right_top_str = f"{right_top_avg:.0f}mm (n={right_top_n})" if right_top_avg is not None else "N/A (n=0)"

            print(f"Front(top{TOP_N_FURTHEST}): {front_top_str:<18} | "
                  f"Left(top{TOP_N_FURTHEST}): {left_top_str:<18} | "
                  f"Right(top{TOP_N_FURTHEST}): {right_top_str:<18}")

    finally:
        print("\n[SHUTDOWN] Disconnecting LiDAR...")
        scanner.disconnect()
        print("[SHUTDOWN] Done.")
        sys.exit(0)


if __name__ == '__main__':
    main()