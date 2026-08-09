"""
Standalone LiDAR diagnostic: prints average left/right distances over
configurable angle ranges, live, at a fixed rate.

Reuses the existing LidarScanner class so behavior matches what your
control loop actually sees.

Usage:
    python3 lidar_range_monitor.py
    python3 lidar_range_monitor.py --right-min 30 --right-max 90 --left-min -90 --left-max -30
    python3 lidar_range_monitor.py --num-values 0   # average ALL points in range, not just the top-N farthest
"""

import argparse
import signal
import sys
import time

import numpy as np

from lidar_steering4sept import LidarScanner

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 230400

scanner = None


def compute_avg_distance(scan_data, angle_min, angle_max, num_values):
    """Average distance over [angle_min, angle_max] inclusive.

    If num_values > 0, mirrors calculate_steering_error()'s behavior of only
    averaging the N farthest points in range (to reject close-range noise).
    If num_values == 0, averages every point in range with no filtering.
    """
    distances = [
        d for a, d in scan_data.items()
        if angle_min <= a <= angle_max and d is not None and 0 < d < 3000
    ]
    if not distances:
        return None, 0

    if num_values > 0:
        distances.sort(reverse=True)
        distances = distances[:num_values]

    return float(np.mean(distances)), len(distances)


def emergency_stop(signum=None, frame=None):
    print("\n[STOP] Disconnecting LiDAR...")
    if scanner:
        try:
            scanner.disconnect()
        except Exception:
            pass
    sys.exit(0)


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGQUIT, emergency_stop)


def main():
    global scanner

    parser = argparse.ArgumentParser(description="Live LiDAR left/right range monitor")
    parser.add_argument("--right-min", type=int, default=30, help="Right sensing window start angle (default: %(default)s)")
    parser.add_argument("--right-max", type=int, default=90, help="Right sensing window end angle, inclusive (default: %(default)s)")
    parser.add_argument("--left-min", type=int, default=-90, help="Left sensing window start angle (default: %(default)s)")
    parser.add_argument("--left-max", type=int, default=-30, help="Left sensing window end angle, inclusive (default: %(default)s)")
    parser.add_argument("--num-values", type=int, default=20,
                         help="Average only the N farthest points per side (matches calculate_steering_error). "
                              "Use 0 to average every point in range instead. (default: %(default)s)")
    parser.add_argument("--rate", type=float, default=2.0, help="Print rate in Hz (default: %(default)s)")
    parser.add_argument("--port", type=str, default=LIDAR_PORT, help="LiDAR serial port (default: %(default)s)")
    parser.add_argument("--baudrate", type=int, default=LIDAR_BAUDRATE, help="LiDAR baudrate (default: %(default)s)")
    args = parser.parse_args()

    print(f"[INFO] Right window: [{args.right_min}, {args.right_max}]  "
          f"Left window: [{args.left_min}, {args.left_max}]  "
          f"num_values={'all' if args.num_values == 0 else args.num_values}")

    scanner = LidarScanner(port=args.port, baudrate=args.baudrate)
    try:
        scanner.connect()
    except Exception as e:
        print(f"[FATAL] Could not connect to LiDAR: {e}")
        sys.exit(1)

    print_interval = 1.0 / args.rate

    try:
        while True:
            loop_start = time.monotonic()

            scan_data = scanner.get_scan_data()
            if not scan_data:
                print("[WARN] No scan data this cycle.")
            else:
                avg_right, n_right = compute_avg_distance(scan_data, args.right_min, args.right_max, args.num_values)
                avg_left, n_left = compute_avg_distance(scan_data, args.left_min, args.left_max, args.num_values)

                right_str = f"{avg_right:7.1f}mm (n={n_right})" if avg_right is not None else "   N/A       "
                left_str = f"{avg_left:7.1f}mm (n={n_left})" if avg_left is not None else "   N/A       "
                print(f"[LIDAR] Right [{args.right_min:>4},{args.right_max:>4}]: {right_str}  |  "
                      f"Left [{args.left_min:>4},{args.left_max:>4}]: {left_str}")

            elapsed = time.monotonic() - loop_start
            remaining = print_interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass
    finally:
        emergency_stop()


if __name__ == "__main__":
    main()