#!/usr/bin/env python3

import signal
import sys
import time
from collections import deque

from lidar_steering_new import (
	LidarScanner,
	PARALLEL_CHECK_HALF_ANGLE_DEG,
	PARALLEL_MAX_VALID_RANGE_MM,
	PARALLEL_SECTOR_WIDTH_DEG,
	PARALLEL_TOLERANCE_MM,
	get_wall_parallel_error,
	get_wall_parallel_sector_stats,
)


LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 230400


SAMPLE_PERIOD_SEC = 0.15
ROLLING_WINDOW = 20


shutdown_requested = False


def _measure_parallel_side(scan_data, side):
	front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, side)
	error = get_wall_parallel_error(scan_data, side)
	return {
		"front_avg": front_avg,
		"rear_avg": rear_avg,
		"front_count": front_count,
		"rear_count": rear_count,
		"error": error,
	}


def _format_mm(value):
	return "N/A" if value is None else f"{value:.1f}"


def _format_min_max(samples):
	if not samples:
		return "N/A"
	return f"{min(samples):.1f}/{max(samples):.1f}"


def _handle_signal(signum, frame):
	global shutdown_requested
	shutdown_requested = True


def main():
	global shutdown_requested

	signal.signal(signal.SIGINT, _handle_signal)
	if hasattr(signal, "SIGTERM"):
		signal.signal(signal.SIGTERM, _handle_signal)

	lidar = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
	front_history = {
		"left": deque(maxlen=ROLLING_WINDOW),
		"right": deque(maxlen=ROLLING_WINDOW),
	}
	rear_history = {
		"left": deque(maxlen=ROLLING_WINDOW),
		"right": deque(maxlen=ROLLING_WINDOW),
	}

	try:
		print(f"[INIT] Connecting LiDAR on {LIDAR_PORT} @ {LIDAR_BAUDRATE} baud...")
		lidar.connect()
		print("[INIT] Stationary diagnostic active. Press Ctrl+C to stop.")

		while not shutdown_requested:
			loop_start = time.monotonic()
			scan_data = lidar.get_scan_data()

			if not scan_data:
				print("[SCAN] No fresh scan data returned.")
				elapsed = time.monotonic() - loop_start
				time.sleep(max(0.0, SAMPLE_PERIOD_SEC - elapsed))
				continue

			left_stats = _measure_parallel_side(scan_data, "left")
			right_stats = _measure_parallel_side(scan_data, "right")

			for side_name, stats in (("left", left_stats), ("right", right_stats)):
				if stats["front_avg"] is not None:
					front_history[side_name].append(stats["front_avg"])
				if stats["rear_avg"] is not None:
					rear_history[side_name].append(stats["rear_avg"])

			timestamp = time.strftime("%H:%M:%S")
			print(f"[{timestamp}] Wall Parallel Diagnostic")

			for side_name, stats in (("LEFT", left_stats), ("RIGHT", right_stats)):
				error = stats["error"]
				status = "FAIL"
				if error is not None and abs(error) <= PARALLEL_TOLERANCE_MM:
					status = "PASS"

				print(
					f"{side_name:>5} | "
					f"front_avg={_format_mm(stats['front_avg'])}mm "
					f"rear_avg={_format_mm(stats['rear_avg'])}mm "
					f"error={_format_mm(error)}mm "
					f"front_pts={stats['front_count']} rear_pts={stats['rear_count']} "
					f"front_hist[min/max]={_format_min_max(front_history[side_name.lower()])} "
					f"rear_hist[min/max]={_format_min_max(rear_history[side_name.lower()])} "
					f"tolerance={PARALLEL_TOLERANCE_MM:.1f}mm status={status}"
				)

			elapsed = time.monotonic() - loop_start
			time.sleep(max(0.0, SAMPLE_PERIOD_SEC - elapsed))

	except KeyboardInterrupt:
		shutdown_requested = True
	finally:
		try:
			lidar.disconnect()
		except Exception as exc:
			print(f"[CLEANUP] LiDAR disconnect raised an error: {exc}")


if __name__ == "__main__":
	sys.exit(main())
