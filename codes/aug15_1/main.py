"""
Entry point: vision process + LiDAR process + nav process.

Run: python3 main.py

Startup order is vision -> lidar -> nav. Camera init (Picamera2 configure
+ start + libcamera pipeline warmup) is the slowest of the three, so
vision_proc is kicked off first to overlap its startup cost with LiDAR's
own connect time rather than stacking both delays in front of nav.

Uses the 'spawn' start method rather than the Linux default 'fork'.
This is required for Picamera2, which genuinely misbehaves after fork
(libcamera's camera manager doesn't survive it) — 'spawn' avoids that.
"""

import multiprocessing as mp
import signal
import sys
import time

if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)

    from shared_state import SharedRobotState

    shared = SharedRobotState()

    from lidar_process import run_lidar_process
    from vision_process import run_vision_process
    from nav_process import run_nav_process
    # ^ nav_process.py imports corner_routine_execution_v2, which registers
    #   its OWN SIGINT handler at import time. The signal.signal() call
    #   below MUST come after these imports so it's the one left in effect
    #   in *this* process. Each child process re-imports independently when
    #   it starts, so this ordering only matters here in main.py.

    def handle_shutdown(signum, frame):
        print("\n[MAIN] Shutdown signal received, stopping all processes...")
        shared.shutdown_event.set()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    vision_proc = mp.Process(target=run_vision_process, args=(shared,), name="vision_proc")
    lidar_proc = mp.Process(target=run_lidar_process, args=(shared,), name="lidar_proc")
    nav_proc = mp.Process(target=run_nav_process, args=(shared,), name="nav_proc")

    print("[MAIN] Starting vision process...")
    vision_proc.start()

    print("[MAIN] Starting LiDAR process...")
    lidar_proc.start()
    time.sleep(1.0)  # let vision (camera warmup) and LiDAR (serial connect) finish before nav starts issuing commands

    print("[MAIN] Starting nav process...")
    nav_proc.start()

    try:
        while not shared.shutdown_event.is_set():
            if not vision_proc.is_alive():
                print("[MAIN][FATAL] Vision process died unexpectedly. Shutting down.")
                shared.shutdown_event.set()
                break
            if not lidar_proc.is_alive():
                print("[MAIN][FATAL] LiDAR process died unexpectedly. Shutting down.")
                shared.shutdown_event.set()
                break
            if not nav_proc.is_alive():
                print("[MAIN][FATAL] Nav process died unexpectedly. Shutting down.")
                shared.shutdown_event.set()
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        shared.shutdown_event.set()

    print("[MAIN] Waiting for child processes to exit...")
    nav_proc.join(timeout=5.0)
    lidar_proc.join(timeout=5.0)
    vision_proc.join(timeout=5.0)

    for p in (nav_proc, lidar_proc, vision_proc):
        if p.is_alive():
            print(f"[MAIN] {p.name} didn't exit cleanly, terminating.")
            p.terminate()
            p.join(timeout=2.0)

    print("[MAIN] Shutdown complete.")
    sys.exit(0)
