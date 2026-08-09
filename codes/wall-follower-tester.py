"""
wall_follow_debug.py
=====================
Standalone debug harness for the wall-following logic ONLY.

This strips out everything else from the main robot script (camera,
vision-based obstacle avoidance, corner-turn state machine, Flask
video server) so you can isolate and tune just:

  1. Parallel-wall alignment (the PID used for normal cruising in the
     main loop's WALL_ALIGN_CORRECTION state) — steers to keep the
     robot parallel to one wall using front/rear sector distance diff.

  2. Distance-based wall-follow (the PID used during CORNER_APPROACH_WALL)
     — steers to hold a target lateral distance from one wall.

Run modes
---------
--dry-run (default)   : reads LiDAR + prints everything, but NEVER
                         writes steering/speed to the ESP32. Motors
                         will not move. Use this first.
--live                : actually sends STR/SPD packets to the ESP32
                         so you can watch the chassis respond. Make
                         sure the robot is on blocks / in open space
                         before using this.
--mode parallel        : test the parallel-alignment PID (default)
--mode distance         : test the distance-hold wall-follow PID
--side left|right       : which wall to track (default: left)
--speed N                : creep speed to use in --live mode

Ctrl+C always stops the motors and exits cleanly.
"""

import sys
import time
import argparse
import threading
import signal

try:
    import serial
except ImportError:
    serial = None

try:
    from lidar_steering_new import (
        LidarScanner,
        PIDController,
        get_wall_parallel_error,
        get_wall_parallel_sector_stats,
        PARALLEL_TOLERANCE_MM,
    )
except ImportError as e:
    print(f"[FATAL] Could not import lidar_steering_new: {e}")
    sys.exit(1)

# =====================================================================
# EDIT THESE TO CONFIGURE THE DEBUG RUN (no terminal flags needed)
# =====================================================================
MODE = "parallel"     # "parallel"  -> tests the cruise/alignment PID (WALL_ALIGN_CORRECTION)
                       # "distance"  -> tests the corner-approach wall-hold PID (CORNER_APPROACH_WALL)
SIDE = "left"          # "left" or "right" -- which wall to track
LIVE = True           # False -> dry run, only prints, motors do NOT move
                       # True  -> actually sends STR/SPD packets to the ESP32, chassis WILL move
SPEED = 150            # creep speed sent to the ESP32 when LIVE = True
NO_LIDAR_THREAD = False  # True -> poll LiDAR directly in the main loop instead of a background thread
# =====================================================================

# --- Hardware config (match your main script) ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400

# --- Control constants (copied from main script so tuning stays consistent) ---
SERVO_CENTER_ANGLE = 100
WALL_FOLLOW_TARGET_MM = 600      # target lateral distance for distance-hold mode
WALL_ALIGN_CREEP_SPEED = 140
SERVO_CLAMP = 20                 # +/- degrees off center
PID_OUTPUT_CLAMP = 40.0          # sanity clamp on raw PID output (degrees-equivalent) before
                                  # it's subtracted from SERVO_CENTER_ANGLE -- prevents a bad
                                  # dt/derivative-kick frame from ever commanding a wild jump

# --- Shared state ---
shutdown_event = threading.Event()
lidar_lock = threading.Lock()
latest_scan = {}
esp_ser = None
lidar_scanner = None


def lidar_thread_func(scanner):
    global latest_scan
    while not shutdown_event.is_set():
        try:
            data = scanner.get_scan_data()
            if data:
                with lidar_lock:
                    latest_scan = data.copy()
        except Exception as e:
            print(f"[LIDAR THREAD] error: {e}")
        time.sleep(0.01)


def send_packet(ser_port, steering, speed, live):
    """Only actually writes to hardware if live=True."""
    if not live:
        return
    if ser_port and ser_port.is_open:
        try:
            packet = f"STR:{steering},SPD:{speed}\n"
            ser_port.write(packet.encode('utf-8'))
        except Exception as e:
            print(f"[SERIAL] write failed: {e}")


def stop_and_exit(*_):
    print("\n[SHUTDOWN] Stopping motors and disconnecting...")
    shutdown_event.set()
    global esp_ser, lidar_scanner
    if esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
        except Exception:
            pass
    if lidar_scanner:
        try:
            lidar_scanner.disconnect()
        except Exception:
            pass
    print("[SHUTDOWN] Done.")
    sys.exit(0)


def main():
    # NOTE: all values below default to the CONFIG block at the top of this file,
    # so you can just edit MODE / SIDE / LIVE / SPEED up there and run this script
    # with no terminal arguments at all (e.g. by pressing "Run" in your editor).
    # Any CLI flag you DO pass will override the in-file config for that run.
    parser = argparse.ArgumentParser(description="Wall-following debug harness")
    parser.add_argument("--live", action="store_true", default=LIVE, help="Actually send steering/speed to ESP32")
    parser.add_argument("--mode", choices=["parallel", "distance"], default=MODE,
                         help="parallel = alignment PID (cruise mode), distance = wall-follow PID (approach mode)")
    parser.add_argument("--side", choices=["left", "right"], default=SIDE, help="Which wall to track")
    parser.add_argument("--speed", type=int, default=SPEED, help="Creep speed for --live mode")
    parser.add_argument("--no-lidar-thread", action="store_true", default=NO_LIDAR_THREAD,
                         help="Poll LiDAR directly in main loop instead of a background thread (for debugging thread issues)")
    args = parser.parse_args()

    live = args.live
    align_side = args.side

    signal.signal(signal.SIGINT, stop_and_exit)
    signal.signal(signal.SIGQUIT, stop_and_exit)

    global esp_ser, lidar_scanner

    # --- Connect ESP32 serial (needed even in dry-run, to read yaw if you want it later) ---
    if serial is None:
        print("[FATAL] pyserial not installed.")
        sys.exit(1)
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print(f"[INFO] Serial link to ESP32 open on {PI_TO_ESP_PORT}.")
    except Exception as e:
        print(f"[FATAL] Could not open serial port {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # --- Connect LiDAR ---
    try:
        lidar_scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        lidar_scanner.connect()
        print("[INFO] LiDAR connected.")
    except Exception as e:
        print(f"[FATAL] Could not connect LiDAR: {e}")
        sys.exit(1)

    lidar_thread = None
    if not args.no_lidar_thread:
        lidar_thread = threading.Thread(target=lidar_thread_func, args=(lidar_scanner,))
        lidar_thread.daemon = True
        lidar_thread.start()

    print(f"[INFO] Mode: {args.mode} | Side: {align_side} | Live: {live} | Speed: {args.speed}")
    print("[INFO] Press Ctrl+C to stop.\n")

    alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)
    wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)

    # --- WARM-UP PASS ---
    # Wait for real LiDAR data, then feed the relevant PID one "throwaway" update
    # using the actual current error. This seeds the controller's internal
    # last_error/last_time state under real timing conditions, so the FIRST
    # update inside the main loop below doesn't see a near-zero dt and produce
    # a derivative-kick spike (this is what caused the pid=-56668 line you saw).
    print("[INFO] Waiting for LiDAR data to warm up PID state...")
    warm_up_done = False
    warm_up_deadline = time.monotonic() + 5.0
    while not warm_up_done and time.monotonic() < warm_up_deadline and not shutdown_event.is_set():
        if args.no_lidar_thread:
            scan_data = lidar_scanner.get_scan_data() or {}
        else:
            with lidar_lock:
                scan_data = latest_scan.copy()
        if scan_data:
            if args.mode == "parallel":
                err = get_wall_parallel_error(scan_data, align_side)
                if err is not None:
                    normalized_err = err if align_side == "left" else -err
                    alignment_pid.update(normalized_err)
                    warm_up_done = True
            else:
                angle_range = range(-105, -75) if align_side == "left" else range(75, 105)
                wall_pts = [scan_data[a] for a in angle_range if a in scan_data and scan_data[a] > 0]
                if wall_pts:
                    avg_wall = sum(wall_pts) / len(wall_pts)
                    wall_err = (avg_wall - WALL_FOLLOW_TARGET_MM) if align_side == "left" else (WALL_FOLLOW_TARGET_MM - avg_wall)
                    wall_follow_pid.update(wall_err)
                    warm_up_done = True
        if not warm_up_done:
            time.sleep(0.02)
    print(f"[INFO] Warm-up {'complete' if warm_up_done else 'timed out -- proceeding anyway'}.\n")

    frame_count = 0
    loop_period = 0.05  # 20Hz print/update rate, independent of main script's 0.02s

    while not shutdown_event.is_set():
        loop_start = time.monotonic()

        if args.no_lidar_thread:
            try:
                scan_data = lidar_scanner.get_scan_data() or {}
            except Exception as e:
                print(f"[LIDAR] read error: {e}")
                scan_data = {}
        else:
            with lidar_lock:
                scan_data = latest_scan.copy()

        if not scan_data:
            print("[WAIT] No LiDAR data yet...")
            time.sleep(0.05)
            continue

        if args.mode == "parallel":
            front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
            parallel_error = get_wall_parallel_error(scan_data, align_side)
            is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM

            if parallel_error is None:
                target_servo = SERVO_CENTER_ANGLE
                pid_out = None
            else:
                normalized_error = parallel_error if align_side == "left" else -parallel_error
                pid_out = alignment_pid.update(normalized_error)
                pid_out = max(-PID_OUTPUT_CLAMP, min(PID_OUTPUT_CLAMP, pid_out))
                target_servo = SERVO_CENTER_ANGLE - pid_out

            final_servo = int(round(max(SERVO_CENTER_ANGLE - SERVO_CLAMP,
                                         min(SERVO_CENTER_ANGLE + SERVO_CLAMP, target_servo))))

            send_packet(esp_ser, final_servo, args.speed, live)

            if frame_count % 5 == 0:
                err_str = f"{parallel_error:+.1f}mm" if parallel_error is not None else "N/A"
                pid_str = f"{pid_out:+.2f}" if pid_out is not None else "N/A"
                print(f"[PARALLEL] side={align_side} front={front_avg if front_avg is not None else float('nan'):.0f}mm "
                      f"rear={rear_avg if rear_avg is not None else float('nan'):.0f}mm "
                      f"fpts={front_count} rpts={rear_count} err={err_str} pid={pid_str} "
                      f"servo={final_servo} aligned={is_aligned} live={live}")

        else:  # distance mode
            angle_range = range(-105, -75) if align_side == "left" else range(75, 105)
            wall_pts = [scan_data[a] for a in angle_range if a in scan_data and scan_data[a] > 0]

            if wall_pts:
                avg_wall = sum(wall_pts) / len(wall_pts)
                wall_error = (avg_wall - WALL_FOLLOW_TARGET_MM) if align_side == "left" else (WALL_FOLLOW_TARGET_MM - avg_wall)
                pid_out = wall_follow_pid.update(wall_error)
                pid_out = max(-PID_OUTPUT_CLAMP, min(PID_OUTPUT_CLAMP, pid_out))
                target_servo = SERVO_CENTER_ANGLE - pid_out
            else:
                avg_wall = None
                wall_error = None
                pid_out = None
                target_servo = SERVO_CENTER_ANGLE

            final_servo = int(round(max(SERVO_CENTER_ANGLE - SERVO_CLAMP,
                                         min(SERVO_CENTER_ANGLE + SERVO_CLAMP, target_servo))))

            send_packet(esp_ser, final_servo, args.speed, live)

            if frame_count % 5 == 0:
                wall_str = f"{avg_wall:.0f}mm" if avg_wall is not None else "N/A"
                err_str = f"{wall_error:+.1f}mm" if wall_error is not None else "N/A"
                pid_str = f"{pid_out:+.2f}" if pid_out is not None else "N/A"
                print(f"[DISTANCE] side={align_side} target={WALL_FOLLOW_TARGET_MM}mm avg_wall={wall_str} "
                      f"pts={len(wall_pts)} err={err_str} pid={pid_str} servo={final_servo} live={live}")

        frame_count += 1
        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, loop_period - elapsed))

    stop_and_exit()


if __name__ == "__main__":
    main()