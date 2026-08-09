"""
corner_detection_debug.py

STANDALONE DEBUG HARNESS -- Corner Detection + Lane Detection + Arc/Pivot Turn ONLY.

This deliberately strips out everything else from the main robot script:
  - NO camera / vision (image_frame_combine_outer_inner_depth) processing
  - NO Flask video streaming
  - NO blue-line lap counting
  - NO normal wall-following track driving

It keeps ONLY:
  - LiDAR read loop
  - ESP32 serial link (yaw feedback + steering/speed packets)
  - Rear ToF sensor (optional, same as main)
  - Corner "signature" detection (front wall + left/right asymmetry)
  - Lane detection (locked wall-reference distance -> Lane 1 vs Lane 2/3)
  - Corner approach (wall-following toward the 55cm second-stop trigger)
  - Arc pivot OR in-place pivot execution
  - Post-pivot parallel wall alignment
  - Backward alignment phase
  - Forward WALL_ALIGN_CORRECTION handoff back to "IDLE / would resume driving" state

WHY THIS EXISTS:
  Debugging the corner/lane/arc-vs-pivot decision inside the full main.py is hard because
  camera frames, Flask streaming, and vision processing all compete for attention in the
  logs and CPU. This file removes all of that so you can watch JUST the corner state
  machine's decisions in isolation, with much more verbose per-state logging.

SAFETY:
  Set DRY_RUN = True (default) to print every command that WOULD be sent to the ESP32
  instead of actually writing to the serial port. Flip to False only once you've
  confirmed the lane/arc/pivot decisions look correct in the logs, and always test on
  blocks / with the robot raised off the ground first.

USAGE:
  python3 corner_detection_debug.py
  (Ctrl+C to stop -- sends a safe center/zero-speed packet before exiting, same as main.py)


forward distance <1000
gyro should be set to 0 compensated angles
gyro following to reach the second brake ==>2 cases-- wall_follow_threshold to set > 60, set wall_thr = 600, else it is 400 to get straight
or use post wall maneuver



"""

# import sys
# import time
# import threading
# import signal
# import numpy as np
# import serial

# try:
#     from lidar_steering_new import (
#         LidarScanner,
#         PIDController,
#         get_wall_parallel_error,
#         get_wall_parallel_sector_stats,
#         PARALLEL_TOLERANCE_MM,
#     )
# except ImportError as e:
#     print(f"[SYSTEM ERROR] Failed to import lidar_steering_new: {e}")
#     sys.exit(1)

# # ============================================================
# # SAFETY SWITCH -- keep True until decisions look right in logs
# # ============================================================
# DRY_RUN = False

# # --- HARDWARE PORTS ---
# PI_TO_ESP_PORT = "/dev/ttyAMA0"
# BAUD_RATE_ESP = 115200
# LIDAR_PORT = "/dev/ttyUSB0"
# LIDAR_BAUD = 230400

# # --- CONTROL CONSTANTS (copied from main.py, corner-relevant subset only) ---
# SERVO_CENTER_ANGLE = 100
# ROBOT_CRUISE_SPEED = 180
# ROBOT_MANEUVER_SPEED = 155
# CORNER_PIVOT_SPEED = 140
# CORNER_BRAKE_DELAY = 0.25
# CORNER_PIVOT_SAFETY_TIMEOUT = 2.5
# CORNER_BACKWARD_DURATION = 4
# CORNER_BACKWARD_TOF_TARGET_MM = 190.0
# CORNER_BACKWARD_TOF_TOLERANCE_MM = 10.0
# CORNER_DETECTION_COOLDOWN_SEC = 10
# FRONT_SCAN_ANGLE_DEG = 15
# WALL_LOSS_THRESHOLD_MM = 400.0
# WALL_FOLLOW_TARGET_MM = 600

# # --- CORNER SIGNATURE / LANE DETECTION ---
# CORNER_SIGNATURE_STOP_DELAY_SEC = 0.25
# LANE_REFERENCE_ARC_THRESHOLD_MM = 600.0   # 60cm lane-1 cutoff
# CORNER_APPROACH_TRIGGER_MM = 400.0        # 55cm second-stop trigger (both pivot & arc)

# # --- ARC PARAMETERS ---
# CORNER_ARC_STEER_OFFSET = 60
# CORNER_ARC_PIVOT_SPEED = 120
# CORNER_ARC_PIVOT_SAFETY_TIMEOUT = 4.0
# CORNER_ARC_CLEAR_VIEW_TOF_MM = 400.0

# TURN_TARGET_RIGHT_DEGREES = 80.0
# TURN_TARGET_LEFT_DEGREES = 80.0
# SERVO_HARD_RIGHT = 180
# SERVO_HARD_LEFT = 0
# WALL_ALIGN_CREEP_SPEED = 140
# WALL_ALIGN_SAFETY_TIMEOUT = 3.0
# WALL_ALIGN_NO_WALL_TIMEOUT = 1.0

# STATE_LOG_EVERY_N_FRAMES = 5   # more verbose than main.py, since this is a debug tool

# # --- GLOBALS ---
# global_shutdown_event = threading.Event()
# esp_ser = None
# lidar_scanner = None
# tof_sensor = None

# latest_lidar_data = {}
# lidar_data_lock = threading.Lock()
# latest_tof_distance_mm = None
# tof_data_lock = threading.Lock()

# current_yaw = 0.0
# CLOCKWISE_WALL_FOLLOWING = True   # flip manually here if you want to force/test a course direction


# class DebugState:
#     WATCHING = "WATCHING"                      # normal loop, waiting for a corner signature
#     CORNER_PRE_ALIGN = "CORNER_PRE_ALIGN"      # NEW: straighten against wall before approach
#     CORNER_APPROACH_WALL = "CORNER_APPROACH_WALL"
#     CORNER_ACTIVE_PIVOT = "CORNER_ACTIVE_PIVOT"
#     CORNER_ARC_ACTIVE_PIVOT = "CORNER_ARC_ACTIVE_PIVOT"
#     CORNER_POST_PIVOT_ALIGN = "CORNER_POST_PIVOT_ALIGN"
#     CORNER_ALIGN_BACKWARD = "CORNER_ALIGN_BACKWARD"
#     WALL_ALIGN_CORRECTION = "WALL_ALIGN_CORRECTION"
#     DONE = "DONE"


# # ============================================================
# # SERIAL / SAFETY HELPERS
# # ============================================================
# def send_esp_packet(steering, speed, tag=""):
#     """Sends (or, in DRY_RUN, just prints) a steering/speed packet."""
#     packet = f"STR:{steering},SPD:{speed}\n"
#     if DRY_RUN:
#         print(f"    [DRY_RUN -> ESP32] {packet.strip()}  {('(' + tag + ')') if tag else ''}")
#         return
#     if esp_ser and esp_ser.is_open and not global_shutdown_event.is_set():
#         try:
#             esp_ser.write(packet.encode('utf-8'))
#         except Exception as e:
#             print(f"[SERIAL WRITE ERROR] {e}")


# def send_reset_yaw():
#     if DRY_RUN:
#         print("    [DRY_RUN -> ESP32] RST_YAW")
#         return
#     if esp_ser:
#         esp_ser.write(b"RST_YAW\n")
#         esp_ser.flush()


# def emergency_shutdown_handler(signum, frame):
#     print("\n[EMERGENCY BRAKE] Shutdown signal captured. Halting...")
#     global_shutdown_event.set()
#     if not DRY_RUN and esp_ser and esp_ser.is_open:
#         try:
#             for _ in range(3):
#                 esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
#                 esp_ser.flush()
#                 time.sleep(0.03)
#             esp_ser.close()
#         except Exception as e:
#             print(f"[CLEANUP ERROR] {e}")
#     if lidar_scanner:
#         try:
#             lidar_scanner.disconnect()
#         except Exception:
#             pass
#     print("[SUCCESS] Debug harness exited clean.\n")
#     sys.exit(0)


# signal.signal(signal.SIGINT, emergency_shutdown_handler)
# signal.signal(signal.SIGQUIT, emergency_shutdown_handler)


# # ============================================================
# # BACKGROUND THREADS
# # ============================================================
# def lidar_acquisition_thread_func(scanner_instance):
#     global latest_lidar_data
#     print("[SYSTEM] LiDAR background thread active.")
#     while not global_shutdown_event.is_set():
#         data = scanner_instance.get_scan_data()
#         if data:
#             with lidar_data_lock:
#                 latest_lidar_data = data.copy()
#         time.sleep(0.01)


# def tof_acquisition_thread_func(sensor_instance):
#     global latest_tof_distance_mm
#     print("[SYSTEM] ToF background thread active.")
#     while not global_shutdown_event.is_set():
#         try:
#             dist = sensor_instance.range
#             with tof_data_lock:
#                 latest_tof_distance_mm = dist
#         except Exception:
#             with tof_data_lock:
#                 latest_tof_distance_mm = None
#         time.sleep(0.03)


# def get_compensated_front_distance(scan_data, yaw):
#     if not scan_data:
#         return 2000.0
#     yaw_offset = int(round(yaw))
#     dynamic_angles = range(-FRONT_SCAN_ANGLE_DEG + yaw_offset, FRONT_SCAN_ANGLE_DEG + yaw_offset + 1)
#     yaw_rad = np.radians(yaw)
#     pts = []
#     for a in dynamic_angles:
#         if a in scan_data and scan_data[a] > 0:
#             pts.append(scan_data[a] * np.cos(yaw_rad))
#     return sum(pts) / len(pts) if pts else 2000.0


# # ============================================================
# # MAIN DEBUG LOOP
# # ============================================================
# def corner_debug_loop():
#     global esp_ser, lidar_scanner, tof_sensor, current_yaw

#     print(f"\n{'='*60}")
#     print(f"  CORNER DETECTION DEBUG HARNESS   |  DRY_RUN = {DRY_RUN}")
#     print(f"{'='*60}\n")

#     # --- Serial (still connects even in DRY_RUN, just for yaw feedback) ---
#     try:
#         esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
#         print(f"[INFO] Serial connected on {PI_TO_ESP_PORT}.")
#     except Exception as e:
#         print(f"[WARN] Serial unavailable ({e}). Yaw will stay at 0.0 -- pivot-degree exits "
#               f"will rely on timeouts only.")
#         esp_ser = None

#     # --- LiDAR ---
#     try:
#         lidar_scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
#         lidar_scanner.connect()
#         threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,), daemon=True).start()
#         print("[INFO] LiDAR connected.")
#     except Exception as e:
#         print(f"[FATAL] LiDAR required for this debug tool but failed to connect: {e}")
#         sys.exit(1)

#     # --- Rear ToF (optional) ---
#     try:
#         import board
#         import busio
#         import adafruit_vl53l0x
#         i2c_tof = busio.I2C(board.SCL, board.SDA)
#         tof_sensor = adafruit_vl53l0x.VL53L0X(i2c_tof)
#         threading.Thread(target=tof_acquisition_thread_func, args=(tof_sensor,), daemon=True).start()
#         print("[INFO] Rear ToF connected.")
#     except Exception as e:
#         print(f"[WARN] ToF unavailable ({e}). Backward/arc-clear-view exits use timeout only.")
#         tof_sensor = None

#     wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)
#     gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
#     alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)

#     state = DebugState.WATCHING
#     turn_direction = None
#     use_reverse_arc = False
#     locked_lane_reference_mm = None
#     lane_number = None
#     baseline_start_yaw = 0.0
#     pivot_phase_start_time = 0.0
#     post_pivot_align_start_time = 0.0
#     post_pivot_align_no_wall_start_time = 0.0
#     backward_phase_start_time = 0.0
#     align_phase_start_time = 0.0
#     align_no_wall_start_time = 0.0
#     pre_align_start_time = 0.0
#     pre_align_no_wall_start_time = 0.0
#     corner_cooldown_end_time = 0.0
#     frame_counter = 0
#     corners_detected_count = 0

#     print("[SYSTEM] Watching for corner signature... (Ctrl+C to stop)\n")

#     try:
#         while not global_shutdown_event.is_set():
#             frame_counter += 1
#             should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

#             # --- Read yaw from ESP32 if available ---
#             if esp_ser:
#                 while esp_ser.in_waiting > 0:
#                     try:
#                         raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
#                         if raw_line.startswith("YAW:"):
#                             current_yaw = float(raw_line.split(":")[1])
#                     except Exception:
#                         pass

#             with lidar_data_lock:
#                 scan_data = latest_lidar_data.copy()

#             avg_front_baseline = get_compensated_front_distance(scan_data, current_yaw)
#             left_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
#             right_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
#             avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
#             avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
#             in_cooldown = time.monotonic() < corner_cooldown_end_time

#             # ========================================================
#             # STATE: WATCHING -- waiting for corner signature
#             # ========================================================
#             if state == DebugState.WATCHING:
#                 if should_log:
#                     cooldown_note = f" | COOLDOWN {corner_cooldown_end_time - time.monotonic():.1f}s" if in_cooldown else ""
#                     print(f"[WATCHING] Front:{avg_front_baseline:7.1f}mm  Left:{avg_left:7.1f}mm  "
#                           f"Right:{avg_right:7.1f}mm  Yaw:{current_yaw:+6.1f}°{cooldown_note}")

#                 is_corner_signature = (
#                     not in_cooldown
#                     and avg_front_baseline <= 1000.0
#                     and ((avg_left < 950.0 and avg_right > 1600.0)
#                          or (avg_right < 900.0 and avg_left > 1800.0))
#                 )

#                 if is_corner_signature:
#                     corners_detected_count += 1
#                     print(f"\n{'*'*60}")
#                     print(f"[CORNER SIGNATURE #{corners_detected_count}] Front:{avg_front_baseline:.1f}mm "
#                           f"Left:{avg_left:.1f}mm Right:{avg_right:.1f}mm")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "signature brake")
#                     time.sleep(CORNER_SIGNATURE_STOP_DELAY_SEC)

#                     if avg_left < avg_right:
#                         turn_direction = "RIGHT"
#                     else:
#                         turn_direction = "LEFT"
#                     print(f"  -> Turn direction: {turn_direction}  (CW course: {CLOCKWISE_WALL_FOLLOWING})")

#                     locked_lane_reference_mm = avg_left if CLOCKWISE_WALL_FOLLOWING else avg_right
#                     tracked_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

#                     if locked_lane_reference_mm > LANE_REFERENCE_ARC_THRESHOLD_MM:
#                         lane_number = 1
#                         use_reverse_arc = (turn_direction == "RIGHT")
#                         if turn_direction != "RIGHT":
#                             print(f"  -> Lane 1 on a LEFT turn: arc not tuned for this side -> falling back to PIVOT.")
#                     else:
#                         lane_number = "2/3"
#                         use_reverse_arc = False

#                     print(f"  -> Locked {tracked_side} wall distance: {locked_lane_reference_mm:.0f}mm "
#                           f"(threshold {LANE_REFERENCE_ARC_THRESHOLD_MM:.0f}mm)")
#                     print(f"  -> LANE = {lane_number}   ->   MANEUVER = {'ARC TURN' if use_reverse_arc else 'PIVOT TURN'}")
#                     print(f"{'*'*60}\n")

#                     send_reset_yaw()
#                     time.sleep(0.1)
#                     current_yaw = 0.0
#                     baseline_start_yaw = 0.0
#                     # NEW: straighten against the tracked wall first, before starting the
#                     # wall-following approach toward the 55cm second-stop trigger.
#                     pre_align_start_time = time.monotonic()
#                     pre_align_no_wall_start_time = 0.0
#                     state = DebugState.CORNER_PRE_ALIGN
#                     send_esp_packet(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED, "pre-align start")

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: CORNER_PRE_ALIGN -- straighten against wall BEFORE approach
#             # ========================================================
#             if state == DebugState.CORNER_PRE_ALIGN:
#                 align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
#                 front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
#                 parallel_error = get_wall_parallel_error(scan_data, align_side)
#                 elapsed = time.monotonic() - pre_align_start_time

#                 if parallel_error is None:
#                     if pre_align_no_wall_start_time == 0.0:
#                         pre_align_no_wall_start_time = time.monotonic()
#                     no_wall_elapsed = time.monotonic() - pre_align_no_wall_start_time
#                 else:
#                     pre_align_no_wall_start_time = 0.0
#                     no_wall_elapsed = 0.0

#                 is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
#                 hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
#                 no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

#                 if should_log:
#                     err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
#                     print(f"[PRE-ALIGN] Side:{align_side} Front:{front_avg} Rear:{rear_avg} "
#                           f"Err:{err_str} FrontPts:{front_count} RearPts:{rear_count} Elapsed:{elapsed:.2f}s")

#                 if is_aligned or hard_timeout or no_wall_timeout:
#                     reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
#                     print(f"[PRE-ALIGN] Exit ({reason}). Straightened -> starting approach.")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "pre-align done brake")
#                     time.sleep(CORNER_BRAKE_DELAY)
#                     alignment_pid.reset()
#                     state = DebugState.CORNER_APPROACH_WALL
#                     send_esp_packet(SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED, "approach start")
#                 else:
#                     if parallel_error is None:
#                         send_esp_packet(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
#                     else:
#                         normalized_error = parallel_error if align_side == "left" else -parallel_error
#                         pid_output = alignment_pid.update(normalized_error)
#                         target = SERVO_CENTER_ANGLE - pid_output
#                         final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
#                         send_esp_packet(final_servo, WALL_ALIGN_CREEP_SPEED)

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: CORNER_APPROACH_WALL -- wall-follow to 55cm trigger
#             # ========================================================
#             if state == DebugState.CORNER_APPROACH_WALL:
#                 if avg_front_baseline < CORNER_APPROACH_TRIGGER_MM:
#                     print(f"[APPROACH] Trigger reached: Front {avg_front_baseline:.1f}mm < "
#                           f"{CORNER_APPROACH_TRIGGER_MM:.0f}mm. Braking...")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "approach brake")
#                     time.sleep(CORNER_BRAKE_DELAY)
#                     send_reset_yaw()
#                     time.sleep(0.1)
#                     pivot_phase_start_time = time.monotonic()
#                     baseline_start_yaw = current_yaw

#                     if use_reverse_arc:
#                         arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
#                         print(f"[APPROACH] -> Entering ARC pivot (Lane 1). steer={arc_steer_angle}° reverse_speed={CORNER_ARC_PIVOT_SPEED}")
#                         state = DebugState.CORNER_ARC_ACTIVE_PIVOT
#                         send_esp_packet(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED, "arc pivot")
#                     else:
#                         final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
#                         print(f"[APPROACH] -> Entering IN-PLACE pivot (Lane {lane_number}). servo={final_servo}°")
#                         send_esp_packet(final_servo, 0, "lock wheels")
#                         time.sleep(0.15)
#                         state = DebugState.CORNER_ACTIVE_PIVOT
#                         send_esp_packet(final_servo, CORNER_PIVOT_SPEED, "pivot")
#                 else:
#                     if CLOCKWISE_WALL_FOLLOWING:
#                         wall_pts = left_pts
#                     else:
#                         wall_pts = right_pts

#                     if wall_pts:
#                         avg_wall = sum(wall_pts) / len(wall_pts)
#                         if avg_wall > WALL_LOSS_THRESHOLD_MM:
#                             heading_error = 0.0 - current_yaw
#                             pid_output = gyro_straight_pid.update(heading_error)
#                             approach_servo = SERVO_CENTER_ANGLE - pid_output
#                             mode_note = "wall lost -> gyro straight"
#                         else:
#                             wall_error = (avg_wall - WALL_FOLLOW_TARGET_MM) if CLOCKWISE_WALL_FOLLOWING else (WALL_FOLLOW_TARGET_MM - avg_wall)
#                             pid_output = wall_follow_pid.update(wall_error)
#                             approach_servo = SERVO_CENTER_ANGLE - pid_output
#                             mode_note = f"wall follow, err={wall_error:.0f}mm"
#                     else:
#                         heading_error = 0.0 - current_yaw
#                         pid_output = gyro_straight_pid.update(heading_error)
#                         approach_servo = SERVO_CENTER_ANGLE - pid_output
#                         mode_note = "no wall data -> gyro straight"

#                     final_approach_servo = int(round(np.clip(approach_servo, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
#                     if should_log:
#                         print(f"[APPROACH] Front:{avg_front_baseline:7.1f}mm / {CORNER_APPROACH_TRIGGER_MM:.0f}mm "
#                               f"| {mode_note} | servo={final_approach_servo}")
#                     send_esp_packet(final_approach_servo, ROBOT_CRUISE_SPEED)

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: CORNER_ACTIVE_PIVOT -- in-place pivot
#             # ========================================================
#             if state == DebugState.CORNER_ACTIVE_PIVOT:
#                 elapsed = time.monotonic() - pivot_phase_start_time
#                 yaw_delta = current_yaw - baseline_start_yaw
#                 target_deg = TURN_TARGET_RIGHT_DEGREES if turn_direction == "RIGHT" else TURN_TARGET_LEFT_DEGREES
#                 pivot_complete = (yaw_delta <= -target_deg) if turn_direction == "RIGHT" else (yaw_delta >= target_deg)
#                 timed_out = elapsed >= CORNER_PIVOT_SAFETY_TIMEOUT

#                 if should_log:
#                     print(f"[PIVOT] Yaw:{yaw_delta:+.1f}° / target {target_deg:.0f}° ({turn_direction}) | Elapsed:{elapsed:.2f}s")

#                 if pivot_complete or timed_out:
#                     if timed_out and not pivot_complete:
#                         print(f"[PIVOT] WARNING: timeout before target reached ({yaw_delta:+.1f}°).")
#                     print(f"[PIVOT] Complete ({yaw_delta:+.1f}°). Braking -> post-pivot align.")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "pivot done brake")
#                     time.sleep(CORNER_BRAKE_DELAY)
#                     send_reset_yaw()
#                     time.sleep(0.1)
#                     current_yaw = 0.0
#                     post_pivot_align_start_time = time.monotonic()
#                     post_pivot_align_no_wall_start_time = 0.0
#                     state = DebugState.CORNER_POST_PIVOT_ALIGN
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0)
#                 else:
#                     final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
#                     send_esp_packet(final_servo, CORNER_PIVOT_SPEED)

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: CORNER_ARC_ACTIVE_PIVOT -- reverse arc pivot
#             # ========================================================
#             if state == DebugState.CORNER_ARC_ACTIVE_PIVOT:
#                 elapsed = time.monotonic() - pivot_phase_start_time
#                 yaw_delta = current_yaw - baseline_start_yaw
#                 arc_complete = yaw_delta <= -TURN_TARGET_RIGHT_DEGREES
#                 timed_out = elapsed >= CORNER_ARC_PIVOT_SAFETY_TIMEOUT

#                 with tof_data_lock:
#                     rear_distance = latest_tof_distance_mm
#                 rear_clear_view_hit = rear_distance is not None and rear_distance <= CORNER_ARC_CLEAR_VIEW_TOF_MM

#                 if should_log:
#                     print(f"[ARC PIVOT] Yaw:{yaw_delta:+.1f}° / -{TURN_TARGET_RIGHT_DEGREES:.0f}° | "
#                           f"Rear:{rear_distance} | Elapsed:{elapsed:.2f}s")

#                 if arc_complete or timed_out or rear_clear_view_hit:
#                     if timed_out and not arc_complete:
#                         print(f"[ARC PIVOT] WARNING: timeout before target reached ({yaw_delta:+.1f}°).")
#                     if rear_clear_view_hit:
#                         print(f"[ARC PIVOT] Rear ToF clear-view hit ({rear_distance:.0f}mm <= "
#                               f"{CORNER_ARC_CLEAR_VIEW_TOF_MM:.0f}mm). Ending arc early.")
#                     print(f"[ARC PIVOT] Complete ({yaw_delta:+.1f}°). Braking -> post-pivot align.")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "arc done brake")
#                     time.sleep(CORNER_BRAKE_DELAY)
#                     send_reset_yaw()
#                     time.sleep(0.1)
#                     current_yaw = 0.0
#                     post_pivot_align_start_time = time.monotonic()
#                     post_pivot_align_no_wall_start_time = 0.0
#                     state = DebugState.CORNER_POST_PIVOT_ALIGN
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0)
#                 else:
#                     arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
#                     send_esp_packet(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: CORNER_POST_PIVOT_ALIGN -- LiDAR parallel alignment (reverse creep)
#             # ========================================================
#             if state == DebugState.CORNER_POST_PIVOT_ALIGN:
#                 align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
#                 front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
#                 parallel_error = get_wall_parallel_error(scan_data, align_side)
#                 elapsed = time.monotonic() - post_pivot_align_start_time

#                 if parallel_error is None:
#                     if post_pivot_align_no_wall_start_time == 0.0:
#                         post_pivot_align_no_wall_start_time = time.monotonic()
#                     no_wall_elapsed = time.monotonic() - post_pivot_align_no_wall_start_time
#                 else:
#                     post_pivot_align_no_wall_start_time = 0.0
#                     no_wall_elapsed = 0.0

#                 is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
#                 hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
#                 no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

#                 if should_log:
#                     err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
#                     print(f"[POST-PIVOT ALIGN] Side:{align_side} Front:{front_avg} Rear:{rear_avg} "
#                           f"Err:{err_str} FrontPts:{front_count} RearPts:{rear_count} Elapsed:{elapsed:.2f}s")

#                 if is_aligned or hard_timeout or no_wall_timeout:
#                     reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
#                     print(f"[POST-PIVOT ALIGN] Exit ({reason}). -> backward phase.")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "align done brake")
#                     time.sleep(CORNER_BRAKE_DELAY)
#                     alignment_pid.reset()
#                     backward_phase_start_time = time.monotonic()
#                     state = DebugState.CORNER_ALIGN_BACKWARD
#                     send_esp_packet(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED, "backward start")
#                 else:
#                     if parallel_error is None:
#                         send_esp_packet(SERVO_CENTER_ANGLE, -WALL_ALIGN_CREEP_SPEED)
#                     else:
#                         normalized_error = parallel_error if align_side == "left" else -parallel_error
#                         pid_output = -alignment_pid.update(normalized_error)
#                         target = SERVO_CENTER_ANGLE - pid_output
#                         final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
#                         send_esp_packet(final_servo, -WALL_ALIGN_CREEP_SPEED)

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: CORNER_ALIGN_BACKWARD -- back off to final ToF distance
#             # ========================================================
#             if state == DebugState.CORNER_ALIGN_BACKWARD:
#                 elapsed = time.monotonic() - backward_phase_start_time
#                 with tof_data_lock:
#                     rear_distance = latest_tof_distance_mm
#                 tof_reached = rear_distance is not None and rear_distance <= (CORNER_BACKWARD_TOF_TARGET_MM + CORNER_BACKWARD_TOF_TOLERANCE_MM)
#                 hard_timeout = elapsed >= CORNER_BACKWARD_DURATION

#                 if should_log:
#                     print(f"[BACKWARD] Rear:{rear_distance} / {CORNER_BACKWARD_TOF_TARGET_MM:.0f}mm Elapsed:{elapsed:.2f}s")

#                 if tof_reached or hard_timeout:
#                     if hard_timeout and not tof_reached:
#                         print(f"[BACKWARD] WARNING: timeout before ToF target reached (rear={rear_distance}).")
#                     else:
#                         print(f"[BACKWARD] ToF target reached (rear={rear_distance:.0f}mm).")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "backward done brake")
#                     time.sleep(0.3)
#                     send_reset_yaw()
#                     time.sleep(0.1)
#                     current_yaw = 0.0
#                     corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
#                     wall_follow_pid.reset()
#                     gyro_straight_pid.reset()
#                     alignment_pid.reset()
#                     align_phase_start_time = time.monotonic()
#                     align_no_wall_start_time = 0.0
#                     state = DebugState.WALL_ALIGN_CORRECTION
#                     turn_direction = None
#                     locked_lane_reference_mm = None
#                     lane_number = None
#                 else:
#                     send_esp_packet(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)

#                 time.sleep(0.02)
#                 continue

#             # ========================================================
#             # STATE: WALL_ALIGN_CORRECTION -- forward parallel re-align, then back to WATCHING
#             # ========================================================
#             if state == DebugState.WALL_ALIGN_CORRECTION:
#                 align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
#                 parallel_error = get_wall_parallel_error(scan_data, align_side)
#                 elapsed = time.monotonic() - align_phase_start_time

#                 if parallel_error is None:
#                     if align_no_wall_start_time == 0.0:
#                         align_no_wall_start_time = time.monotonic()
#                     no_wall_elapsed = time.monotonic() - align_no_wall_start_time
#                 else:
#                     align_no_wall_start_time = 0.0
#                     no_wall_elapsed = 0.0

#                 is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
#                 hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
#                 no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

#                 if should_log:
#                     err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
#                     print(f"[WALL ALIGN] Side:{align_side} Err:{err_str} Elapsed:{elapsed:.2f}s")

#                 if is_aligned or hard_timeout or no_wall_timeout:
#                     reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
#                     print(f"[WALL ALIGN] Exit ({reason}). Corner maneuver COMPLETE. -> back to WATCHING.\n")
#                     send_esp_packet(SERVO_CENTER_ANGLE, 0, "resume watching")
#                     alignment_pid.reset()
#                     state = DebugState.WATCHING
#                     continue

#                 if parallel_error is None:
#                     send_esp_packet(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
#                 else:
#                     normalized_error = parallel_error if align_side == "left" else -parallel_error
#                     pid_output = alignment_pid.update(normalized_error)
#                     target = SERVO_CENTER_ANGLE - pid_output
#                     final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
#                     send_esp_packet(final_servo, WALL_ALIGN_CREEP_SPEED)

#                 time.sleep(0.02)
#                 continue

#     except Exception as e:
#         print(f"[SYSTEM FAILURE] {e}")
#     finally:
#         emergency_shutdown_handler(None, None)


# if __name__ == '__main__':
#     corner_debug_loop()


import sys
import time
import threading
import signal
import numpy as np
import serial

try:
    from lidar_steering_new import (
        LidarScanner,
        PIDController,
        get_wall_parallel_error,
        get_wall_parallel_sector_stats,
        PARALLEL_TOLERANCE_MM,
    )
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to import lidar_steering_new: {e}")
    sys.exit(1)

# ============================================================
# SAFETY SWITCH -- keep True until decisions look right in logs
# ============================================================
DRY_RUN = False

# --- HARDWARE PORTS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400

# --- CONTROL CONSTANTS (copied from main.py, corner-relevant subset only) ---
SERVO_CENTER_ANGLE = 100
ROBOT_CRUISE_SPEED = 180
ROBOT_MANEUVER_SPEED = 185 # backward speed =155
CORNER_PIVOT_SPEED = 180       #140 -- gryo yaw = 80
CORNER_BRAKE_DELAY = 0.20
CORNER_PIVOT_SAFETY_TIMEOUT = 2.5
CORNER_BACKWARD_DURATION = 1.5
CORNER_BACKWARD_TOF_TARGET_MM = 190.0
CORNER_BACKWARD_TOF_TOLERANCE_MM = 10.0
CORNER_DETECTION_COOLDOWN_SEC = 10
FRONT_SCAN_ANGLE_DEG = 15
WALL_LOSS_THRESHOLD_MM = 400.0
WALL_FOLLOW_TARGET_MM = 600

# --- CORNER SIGNATURE / LANE DETECTION ---
CORNER_SIGNATURE_STOP_DELAY_SEC = 0.25
LANE_REFERENCE_ARC_THRESHOLD_MM = 600.0   # 60cm lane-1 cutoff
CORNER_APPROACH_TRIGGER_MM = 350.0        # 40cm second-stop trigger (both pivot & arc)

# --- ARC PARAMETERS ---
CORNER_ARC_STEER_OFFSET = 60
CORNER_ARC_PIVOT_SPEED = 200   #120 - y=80
CORNER_ARC_PIVOT_SAFETY_TIMEOUT = 2.0
CORNER_ARC_CLEAR_VIEW_TOF_MM = 400.0

TURN_TARGET_RIGHT_DEGREES = 70.0   # 140 --- gyro yaw = 80
TURN_TARGET_LEFT_DEGREES = 70.0
SERVO_HARD_RIGHT = 180
SERVO_HARD_LEFT = 0
WALL_ALIGN_CREEP_SPEED = 140
WALL_ALIGN_SAFETY_TIMEOUT = 3.0
WALL_ALIGN_NO_WALL_TIMEOUT = 1.0

STATE_LOG_EVERY_N_FRAMES = 5   # more verbose than main.py, since this is a debug tool

# --- GLOBALS ---
global_shutdown_event = threading.Event()
esp_ser = None
lidar_scanner = None
tof_sensor = None

latest_lidar_data = {}
lidar_data_lock = threading.Lock()
last_lidar_update_time = 0.0   # NEW: tracks freshness of latest_lidar_data
LIDAR_STALE_TIMEOUT_SEC = 0.5  # NEW: if no fresh scan in this long, treat as LiDAR lost
latest_tof_distance_mm = None
tof_data_lock = threading.Lock()

current_yaw = 0.0
CLOCKWISE_WALL_FOLLOWING = True   # flip manually here if you want to force/test a course direction


class DebugState:
    WATCHING = "WATCHING"                      # normal loop, waiting for a corner signature
    CORNER_PRE_ALIGN = "CORNER_PRE_ALIGN"      # straighten against wall before approach
    CORNER_APPROACH_WALL = "CORNER_APPROACH_WALL"
    CORNER_ACTIVE_PIVOT = "CORNER_ACTIVE_PIVOT"
    CORNER_ARC_ACTIVE_PIVOT = "CORNER_ARC_ACTIVE_PIVOT"
    CORNER_POST_PIVOT_ALIGN = "CORNER_POST_PIVOT_ALIGN"
    CORNER_ALIGN_BACKWARD = "CORNER_ALIGN_BACKWARD"
    WALL_ALIGN_CORRECTION = "WALL_ALIGN_CORRECTION"
    DONE = "DONE"


# ============================================================
# SERIAL / SAFETY HELPERS
# ============================================================
def send_esp_packet(steering, speed, tag=""):
    """Sends (or, in DRY_RUN, just prints) a steering/speed packet."""
    packet = f"STR:{steering},SPD:{speed}\n"
    if DRY_RUN:
        print(f"    [DRY_RUN -> ESP32] {packet.strip()}  {('(' + tag + ')') if tag else ''}")
        return
    if esp_ser and esp_ser.is_open and not global_shutdown_event.is_set():
        try:
            esp_ser.write(packet.encode('utf-8'))
        except Exception as e:
            print(f"[SERIAL WRITE ERROR] {e}")


def send_reset_yaw():
    if DRY_RUN:
        print("    [DRY_RUN -> ESP32] RST_YAW")
        return
    if esp_ser:
        esp_ser.write(b"RST_YAW\n")
        esp_ser.flush()


def emergency_shutdown_handler(signum, frame):
    print("\n[EMERGENCY BRAKE] Shutdown signal captured. Halting...")
    global_shutdown_event.set()
    if not DRY_RUN and esp_ser and esp_ser.is_open:
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
        except Exception as e:
            print(f"[CLEANUP ERROR] {e}")
    if lidar_scanner:
        try:
            lidar_scanner.disconnect()
        except Exception:
            pass
    print("[SUCCESS] Debug harness exited clean.\n")
    sys.exit(0)


signal.signal(signal.SIGINT, emergency_shutdown_handler)
signal.signal(signal.SIGQUIT, emergency_shutdown_handler)


# ============================================================
# BACKGROUND THREADS
# ============================================================
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, last_lidar_update_time
    print("[SYSTEM] LiDAR background thread active.")
    consecutive_failures = 0
    RECONNECT_AFTER_N_FAILURES = 15   # ~0.15-1.5s of failures depending on read speed, before trying a reconnect

    while not global_shutdown_event.is_set():
        try:
            data = scanner_instance.get_scan_data()
        except Exception as e:
            print(f"[LIDAR THREAD] get_scan_data() raised: {e}")
            data = None

        if data:
            with lidar_data_lock:
                latest_lidar_data = data.copy()
            last_lidar_update_time = time.monotonic()
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if consecutive_failures == 1:
                print("[LIDAR THREAD] WARNING: scan read failed (device may have disconnected).")
            if consecutive_failures >= RECONNECT_AFTER_N_FAILURES:
                print(f"[LIDAR THREAD] {consecutive_failures} consecutive failures -- "
                      f"attempting reconnect...")
                try:
                    scanner_instance.disconnect()
                except Exception as e:
                    print(f"[LIDAR THREAD] disconnect() during reconnect raised: {e}")
                time.sleep(0.5)
                try:
                    scanner_instance.connect()
                    print("[LIDAR THREAD] Reconnect succeeded.")
                    consecutive_failures = 0
                except Exception as e:
                    print(f"[LIDAR THREAD] Reconnect FAILED: {e}. Will retry.")
                    time.sleep(1.0)  # back off before hammering reconnect attempts
        time.sleep(0.01)


def tof_acquisition_thread_func(sensor_instance):
    global latest_tof_distance_mm
    print("[SYSTEM] ToF background thread active.")
    while not global_shutdown_event.is_set():
        try:
            dist = sensor_instance.range
            with tof_data_lock:
                latest_tof_distance_mm = dist
        except Exception:
            with tof_data_lock:
                latest_tof_distance_mm = None
        time.sleep(0.03)


def get_compensated_front_distance(scan_data, yaw):
    if not scan_data:
        return 2000.0
    yaw_offset = int(round(yaw))
    dynamic_angles = range(-FRONT_SCAN_ANGLE_DEG + yaw_offset, FRONT_SCAN_ANGLE_DEG + yaw_offset + 1)
    yaw_rad = np.radians(yaw)
    pts = []
    for a in dynamic_angles:
        if a in scan_data and scan_data[a] > 0:
            pts.append(scan_data[a] * np.cos(yaw_rad))
    return sum(pts) / len(pts) if pts else 2000.0


# ============================================================
# MAIN DEBUG LOOP
# ============================================================
def corner_debug_loop():
    global esp_ser, lidar_scanner, tof_sensor, current_yaw

    print(f"\n{'='*60}")
    print(f"  CORNER DETECTION DEBUG HARNESS   |  DRY_RUN = {DRY_RUN}")
    print(f"{'='*60}\n")

    # --- Serial (still connects even in DRY_RUN, just for yaw feedback) ---
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print(f"[INFO] Serial connected on {PI_TO_ESP_PORT}.")
    except Exception as e:
        print(f"[WARN] Serial unavailable ({e}). Yaw will stay at 0.0 -- pivot-degree exits "
              f"will rely on timeouts only.")
        esp_ser = None

    # --- LiDAR ---
    try:
        lidar_scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        lidar_scanner.connect()
        threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,), daemon=True).start()
        print("[INFO] LiDAR connected.")
    except Exception as e:
        print(f"[FATAL] LiDAR required for this debug tool but failed to connect: {e}")
        sys.exit(1)

    # --- Rear ToF (optional) ---
    try:
        import board
        import busio
        import adafruit_vl53l0x
        i2c_tof = busio.I2C(board.SCL, board.SDA)
        tof_sensor = adafruit_vl53l0x.VL53L0X(i2c_tof)
        threading.Thread(target=tof_acquisition_thread_func, args=(tof_sensor,), daemon=True).start()
        print("[INFO] Rear ToF connected.")
    except Exception as e:
        print(f"[WARN] ToF unavailable ({e}). Backward/arc-clear-view exits use timeout only.")
        tof_sensor = None

    wall_follow_pid = PIDController(Kp=0.35, Ki=0.001, Kd=0.04, setpoint=0)
    alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)
    # NOTE: gyro_straight_pid has been removed. Straightness during the corner-approach
    # phase (and everywhere else that needs it) is judged against the tracked wall's
    # front/rear LiDAR sectors (get_wall_parallel_error / get_wall_parallel_sector_stats),
    # not against an absolute yaw=0 heading. This keeps the chassis parallel to the actual
    # wall geometry instead of an arbitrary gyro origin that can drift or be wrong if the
    # RST_YAW moment wasn't perfectly square to begin with.

    state = DebugState.WATCHING
    turn_direction = None
    use_reverse_arc = False
    locked_lane_reference_mm = None
    lane_number = None
    baseline_start_yaw = 0.0
    pivot_phase_start_time = 0.0
    post_pivot_align_start_time = 0.0
    post_pivot_align_no_wall_start_time = 0.0
    backward_phase_start_time = 0.0
    align_phase_start_time = 0.0
    align_no_wall_start_time = 0.0
    pre_align_start_time = 0.0
    pre_align_no_wall_start_time = 0.0
    corner_cooldown_end_time = 0.0
    frame_counter = 0
    corners_detected_count = 0

    print("[SYSTEM] Watching for corner signature... (Ctrl+C to stop)\n")

    try:
        while not global_shutdown_event.is_set():
            frame_counter += 1
            should_log = (frame_counter % STATE_LOG_EVERY_N_FRAMES == 0)

            # --- Read yaw from ESP32 if available ---
            if esp_ser:
                while esp_ser.in_waiting > 0:
                    try:
                        raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                        if raw_line.startswith("YAW:"):
                            current_yaw = float(raw_line.split(":")[1])
                    except Exception:
                        pass

            with lidar_data_lock:
                scan_data = latest_lidar_data.copy()

            # ==========================================================
            # SAFETY: if LiDAR hasn't produced a fresh scan recently, the
            # device has likely disconnected. Don't let the state machine
            # keep driving on stale/frozen data -- brake immediately and
            # skip this iteration's movement logic entirely.
            # ==========================================================
            lidar_age = time.monotonic() - last_lidar_update_time
            if last_lidar_update_time == 0.0 or lidar_age > LIDAR_STALE_TIMEOUT_SEC:
                if should_log:
                    print(f"[SAFETY] LiDAR data stale ({lidar_age:.2f}s old, timeout {LIDAR_STALE_TIMEOUT_SEC}s) "
                          f"-- braking, ignoring state machine this frame.")
                send_esp_packet(SERVO_CENTER_ANGLE, 0, "lidar stale brake")
                time.sleep(0.02)
                continue

            avg_front_baseline = get_compensated_front_distance(scan_data, current_yaw)
            left_pts = [scan_data[a] for a in range(-90, -39) if a in scan_data and scan_data[a] > 0]
            right_pts = [scan_data[a] for a in range(40, 91) if a in scan_data and scan_data[a] > 0]
            avg_left = sum(left_pts) / len(left_pts) if left_pts else 2000.0
            avg_right = sum(right_pts) / len(right_pts) if right_pts else 2000.0
            in_cooldown = time.monotonic() < corner_cooldown_end_time

            # ========================================================
            # STATE: WATCHING -- waiting for corner signature
            # ========================================================
            if state == DebugState.WATCHING:
                if should_log:
                    cooldown_note = f" | COOLDOWN {corner_cooldown_end_time - time.monotonic():.1f}s" if in_cooldown else ""
                    print(f"[WATCHING] Front:{avg_front_baseline:7.1f}mm  Left:{avg_left:7.1f}mm  "
                          f"Right:{avg_right:7.1f}mm  Yaw:{current_yaw:+6.1f}°{cooldown_note}")

                is_corner_signature = (
                    not in_cooldown
                    and avg_front_baseline <= 1000.0
                    and ((avg_left < 950.0 and avg_right > 1600.0)
                         or (avg_right < 900.0 and avg_left > 1800.0))
                )

                if is_corner_signature:
                    corners_detected_count += 1
                    print(f"\n{'*'*60}")
                    print(f"[CORNER SIGNATURE #{corners_detected_count}] Front:{avg_front_baseline:.1f}mm "
                          f"Left:{avg_left:.1f}mm Right:{avg_right:.1f}mm")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "signature brake")
                    time.sleep(CORNER_SIGNATURE_STOP_DELAY_SEC)

                    if avg_left < avg_right:
                        turn_direction = "RIGHT"
                    else:
                        turn_direction = "LEFT"
                    print(f"  -> Turn direction: {turn_direction}  (CW course: {CLOCKWISE_WALL_FOLLOWING})")

                    locked_lane_reference_mm = avg_left if CLOCKWISE_WALL_FOLLOWING else avg_right
                    tracked_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

                    if locked_lane_reference_mm > LANE_REFERENCE_ARC_THRESHOLD_MM:
                        lane_number = 1
                        use_reverse_arc = (turn_direction == "RIGHT")
                        if turn_direction != "RIGHT":
                            print(f"  -> Lane 1 on a LEFT turn: arc not tuned for this side -> falling back to PIVOT.")
                    else:
                        lane_number = "2/3"
                        use_reverse_arc = False

                    print(f"  -> Locked {tracked_side} wall distance: {locked_lane_reference_mm:.0f}mm "
                          f"(threshold {LANE_REFERENCE_ARC_THRESHOLD_MM:.0f}mm)")
                    print(f"  -> LANE = {lane_number}   ->   MANEUVER = {'ARC TURN' if use_reverse_arc else 'PIVOT TURN'}")
                    print(f"{'*'*60}\n")

                    send_reset_yaw()
                    time.sleep(0.1)
                    current_yaw = 0.0
                    baseline_start_yaw = 0.0
                    # Straighten against the tracked wall first, before starting the
                    # wall-following approach toward the 40cm second-stop trigger.
                    pre_align_start_time = time.monotonic()
                    pre_align_no_wall_start_time = 0.0
                    state = DebugState.CORNER_PRE_ALIGN
                    send_esp_packet(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED, "pre-align start")

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: CORNER_PRE_ALIGN -- straighten against wall BEFORE approach
            # ========================================================
            if state == DebugState.CORNER_PRE_ALIGN:
                align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                parallel_error = get_wall_parallel_error(scan_data, align_side)
                elapsed = time.monotonic() - pre_align_start_time

                if parallel_error is None:
                    if pre_align_no_wall_start_time == 0.0:
                        pre_align_no_wall_start_time = time.monotonic()
                    no_wall_elapsed = time.monotonic() - pre_align_no_wall_start_time
                else:
                    pre_align_no_wall_start_time = 0.0
                    no_wall_elapsed = 0.0

                is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
                no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                if should_log:
                    err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
                    print(f"[PRE-ALIGN] Side:{align_side} Front:{front_avg} Rear:{rear_avg} "
                          f"Err:{err_str} FrontPts:{front_count} RearPts:{rear_count} Elapsed:{elapsed:.2f}s")

                if is_aligned or hard_timeout or no_wall_timeout:
                    reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                    print(f"[PRE-ALIGN] Exit ({reason}). Straightened -> starting approach.")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "pre-align done brake")
                    time.sleep(CORNER_BRAKE_DELAY)
                    alignment_pid.reset()
                    state = DebugState.CORNER_APPROACH_WALL
                    send_esp_packet(SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED, "approach start")
                else:
                    if parallel_error is None:
                        send_esp_packet(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                    else:
                        normalized_error = parallel_error if align_side == "left" else -parallel_error
                        pid_output = alignment_pid.update(normalized_error)
                        target = SERVO_CENTER_ANGLE - pid_output
                        final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                        send_esp_packet(final_servo, WALL_ALIGN_CREEP_SPEED)

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: CORNER_APPROACH_WALL -- wall-follow to 40cm trigger
            #
            # Straightness fallback (when the tracked side wall is lost, or no side-wall
            # LiDAR points are available at all) uses the same front/rear parallel-wall
            # alignment method as CORNER_PRE_ALIGN / WALL_ALIGN_CORRECTION, driven by
            # alignment_pid, instead of a gyro-based approach.
            # ========================================================
            if state == DebugState.CORNER_APPROACH_WALL:
                align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"

                if avg_front_baseline < CORNER_APPROACH_TRIGGER_MM:
                    print(f"[APPROACH] Trigger reached: Front {avg_front_baseline:.1f}mm < "
                          f"{CORNER_APPROACH_TRIGGER_MM:.0f}mm. Braking...")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "approach brake")
                    time.sleep(CORNER_BRAKE_DELAY)
                    send_reset_yaw()
                    time.sleep(0.1)
                    pivot_phase_start_time = time.monotonic()
                    baseline_start_yaw = current_yaw

                    if use_reverse_arc:
                        arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                        print(f"[APPROACH] -> Entering ARC pivot (Lane 1). steer={arc_steer_angle}° reverse_speed={CORNER_ARC_PIVOT_SPEED}")
                        state = DebugState.CORNER_ARC_ACTIVE_PIVOT
                        send_esp_packet(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED, "arc pivot")
                    else:
                        final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                        print(f"[APPROACH] -> Entering IN-PLACE pivot (Lane {lane_number}). servo={final_servo}°")
                        send_esp_packet(final_servo, 0, "lock wheels")
                        time.sleep(0.15)
                        state = DebugState.CORNER_ACTIVE_PIVOT
                        send_esp_packet(final_servo, CORNER_PIVOT_SPEED, "pivot")
                else:
                    wall_pts = left_pts if CLOCKWISE_WALL_FOLLOWING else right_pts
                    parallel_error = get_wall_parallel_error(scan_data, align_side)

                    if wall_pts:
                        avg_wall = sum(wall_pts) / len(wall_pts)
                        if avg_wall > WALL_LOSS_THRESHOLD_MM:
                            # Side wall too far / effectively lost -> fall back to
                            # parallel-wall straightening instead of gyro-straight.
                            if parallel_error is None:
                                approach_servo = SERVO_CENTER_ANGLE
                                mode_note = "wall lost, no parallel data -> center"
                            else:
                                normalized_error = parallel_error if align_side == "left" else -parallel_error
                                pid_output = alignment_pid.update(normalized_error)
                                approach_servo = SERVO_CENTER_ANGLE - pid_output
                                mode_note = f"wall lost -> parallel straighten err={parallel_error:.0f}mm"
                        else:
                            wall_error = (avg_wall - WALL_FOLLOW_TARGET_MM) if CLOCKWISE_WALL_FOLLOWING else (WALL_FOLLOW_TARGET_MM - avg_wall)
                            pid_output = wall_follow_pid.update(wall_error)
                            approach_servo = SERVO_CENTER_ANGLE - pid_output
                            mode_note = f"wall follow, err={wall_error:.0f}mm"
                    else:
                        # No side-wall points at all -> parallel straighten if we have
                        # front/rear sector data, otherwise just hold center.
                        if parallel_error is None:
                            approach_servo = SERVO_CENTER_ANGLE
                            mode_note = "no wall data -> center"
                        else:
                            normalized_error = parallel_error if align_side == "left" else -parallel_error
                            pid_output = alignment_pid.update(normalized_error)
                            approach_servo = SERVO_CENTER_ANGLE - pid_output
                            mode_note = f"no side-wall pts -> parallel straighten err={parallel_error:.0f}mm"

                    final_approach_servo = int(round(np.clip(approach_servo, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                    if should_log:
                        print(f"[APPROACH] Front:{avg_front_baseline:7.1f}mm / {CORNER_APPROACH_TRIGGER_MM:.0f}mm "
                              f"| {mode_note} | servo={final_approach_servo}")
                    send_esp_packet(final_approach_servo, ROBOT_CRUISE_SPEED)

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: CORNER_ACTIVE_PIVOT -- in-place pivot
            # ========================================================
            if state == DebugState.CORNER_ACTIVE_PIVOT:
                elapsed = time.monotonic() - pivot_phase_start_time
                yaw_delta = current_yaw - baseline_start_yaw
                target_deg = TURN_TARGET_RIGHT_DEGREES if turn_direction == "RIGHT" else TURN_TARGET_LEFT_DEGREES
                pivot_complete = (yaw_delta <= -target_deg) if turn_direction == "RIGHT" else (yaw_delta >= target_deg)
                timed_out = elapsed >= CORNER_PIVOT_SAFETY_TIMEOUT

                if should_log:
                    print(f"[PIVOT] Yaw:{yaw_delta:+.1f}° / target {target_deg:.0f}° ({turn_direction}) | Elapsed:{elapsed:.2f}s")

                if pivot_complete or timed_out:
                    if timed_out and not pivot_complete:
                        print(f"[PIVOT] WARNING: timeout before target reached ({yaw_delta:+.1f}°).")
                    print(f"[PIVOT] Complete ({yaw_delta:+.1f}°). Braking...")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "pivot done brake")
                    time.sleep(CORNER_BRAKE_DELAY)
                    send_reset_yaw()
                    time.sleep(0.1)
                    current_yaw = 0.0

                    if lane_number == 1:
                        # Lane 1 -- still needs the backward-creep parallel align + final
                        # ToF backward phase (this only happens for a LEFT turn that fell
                        # back to in-place pivot despite being Lane 1 -- RIGHT turns on
                        # Lane 1 use the arc path instead, which also lands here).
                        print(f"[PIVOT] Lane 1 -> post-pivot align (backward creep).")
                        post_pivot_align_start_time = time.monotonic()
                        post_pivot_align_no_wall_start_time = 0.0
                        state = DebugState.CORNER_POST_PIVOT_ALIGN
                        send_esp_packet(SERVO_CENTER_ANGLE, 0)
                    else:
                        # Lane 2/3 -- NO backward movement at all. Skip straight past
                        # CORNER_POST_PIVOT_ALIGN (which also creeps backward) into the
                        # FORWARD parallel-align state, then resume driving.
                        print(f"[PIVOT] Lane {lane_number} -> skipping ALL backward movement, "
                              f"forward-aligning directly.")
                        corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
                        wall_follow_pid.reset()
                        alignment_pid.reset()
                        align_phase_start_time = time.monotonic()
                        align_no_wall_start_time = 0.0
                        state = DebugState.WALL_ALIGN_CORRECTION
                        turn_direction = None
                        locked_lane_reference_mm = None
                        lane_number = None
                        send_esp_packet(SERVO_CENTER_ANGLE, 0)
                else:
                    final_servo = SERVO_HARD_RIGHT if turn_direction == "RIGHT" else SERVO_HARD_LEFT
                    send_esp_packet(final_servo, CORNER_PIVOT_SPEED)

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: CORNER_ARC_ACTIVE_PIVOT -- reverse arc pivot
            # ========================================================
            if state == DebugState.CORNER_ARC_ACTIVE_PIVOT:
                elapsed = time.monotonic() - pivot_phase_start_time
                yaw_delta = current_yaw - baseline_start_yaw
                arc_complete = yaw_delta <= -TURN_TARGET_RIGHT_DEGREES
                timed_out = elapsed >= CORNER_ARC_PIVOT_SAFETY_TIMEOUT

                with tof_data_lock:
                    rear_distance = latest_tof_distance_mm
                rear_clear_view_hit = rear_distance is not None and rear_distance <= CORNER_ARC_CLEAR_VIEW_TOF_MM

                if should_log:
                    print(f"[ARC PIVOT] Yaw:{yaw_delta:+.1f}° / -{TURN_TARGET_RIGHT_DEGREES:.0f}° | "
                          f"Rear:{rear_distance} | Elapsed:{elapsed:.2f}s")

                if arc_complete or timed_out or rear_clear_view_hit:
                    if timed_out and not arc_complete:
                        print(f"[ARC PIVOT] WARNING: timeout before target reached ({yaw_delta:+.1f}°).")
                    if rear_clear_view_hit:
                        print(f"[ARC PIVOT] Rear ToF clear-view hit ({rear_distance:.0f}mm <= "
                              f"{CORNER_ARC_CLEAR_VIEW_TOF_MM:.0f}mm). Ending arc early.")
                    print(f"[ARC PIVOT] Complete ({yaw_delta:+.1f}°). Braking -> post-pivot align.")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "arc done brake")
                    time.sleep(CORNER_BRAKE_DELAY)
                    send_reset_yaw()
                    time.sleep(0.1)
                    current_yaw = 0.0
                    post_pivot_align_start_time = time.monotonic()
                    post_pivot_align_no_wall_start_time = 0.0
                    state = DebugState.CORNER_POST_PIVOT_ALIGN
                    send_esp_packet(SERVO_CENTER_ANGLE, 0)
                else:
                    arc_steer_angle = SERVO_CENTER_ANGLE - CORNER_ARC_STEER_OFFSET
                    send_esp_packet(arc_steer_angle, -CORNER_ARC_PIVOT_SPEED)

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: CORNER_POST_PIVOT_ALIGN -- LiDAR parallel alignment (reverse creep)
            #
            # NEW: only Lane 1 (arc turn) proceeds to CORNER_ALIGN_BACKWARD afterward.
            # A Lane 2/3 pivot turn already ends up close to the wall (no wide reverse-arc
            # swing to compensate for), so backing up further to chase the 190mm ToF target
            # just costs time for no positional benefit -- skip straight to the forward
            # WALL_ALIGN_CORRECTION handoff and resume driving.
            # ========================================================
            if state == DebugState.CORNER_POST_PIVOT_ALIGN:
                align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                front_avg, rear_avg, front_count, rear_count = get_wall_parallel_sector_stats(scan_data, align_side)
                parallel_error = get_wall_parallel_error(scan_data, align_side)
                elapsed = time.monotonic() - post_pivot_align_start_time

                if parallel_error is None:
                    if post_pivot_align_no_wall_start_time == 0.0:
                        post_pivot_align_no_wall_start_time = time.monotonic()
                    no_wall_elapsed = time.monotonic() - post_pivot_align_no_wall_start_time
                else:
                    post_pivot_align_no_wall_start_time = 0.0
                    no_wall_elapsed = 0.0

                is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
                no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                if should_log:
                    err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
                    print(f"[POST-PIVOT ALIGN] Side:{align_side} Front:{front_avg} Rear:{rear_avg} "
                          f"Err:{err_str} FrontPts:{front_count} RearPts:{rear_count} Elapsed:{elapsed:.2f}s")

                if is_aligned or hard_timeout or no_wall_timeout:
                    reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                    # This state is only ever reached by Lane 1 now (Lane 2/3 skips it
                    # entirely from the pivot-complete dispatch above), so it always
                    # proceeds to the backward ToF-docking phase.
                    print(f"[POST-PIVOT ALIGN] Exit ({reason}). Lane 1 -> backward phase.")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "align done brake")
                    time.sleep(CORNER_BRAKE_DELAY)
                    alignment_pid.reset()
                    backward_phase_start_time = time.monotonic()
                    state = DebugState.CORNER_ALIGN_BACKWARD
                    send_esp_packet(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED, "backward start")
                else:
                    if parallel_error is None:
                        send_esp_packet(SERVO_CENTER_ANGLE, -WALL_ALIGN_CREEP_SPEED)
                    else:
                        normalized_error = parallel_error if align_side == "left" else -parallel_error
                        pid_output = -alignment_pid.update(normalized_error)
                        target = SERVO_CENTER_ANGLE - pid_output
                        final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                        send_esp_packet(final_servo, -WALL_ALIGN_CREEP_SPEED)

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: CORNER_ALIGN_BACKWARD -- back off to final ToF distance
            # (Lane 1 / arc-turn only -- see CORNER_POST_PIVOT_ALIGN dispatch above)
            # ========================================================
            if state == DebugState.CORNER_ALIGN_BACKWARD:
                elapsed = time.monotonic() - backward_phase_start_time
                with tof_data_lock:
                    rear_distance = latest_tof_distance_mm
                tof_reached = rear_distance is not None and rear_distance <= (CORNER_BACKWARD_TOF_TARGET_MM + CORNER_BACKWARD_TOF_TOLERANCE_MM)
                hard_timeout = elapsed >= CORNER_BACKWARD_DURATION

                if should_log:
                    print(f"[BACKWARD] Rear:{rear_distance} / {CORNER_BACKWARD_TOF_TARGET_MM:.0f}mm Elapsed:{elapsed:.2f}s")

                if tof_reached or hard_timeout:
                    if hard_timeout and not tof_reached:
                        print(f"[BACKWARD] WARNING: timeout before ToF target reached (rear={rear_distance}).")
                    else:
                        print(f"[BACKWARD] ToF target reached (rear={rear_distance:.0f}mm).")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "backward done brake")
                    time.sleep(0.3)
                    send_reset_yaw()
                    time.sleep(0.1)
                    current_yaw = 0.0
                    corner_cooldown_end_time = time.monotonic() + CORNER_DETECTION_COOLDOWN_SEC
                    wall_follow_pid.reset()
                    alignment_pid.reset()
                    align_phase_start_time = time.monotonic()
                    align_no_wall_start_time = 0.0
                    state = DebugState.WALL_ALIGN_CORRECTION
                    turn_direction = None
                    locked_lane_reference_mm = None
                    lane_number = None
                else:
                    send_esp_packet(SERVO_CENTER_ANGLE, -ROBOT_MANEUVER_SPEED)

                time.sleep(0.02)
                continue

            # ========================================================
            # STATE: WALL_ALIGN_CORRECTION -- forward parallel re-align, then back to WATCHING
            # ========================================================
            if state == DebugState.WALL_ALIGN_CORRECTION:
                align_side = "left" if CLOCKWISE_WALL_FOLLOWING else "right"
                parallel_error = get_wall_parallel_error(scan_data, align_side)
                elapsed = time.monotonic() - align_phase_start_time

                if parallel_error is None:
                    if align_no_wall_start_time == 0.0:
                        align_no_wall_start_time = time.monotonic()
                    no_wall_elapsed = time.monotonic() - align_no_wall_start_time
                else:
                    align_no_wall_start_time = 0.0
                    no_wall_elapsed = 0.0

                is_aligned = parallel_error is not None and abs(parallel_error) < PARALLEL_TOLERANCE_MM
                hard_timeout = elapsed >= WALL_ALIGN_SAFETY_TIMEOUT
                no_wall_timeout = parallel_error is None and no_wall_elapsed >= WALL_ALIGN_NO_WALL_TIMEOUT

                if should_log:
                    err_str = f"{parallel_error:.1f}mm" if parallel_error is not None else "N/A"
                    print(f"[WALL ALIGN] Side:{align_side} Err:{err_str} Elapsed:{elapsed:.2f}s")

                if is_aligned or hard_timeout or no_wall_timeout:
                    reason = "aligned" if is_aligned else "timeout" if hard_timeout else "no_wall"
                    print(f"[WALL ALIGN] Exit ({reason}). Corner maneuver COMPLETE. -> back to WATCHING.\n")
                    send_esp_packet(SERVO_CENTER_ANGLE, 0, "resume watching")
                    alignment_pid.reset()
                    state = DebugState.WATCHING
                    continue

                if parallel_error is None:
                    send_esp_packet(SERVO_CENTER_ANGLE, WALL_ALIGN_CREEP_SPEED)
                else:
                    normalized_error = parallel_error if align_side == "left" else -parallel_error
                    pid_output = alignment_pid.update(normalized_error)
                    target = SERVO_CENTER_ANGLE - pid_output
                    final_servo = int(round(np.clip(target, SERVO_CENTER_ANGLE - 20, SERVO_CENTER_ANGLE + 20)))
                    send_esp_packet(final_servo, WALL_ALIGN_CREEP_SPEED)

                time.sleep(0.02)
                continue

    except Exception as e:
        print(f"[SYSTEM FAILURE] {e}")
    finally:
        emergency_shutdown_handler(None, None)


if __name__ == '__main__':
    corner_debug_loop()