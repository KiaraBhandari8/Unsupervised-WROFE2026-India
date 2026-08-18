"""
Nav process — the priority state machine, same tiers as the original
robot_control_loop(), minus the camera-owning and LiDAR-owning parts
(those moved to their own processes) and minus the Flask/overlay
streaming (that's coming back once the vision process exists — for now
this process just prints state to the console).

Tiers implemented: 0 (lap termination), 1 (corner), 3 (side avoidance),
4 (vision obstacle avoidance), 5 (wall-follow default). Tier 4 reads the
VisionResult published by vision_process.py — servo_adjust is already
gain-scaled there, so this file just applies it, same shape as reading
LidarResult from lidar_process.py.

IMPORTANT ORDERING NOTE: corner_routine_execution_v2 registers its own
SIGINT handler at import time (module-level, not inside `if __name__`).
The import below MUST happen before this file's own signal.signal() call
in main.py sets the real handler, exactly like the original script did —
otherwise Ctrl+C will silently call the wrong (uninitialized) handler
instead of actually stopping the ESP32. main.py handles this; don't
reorder the imports there.
"""

import sys
import time

import serial

from config import (
    RobotState,
    PI_TO_ESP_PORT, BAUD_RATE_ESP,
    SERVO_CENTER_ANGLE, SERVO_ANGLE_LIMIT,
    ROBOT_CRUISE_SPEED, ROBOT_MANEUVER_SPEED,
    CORNER_COOLDOWN_SEC,
    SIDE_STEER_MIN_MAGNITUDE, SIDE_STEER_MAX_MAGNITUDE, SIDE_TRIGGER_DISTANCE_MM,
    TRIGGER_CONFIRM_FRAMES, RELEASE_CONFIRM_FRAMES,
    PID_OUTPUT_CLAMP, WALL_FOLLOW_SIDE,
    LOG_EVERY_N_LOOPS, VERBOSE_LOGGING,
    NAV_LOOP_SLEEP_S, LIDAR_STALE_TIMEOUT_S, VISION_STALE_TIMEOUT_S,
)
from lidar_steering_new_parallel import PIDController
from corner_routine_execution_v2 import execute_cornering, init_ultrasonic
from profiling import profile_section, record_state_fps, print_profile_summary
from shared_state import SharedRobotState, SharedScanReader

import numpy as np


def send_esp_packet(ser_port, steering, speed, shutdown_event):
    if ser_port and ser_port.is_open and not shutdown_event.is_set():
        try:
            packet = f"STR:{steering},SPD:{speed}\n"
            ser_port.write(packet.encode('utf-8'))
        except Exception:
            pass


def compute_side_avoidance_magnitude(min_dist_mm):
    if min_dist_mm is None or min_dist_mm < 0:
        return SIDE_STEER_MIN_MAGNITUDE
    proximity_frac = min(1.0, max(0.0, (SIDE_TRIGGER_DISTANCE_MM - min_dist_mm) / SIDE_TRIGGER_DISTANCE_MM))
    return SIDE_STEER_MIN_MAGNITUDE + proximity_frac * (SIDE_STEER_MAX_MAGNITUDE - SIDE_STEER_MIN_MAGNITUDE)


def run_nav_process(shared: SharedRobotState):
    print("[NAV PROC] Starting up.")

    # --- Serial to ESP32 (owned exclusively by this process) ---
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print("[NAV PROC] Serial connection established with ESP32.")
    except Exception as e:
        print(f"[NAV PROC][FATAL] Serial init failed on {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # init_ultrasonic() must run inside this process (GPIO setup), not at
    # module import time before the process forks/spawns.
    init_ultrasonic()

    scan_reader = SharedScanReader(shared)

    gyro_straight_pid = PIDController(Kp=2.2, Ki=0.002, Kd=0.15, setpoint=0)
    alignment_pid = PIDController(Kp=0.22, Ki=0.0, Kd=0.08, setpoint=0)

    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
    current_yaw = 0.0
    turn_count = 0
    corner_cooldown_end_time = 0.0

    # --- Lap / timing tracking ---
    lap_start_time = time.monotonic()
    side_start_time = time.monotonic()
    corner_start_time = time.monotonic()
    lap_times = []
    side_times = []
    corner_times = []
    total_laps = 3
    total_sides = 12

    right_side_engaged = False
    left_side_engaged = False
    right_confirm_streak = 0
    left_confirm_streak = 0
    right_clear_streak = 0
    left_clear_streak = 0

    loop_iteration = 0
    prev_loop_start_time = time.monotonic()

    print(f"[NAV PROC] Calibration complete. Initial state: {current_robot_state} | "
          f"wall-follow side: {WALL_FOLLOW_SIDE}")

    try:
        while not shared.shutdown_event.is_set():
            loop_start_time = time.monotonic()
            loop_duration = loop_start_time - prev_loop_start_time
            fps = 1.0 / loop_duration if loop_duration > 0 else 0.0
            prev_loop_start_time = loop_start_time
            record_state_fps(current_robot_state, loop_duration)

            loop_iteration += 1
            should_log = (loop_iteration % LOG_EVERY_N_LOOPS == 0)
            if should_log:
                print_profile_summary(tag="nav")

            with profile_section("nav.serial_read_yaw"):
                while esp_ser.in_waiting > 0:
                    try:
                        raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                        if raw_line.startswith("YAW:"):
                            current_yaw = float(raw_line.split(":")[1])
                    except Exception:
                        pass

            with profile_section("nav.read_lidar_result"):
                with shared.lidar_result.get_lock():
                    r = shared.lidar_result
                    lidar_age_s = time.monotonic() - r.timestamp
                    avg_front_dist = r.avg_front_dist
                    corner_flag = r.corner_flag
                    corner_distance_mm = r.corner_distance_mm
                    wall_error_valid = r.wall_error_valid
                    wall_combined_error = r.wall_combined_error
                    right_min_dist = r.right_min_dist
                    left_min_dist = r.left_min_dist
                    right_raw_trigger = r.right_raw_trigger
                    left_raw_trigger = r.left_raw_trigger

            lidar_is_stale = lidar_age_s > LIDAR_STALE_TIMEOUT_S
            if lidar_is_stale and VERBOSE_LOGGING:
                print(f"[NAV PROC][WARN] LiDAR data is stale ({lidar_age_s:.2f}s old)")

            with profile_section("nav.read_vision_result"):
                with shared.vision_result.get_lock():
                    v = shared.vision_result
                    vision_age_s = time.monotonic() - v.timestamp
                    vision_valid = v.valid
                    vision_label = v.obstacle_label.decode('utf-8', errors='ignore')
                    vision_servo_adjust = v.servo_adjust

            vision_is_stale = (not vision_valid) or (vision_age_s > VISION_STALE_TIMEOUT_S)
            if vision_is_stale and VERBOSE_LOGGING:
                print(f"[NAV PROC][WARN] Vision data is stale/unavailable ({vision_age_s:.2f}s old)")

            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # ====================================================
            # PRIORITY 0: LAP TERMINATION
            # ====================================================
            if current_robot_state == RobotState.LAP_TERMINATION or turn_count >= 12:
                current_robot_state = RobotState.LAP_TERMINATION
                total_elapsed = time.monotonic() - lap_start_time
                print("\n[MATCH COMPLETE] 12 race turns logged! Finishing.")
                print(f"[TIMING SUMMARY] Total time: {total_elapsed:.2f}s")
                if lap_times:
                    print(f"  Lap times: {[f'{t:.2f}s' for t in lap_times]}")
                if side_times:
                    print(f"  Side times: {[f'{t:.2f}s' for t in side_times]}")
                if corner_times:
                    print(f"  Corner times: {[f'{t:.2f}s' for t in corner_times]}")
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, ROBOT_CRUISE_SPEED, shared.shutdown_event)
                time.sleep(4.0)
                send_esp_packet(esp_ser, SERVO_CENTER_ANGLE, 0, shared.shutdown_event)
                print("[NAV PROC] Hard race shutdown executed.")
                break

            # ====================================================
            # PRIORITY 1: CORNER TURNING OVERRIDE
            # ====================================================
            current_time = time.monotonic()
            in_corner_cooldown = current_time < corner_cooldown_end_time
            cooldown_remaining = max(0.0, corner_cooldown_end_time - current_time)

            if corner_flag and not lidar_is_stale:
                if in_corner_cooldown:
                    if VERBOSE_LOGGING:
                        print(f"[CORNER BLOCKED] cooldown, {cooldown_remaining:.2f}s left")
                else:
                    print("[NAV PROC] Executing corner maneuver...")
                    previous_state = current_robot_state
                    current_robot_state = RobotState.CORNER_MANEUVER

                    corner_start_time = time.monotonic()
                    with profile_section("nav.execute_cornering"):
                        # scan_reader stands in for the live LidarScanner —
                        # execute_cornering calls .get_scan_data() on it
                        # repeatedly throughout the maneuver, same as before.
                        execute_cornering(esp_ser, None, scan_reader)

                    corner_elapsed = time.monotonic() - corner_start_time
                    corner_times.append(corner_elapsed)
                    side_elapsed = time.monotonic() - side_start_time
                    side_times.append(side_elapsed)
                    side_start_time = time.monotonic()

                    turn_count += 1
                    corner_cooldown_end_time = time.monotonic() + CORNER_COOLDOWN_SEC
                    current_robot_state = previous_state

                    # Check for lap completion (4 corners = 1 lap)
                    if turn_count % 4 == 0:
                        lap_elapsed = time.monotonic() - lap_start_time
                        lap_times.append(lap_elapsed)
                        lap_start_time = time.monotonic()
                        lap_num = turn_count // 4
                        print(f"[LAP {lap_num}/{total_laps}] Completed in {lap_elapsed:.2f}s")

                    print(f"[CORNER {turn_count}/{total_sides}] Time: {corner_elapsed:.2f}s | "
                          f"Side: {side_elapsed:.2f}s | "
                          f"Lap: {turn_count // 4}/{total_laps}")
                    continue

            # ====================================================
            # PRIORITY 3: PROXIMITY CRITICAL WALL OVERRIDES
            # ====================================================
            right_confirm_streak = right_confirm_streak + 1 if right_raw_trigger else 0
            left_confirm_streak = left_confirm_streak + 1 if left_raw_trigger else 0

            if not right_side_engaged and right_confirm_streak >= TRIGGER_CONFIRM_FRAMES:
                right_side_engaged = True
                right_clear_streak = 0
            if not left_side_engaged and left_confirm_streak >= TRIGGER_CONFIRM_FRAMES:
                left_side_engaged = True
                left_clear_streak = 0

            if right_side_engaged:
                right_clear_streak = right_clear_streak + 1 if not right_raw_trigger else 0
                if right_clear_streak >= RELEASE_CONFIRM_FRAMES:
                    right_side_engaged = False
                    right_confirm_streak = 0
            if left_side_engaged:
                left_clear_streak = left_clear_streak + 1 if not left_raw_trigger else 0
                if left_clear_streak >= RELEASE_CONFIRM_FRAMES:
                    left_side_engaged = False
                    left_confirm_streak = 0

            right_offset = compute_side_avoidance_magnitude(right_min_dist) if right_side_engaged else 0.0
            left_offset = compute_side_avoidance_magnitude(left_min_dist) if left_side_engaged else 0.0

            if right_side_engaged or left_side_engaged:
                current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                robot_speed_current = ROBOT_MANEUVER_SPEED

                if right_offset >= left_offset:
                    target_servo_angle = SERVO_CENTER_ANGLE - right_offset
                    display_text = f"MODE: Side Avoid (Right) | Steer: {int(target_servo_angle)}"
                else:
                    target_servo_angle = SERVO_CENTER_ANGLE + left_offset
                    display_text = f"MODE: Side Avoid (Left) | Steer: {int(target_servo_angle)}"

            # ====================================================
            # PRIORITY 4: VISION OBSTACLE AVOIDANCE
            # servo_adjust is already gain-scaled (STEERING_GAIN_RED/GREEN,
            # RED_CLEARANCE_OFFSET) inside vision_process.py — this tier
            # just applies it, same shape as the LiDAR PID output below.
            # ====================================================
            else:
                if (not vision_is_stale) and vision_label in ("red_obstacle", "obstacle"):
                    current_robot_state = RobotState.VISION_OBSTACLE_AVOIDANCE
                    robot_speed_current = ROBOT_MANEUVER_SPEED
                    target_servo_angle = SERVO_CENTER_ANGLE - vision_servo_adjust
                    label_str = "Red Avoid" if vision_label == "red_obstacle" else "Green Avoid"
                    display_text = f"MODE: {label_str} | Steer: {int(target_servo_angle)}"

                # ====================================================
                # PRIORITY 5: WALL-FOLLOW DEFAULT
                # ====================================================
                else:
                    robot_speed_current = ROBOT_CRUISE_SPEED
                    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING

                    with profile_section("nav.wall_follow_pid"):
                        if not wall_error_valid or lidar_is_stale:
                            # No need for this just print and pass
                            # heading_error = 0.0 - current_yaw
                            # pid_output = gyro_straight_pid.update(heading_error)
                            # target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            display_text = f"MODE: Wall Lost Fallback (Gyro Straight)"
                        else:
                            normalized_error = wall_combined_error if WALL_FOLLOW_SIDE == "left" else -wall_combined_error
                            pid_output = alignment_pid.update(normalized_error)
                            pid_output = max(-PID_OUTPUT_CLAMP, min(PID_OUTPUT_CLAMP, pid_output))
                            target_servo_angle = SERVO_CENTER_ANGLE - pid_output
                            display_text = f"MODE: Wall Follow ({WALL_FOLLOW_SIDE}) | Err: {wall_combined_error:+.0f}mm"

            # ====================================================
            # ACTUATE
            # ====================================================
            with profile_section("nav.send_packet"):
                final_servo_angle = int(round(np.clip(
                    target_servo_angle,
                    SERVO_CENTER_ANGLE - SERVO_ANGLE_LIMIT,
                    SERVO_CENTER_ANGLE + SERVO_ANGLE_LIMIT,
                )))
                send_esp_packet(esp_ser, final_servo_angle, robot_speed_current, shared.shutdown_event)

            if should_log and VERBOSE_LOGGING:
                elapsed_total = time.monotonic() - lap_start_time
                current_side = turn_count % 4 if turn_count % 4 != 0 else 4
                current_lap = turn_count // 4 + 1
                # print(f"[{current_robot_state}] {display_text} | fps={fps:.1f}")
                print(f"  [TIMING] Side {current_side}/{total_sides} | "
                      f"Lap {current_lap}/{total_laps} | "
                      f"Elapsed: {elapsed_total:.2f}s | "
                      f"Sides done: {turn_count}/{total_sides}")

            time.sleep(NAV_LOOP_SLEEP_S)

    except Exception as e:
        print(f"[NAV PROC][FAILURE] {e}")
    finally:
        print("[NAV PROC] Shutting down, sending stop command.")
        try:
            for _ in range(3):
                esp_ser.write(f"STR:{SERVO_CENTER_ANGLE},SPD:0\n".encode('utf-8'))
                esp_ser.flush()
                time.sleep(0.03)
            esp_ser.close()
        except Exception:
            pass
