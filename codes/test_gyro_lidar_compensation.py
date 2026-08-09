import time
import sys
import threading
import numpy as np
import serial
import signal

# Import your existing working LiDAR scanner class
try:
    from lidar_steering4sept import LidarScanner
except ImportError as e:
    print(f"[ERROR] Cannot find lidar_steering4sept.py: {e}")
    sys.exit(1)

# --- CONFIGURATION CONSTANTS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200
FRONT_SCAN_ANGLE_DEG = 15  # Size of the scanning cone (+/- 15 degrees)

# --- GLOBAL VARIABLES ---
current_yaw = 0.0
latest_lidar_data = {}
lidar_data_lock = threading.Lock()
shutdown_event = threading.Event()

esp_ser = None
lidar_scanner = None

# --- GRACEFUL SHUTDOWN HANDLER ---
def sigint_handler(signum, frame):
    print("\n[SHUTDOWN] Exiting cleanly and releasing hardware ports...")
    shutdown_event.set()
    
    if esp_ser and esp_ser.is_open:
        esp_ser.close()
    if lidar_scanner:
        lidar_scanner.disconnect()
        
    print("[SUCCESS] All resources released.")
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

# --- BACKGROUND LIDAR INGESTION THREAD ---
def lidar_thread_func(scanner):
    global latest_lidar_data
    while not shutdown_event.is_set():
        data = scanner.get_scan_data()
        if data:
            with lidar_data_lock:
                latest_lidar_data = data.copy()
        time.sleep(0.01)

# --- MAIN DIAGNOSTIC LOOP ---
def main():
    global current_yaw, esp_ser, lidar_scanner

    print("--- Starting Gyro-LiDAR Compensation Diagnostic Test ---")
    
    # 1. Connect to ESP32 for Gyro Data
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.05)
        print(f"[CONNECTED] ESP32 Serial Link online on {PI_TO_ESP_PORT}")
    except Exception as e:
        print(f"[FATAL ERROR] Failed to connect to ESP32: {e}")
        sys.exit(1)

    # 2. Connect to YDLIDAR Tmini Plus
    try:
        lidar_scanner = LidarScanner(port='/dev/ttyUSB0', baudrate=230400)
        lidar_scanner.connect()
        
        # Start background lidar thread
        lidar_thread = threading.Thread(target=lidar_thread_func, args=(lidar_scanner,))
        lidar_thread.daemon = True
        lidar_thread.start()
        print("[CONNECTED] YDLIDAR Tmini Plus online on /dev/ttyUSB0")
    except Exception as e:
        print(f"[FATAL ERROR] Failed to connect to LiDAR: {e}")
        if esp_ser:
            esp_ser.close()
        sys.exit(1)

    print("\nInitializing streams... Twisting the robot will show data adjustments.")
    print("-" * 80)
    print(f"{'YAW ANGLE':^12} | {'CONE RANGE':^15} | {'RAW FRONT':^15} | {'COMPENSATED FRONT':^15}")
    print("-" * 80)

    try:
        while not shutdown_event.is_set():
            # Read Yaw heading from ESP32
            while esp_ser.in_waiting > 0:
                try:
                    raw_line = esp_ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line.startswith("YAW:"):
                        current_yaw = float(raw_line.split(":")[1])
                except Exception:
                    pass

            # Capture local snapshot of current lidar environment
            scan_data = {}
            with lidar_data_lock:
                if latest_lidar_data:
                    scan_data = latest_lidar_data.copy()

            if scan_data:
                # --- CALCULATE UNCOMPENSATED (RAW) DISTANCE ---
                # Looks strictly at local 0 degrees +/- FRONT_SCAN_ANGLE_DEG
                raw_angles = range(-FRONT_SCAN_ANGLE_DEG, FRONT_SCAN_ANGLE_DEG + 1)
                raw_points = [scan_data[a] for a in raw_angles if a in scan_data and scan_data[a] > 0]
                avg_front_raw = sum(raw_points) / len(raw_points) if raw_points else 0.0

                # --- CALCULATE GYRO-COMPENSATED DISTANCE ---
                # 1. Shift look-window center index to match yaw drift
                yaw_offset = int(round(current_yaw))
                dynamic_angles = range(-FRONT_SCAN_ANGLE_DEG + yaw_offset, FRONT_SCAN_ANGLE_DEG + yaw_offset + 1)

                # 2. Extract points and project perpendicular distance using cos(yaw)
                compensated_points = []
                yaw_rad = np.radians(current_yaw)
                
                for a in dynamic_angles:
                    if a in scan_data and scan_data[a] > 0:
                        raw_distance = scan_data[a]
                        # Perpendicular projection formula
                        true_distance = raw_distance * np.cos(yaw_rad)
                        compensated_points.append(true_distance)

                avg_front_compensated = sum(compensated_points) / len(compensated_points) if compensated_points else 0.0

                # --- PRINT TELEMETRY MATRIX ---
                cone_str = f"[{min(dynamic_angles)} to {max(dynamic_angles)}]"
                print(f"{current_yaw:^+11.2f}° | {cone_str:^15} | {avg_front_raw:^13.1f} mm | {avg_front_compensated:^13.1f} mm", end='\r')
                sys.stdout.flush()

            time.sleep(0.05)  # Throttles terminal outputs to ~20Hz updates

    except KeyboardInterrupt:
        sigint_handler(None, None)

if __name__ == '__main__':
    main()