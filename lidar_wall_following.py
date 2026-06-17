import serial
import math
import time
import ydlidar
import sys
import threading

# --- TUNED RUNTIME PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# Your Exact Symmetrical Steering Mappings
STEERING_CENTER = 90        
MAX_LEFT_TURN = 75          # Keeps tracking smooth
MAX_RIGHT_TURN = 105        # Keeps tracking smooth

TARGET_DISTANCE_MM = 500.0  # Desired distance to maintain from the left wall
BASE_SPEED = 85            # Controlled test speed for path adjustments

# Proportional control tuning coefficients
KP_GYRO = 0.35
KP_LIDAR = 0.06

current_yaw = 0.0
serial_lock = threading.Lock()

# --- Native YDLIDAR Driver Block ---
class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.laser = None
        self.scan_data = {}
        self.MIN_ANGLE = -180.0
        self.MAX_ANGLE = 180.0
        self.MIN_RANGE = 0.02
        self.MAX_RANGE = 16.0

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        try:
            ydlidar.os_init()
            self.laser = ydlidar.CYdLidar()
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.MAX_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.MIN_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.MAX_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.MIN_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
            self.laser.initialize()
            self.laser.turnOn()
            print(f"[LIDAR] Driver initialized successfully on {self.port}")
        except Exception as e:
            print(f"[LIDAR ERROR] Native initialization failure: {e}")
            raise IOError()

    def disconnect(self):
        if self.laser:
            self.laser.turnOff()
            self.laser.disconnecting()

    def get_scan_data(self):
        if not self.laser: return None
        self.scan_data = {}
        scan = ydlidar.LaserScan()
        if self.laser.doProcessSimple(scan):
            for p in scan.points:
                if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                    angle_deg = round(math.degrees(p.angle))
                    self.scan_data[angle_deg] = p.range * 1000.0
            return self.scan_data
        return None

# --- Background Telemetry Thread Worker ---
def handle_esp_telemetry(ser_conn):
    global current_yaw
    while ser_conn.is_open:
        try:
            if ser_conn.in_waiting > 0:
                line = ser_conn.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("YAW:"):
                    with serial_lock:
                        current_yaw = float(line.split(":")[1])
        except Exception:
            break

def main():
    global current_yaw
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=1)
        print("[INFO] High-Speed UART interface link online with ESP32.")
    except Exception as e:
        print(f"[FATAL ERROR] Cannot access UART port {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # Spawn background worker thread
    telemetry_thread = threading.Thread(target=handle_esp_telemetry, args=(esp_ser,), daemon=True)
    telemetry_thread.start()

    try:
        with LidarScanner(port='/dev/ttyUSB0', baudrate=230400) as scanner:
            time.sleep(1.5) # System grace period for laser stabilization
            
            while True:
                scan_records = scanner.get_scan_data()
                if not scan_records:
                    continue
                
                # Verified Left-Wall Sensor Corridor Mapping (-90 to -40)
                left_corridor = [
                    scan_records[deg] for deg in range(-90, -40)
                    if deg in scan_records and 150.0 < scan_records[deg] < 2000.0
                ]
                
                if len(left_corridor) > 3:
                    avg_left_dist = sum(left_corridor) / len(left_corridor)
                    distance_error = avg_left_dist - TARGET_DISTANCE_MM
                    
                    with serial_lock:
                        yaw_snapshot = current_yaw
                    
                    # --- NATIVE HARDWARE FUSED MATH ---
                    # Negative distance error (too close left) -> Adds to 90 -> Turns Right (>90)
                    # Negative gyro yaw (drifting right) -> Subtracts from 90 -> Turns Left (<90)
                    steering_output = STEERING_CENTER + (KP_GYRO * yaw_snapshot) - (KP_LIDAR * distance_error)
                    
                    # Hard clamping limits using your calibrated 75° - 105° smooth bounds
                    final_steering = max(MAX_LEFT_TURN, min(MAX_RIGHT_TURN, int(round(steering_output))))
                    
                    # Ship command packet to execution tier
                    command_string = f"STR:{final_steering},SPD:{BASE_SPEED}\n"
                    esp_ser.write(command_string.encode('utf-8'))
                    
                    print(f"[RUNNING] Distance: {avg_left_dist:4.0f}mm | Error: {distance_error:4.0f}mm | Filtered Yaw: {yaw_snapshot:5.1f}° -> Steering: {final_steering}°")
                else:
                    # Target lost safety recovery fallback: hold straight line
                    command_string = f"STR:{STEERING_CENTER},SPD:{BASE_SPEED}\n"
                    esp_ser.write(command_string.encode('utf-8'))
                    print("[WARNING] Sensed left wall envelope returned empty. Driving straight...")
                    
                time.sleep(0.05) # Fixed 20Hz refresh frequency clock

    except KeyboardInterrupt:
        print("\n[INFO] Manual termination sequence triggered.")
    finally:
        # Final emergency stop handshake sequence
        esp_ser.write(f"STR:{STEERING_CENTER},SPD:0\n".encode('utf-8'))
        esp_ser.close()
        print("[SUCCESS] Safety shutdown complete. System isolated.")

if __name__ == "__main__":
    main()