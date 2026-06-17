import serial
import math
import time
import ydlidar
import sys
import threading

# --- TUNED RUNTIME PARAMETERS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

STEERING_CENTER = 90       
MAX_LEFT_TURN = 60          # 60 degrees = Physical Hard Left Lock
MAX_RIGHT_TURN = 120        # 120 degrees = Physical Hard Right Lock

TARGET_DISTANCE_MM = 300.0  # Maintained distance to outer wall
FRONT_TRIGGER_MM = 400.0    # 40cm front wall detection threshold
BASE_SPEED = 90             

KP_GYRO = 0.35
KP_LIDAR = 0.08

# State Machine Definitions
STATE_WALL_FOLLOW = 0
STATE_TURNING = 1
STATE_RECOVERY_STRAIGHT = 2
STATE_NAMES = {0: "WALL_FOLLOW", 1: "GYRO_TURNING", 2: "INTERSECTION_RECOVERY"}

current_state = STATE_WALL_FOLLOW
turn_direction = None

current_yaw = 0.0
last_sent_steering = 90
serial_lock = threading.Lock()

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
            print(f"[LIDAR] Driver running smoothly on {self.port}")
        except Exception as e:
            print(f"[LIDAL ERROR] Initialization failure: {e}")
            raise IOError()

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

    def disconnect(self):
        if self.laser:
            self.laser.turnOff()
            self.laser.disconnecting()

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

# Monitored outbound write wrapper
def send_steering_command(ser_conn, angle, speed):
    global last_sent_steering
    last_sent_steering = max(60, min(120, int(round(angle))))
    packet = f"STR:{last_sent_steering},SPD:{speed}\n"
    ser_conn.write(packet.encode('utf-8'))

def main():
    global current_state, turn_direction, current_yaw, last_sent_steering
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=1)
        print("[INFO] High-Speed UART communication active with ESP32.")
    except Exception as e:
        print(f"[FATAL] Cannot establish connection to port {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    telemetry_thread = threading.Thread(target=handle_esp_telemetry, args=(esp_ser,), daemon=True)
    telemetry_thread.start()

    try:
        with LidarScanner(port='/dev/ttyUSB0', baudrate=230400) as scanner:
            time.sleep(2.0) 
            
            while True:
                scan_records = scanner.get_scan_data()
                if not scan_records:
                    continue
                
                with serial_lock:
                    yaw_snapshot = current_yaw

                # 1. Gather raw pinpoint metrics for debugging dashboard
                raw_left_90 = scan_records.get(-90, 0.0)
                raw_right_90 = scan_records.get(90, 0.0)

                # 2. Extract forward array segment (-10 to +10 degrees)
                forward_corridor = [
                    scan_records[deg] for deg in range(-10, 11)
                    if deg in scan_records and scan_records[deg] > 50.0
                ]
                avg_front_dist = sum(forward_corridor) / len(forward_corridor) if len(forward_corridor) > 0 else 2000.0

                # ====================================================
                # STATE 0: OUTER WALL RECTILINEAR FOLLOWER
                # ====================================================
                if current_state == STATE_WALL_FOLLOW:
                    if avg_front_dist < FRONT_TRIGGER_MM:
                        print(f"\n[ALERT] Front obstacle detected at {avg_front_dist:.0f}mm. Evaluating path configurations...")
                        
                        left_check = scan_records.get(-45, 0.0)
                        right_check = scan_records.get(45, 0.0)
                        
                        current_state = STATE_TURNING
                        
                        if left_check > 1200.0 or left_check == 0.0:
                            turn_direction = "CCW"
                            print("[DECISION] Left vector clear. Launching Left Turn Profile.")
                        else:
                            turn_direction = "CW"
                            print("[DECISION] Right vector clear. Launching Right Turn Profile.")
                    
                    else:
                        left_corridor = [
                            scan_records[deg] for deg in range(-90, -40)
                            if deg in scan_records and 150.0 < scan_records[deg] < 1500.0
                        ]
                        
                        if len(left_corridor) > 3:
                            avg_left_dist = sum(left_corridor) / len(left_corridor)
                            distance_error = avg_left_dist - TARGET_DISTANCE_MM
                            
                            steering_output = STEERING_CENTER + (KP_GYRO * yaw_snapshot) - (KP_LIDAR * distance_error)
                            send_steering_command(esp_ser, steering_output, BASE_SPEED)
                        else:
                            send_steering_command(esp_ser, STEERING_CENTER, BASE_SPEED)

                # ====================================================
                # STATE 1: GYRO-LOCKED CORNER PIVOT
                # ====================================================
                elif current_state == STATE_TURNING:
                    if turn_direction == "CCW":
                        send_steering_command(esp_ser, MAX_LEFT_TURN, BASE_SPEED)
                        
                        if yaw_snapshot >= 90.0:
                            print(f"\n[TURN END] Gyro reached {yaw_snapshot:.1f}°. Resetting heading matrix to 0°.")
                            esp_ser.write(b"RESET_YAW\n")
                            time.sleep(0.01) 
                            current_state = STATE_RECOVERY_STRAIGHT
                            
                    elif turn_direction == "CW":
                        send_steering_command(esp_ser, MAX_RIGHT_TURN, BASE_SPEED)
                        
                        if yaw_snapshot <= -90.0:
                            print(f"\n[TURN END] Gyro reached {yaw_snapshot:.1f}°. Resetting heading matrix to 0°.")
                            esp_ser.write(b"RESET_YAW\n")
                            time.sleep(0.01) 
                            current_state = STATE_RECOVERY_STRAIGHT

                # ====================================================
                # STATE 2: BLIND DEAD-RECKONING INTERSECTION ESCAPE
                # ====================================================
                elif current_state == STATE_RECOVERY_STRAIGHT:
                    send_steering_command(esp_ser, STEERING_CENTER, BASE_SPEED)
                    
                    left_corridor = [
                        scan_records[deg] for deg in range(-90, -40)
                        if deg in scan_records and 150.0 < scan_records[deg] < 1500.0
                    ]
                    
                    if len(left_corridor) > 3:
                        avg_left_dist = sum(left_corridor) / len(left_corridor)
                        
                        if 150.0 < avg_left_dist < 600.0:
                            print(f"\n[TRACK RECOVERED] Outer wall detected at {avg_left_dist:.0f}mm. Resuming wall follower.")
                            current_state = STATE_WALL_FOLLOW

                # --- UNIFIED MASTER DEBUGGING DASHBOARD TIER ---
                print(f"[{STATE_NAMES[current_state]}] "
                      f"L_90: {raw_left_90:4.0f}mm | "
                      f"R_90: {raw_right_90:4.0f}mm | "
                      f"Front_Avg: {avg_front_dist:4.0f}mm || "
                      f"Yaw: {yaw_snapshot:6.1f}° | "
                      f"Servo: {last_sent_steering:3d}°")

                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Manual termination sequence triggered.")
    finally:
        send_steering_command(esp_ser, STEERING_CENTER, 0)
        esp_ser.close()
        print("[SUCCESS] Safety shutdown complete. System isolated.")

if __name__ == "__main__":
    main()