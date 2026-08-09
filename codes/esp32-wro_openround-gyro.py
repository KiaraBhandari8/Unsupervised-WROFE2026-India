import serial
import math
import time
import ydlidar
import sys
import threading

# --- PHYSICAL UART COMMUNICATION CONFIGURATIONS ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

STEERING_CENTER = 90        
MAX_LEFT_TURN = 60          
MAX_RIGHT_TURN = 120        

TARGET_DISTANCE_MM = 300.0  
FRONT_TRIGGER_MM = 400.0    
BASE_SPEED = 90             

KP_GYRO = 0.35              
KP_LIDAR = 0.08

STATE_WALL_FOLLOW = 0
STATE_TURNING = 1
STATE_RECOVERY_STRAIGHT = 2
STATE_NAMES = {0: "WALL_FOLLOW", 1: "GYRO_TURNING", 2: "INTERSECTION_RECOVERY"}

current_state = STATE_WALL_FOLLOW
track_direction = "CW"             
track_direction_locked = False     

turn_state_start_time = 0.0  # Safety timeout clock tracking variable

current_yaw = 0.0
last_logical_steering = 90
last_physical_steering = 90
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
            print(f"[LIDAR] Driver initialized successfully on connection line: {self.port}")
        except Exception as e:
            print(f"[FATAL LIDAR ERROR] Driver cannot spin hardware: {e}")
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

def send_actuation_packet(ser_connection, logical_steering, motor_velocity):
    global last_logical_steering, last_physical_steering
    last_logical_steering = logical_steering
    
    physical_steering = 180 - logical_steering
    physical_steering = max(60, min(120, physical_steering))
    last_physical_steering = physical_steering
    
    packet = f"STR:{physical_steering},SPD:{motor_velocity}\n"
    ser_connection.write(packet.encode('utf-8'))

def main():
    global current_state, track_direction, track_direction_locked, current_yaw, turn_state_start_time
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=1)
        print("[INFO] High-Speed UART active. Telemetry logging online.")
    except Exception as e:
        print(f"[FATAL LINK FAILURE] Serial link unavailable at port {PI_TO_ESP_PORT}: {e}")
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

                if track_direction == "CW" or not track_direction_locked:
                    side_angles = range(-90, -40)  
                else:
                    side_angles = range(40, 90)    

                forward_corridor = [
                    scan_records[deg] for deg in range(-10, 11)
                    if deg in scan_records and scan_records[deg] > 50.0
                ]
                avg_front_dist = sum(forward_corridor) / len(forward_corridor) if len(forward_corridor) > 0 else 2000.0

                # ====================================================
                # STATE 0: AUTO-DETECTING SMOOTH WALL FOLLOWER
                # ====================================================
                if current_state == STATE_WALL_FOLLOW:
                    if avg_front_dist < FRONT_TRIGGER_MM:
                        print(f"\n[CORNER INTERCEPT] Distance: {avg_front_dist:.0f}mm. Parsing side configurations...")
                        
                        # FIXED: Group filtering arrays to remove 0mm noise drops out of side data profiles
                        left_samples = [scan_records[d] for d in range(-95, -85) if d in scan_records and scan_records[d] > 50.0]
                        right_samples = [scan_records[d] for d in range(85, 96) if d in scan_records and scan_records[d] > 50.0]
                        
                        left_side_dist = sum(left_samples) / len(left_samples) if len(left_samples) > 0 else 0.0
                        right_side_dist = sum(right_samples) / len(right_samples) if len(right_samples) > 0 else 0.0
                        
                        if not track_direction_locked:
                            # Safely evaluate if left side contains an authentic tracking boundary wall
                            if (250.0 <= left_side_dist <= 600.0) and (right_side_dist > 1500.0 or right_side_dist == 0.0):
                                track_direction = "CW"
                                print(f"[AUTO-DETECT DETAILED] Left Wall Found: {left_side_dist:.0f}mm | Right Void: {right_side_dist:.0f}mm")
                                print(">>> DECISION PARAMETER: Configured to CLOCKWISE (CW) Wall Tracking.")
                            else:
                                track_direction = "CCW"
                                print(f"[AUTO-DETECT DETAILED] Left Void: {left_side_dist:.0f}mm | Right Wall Found: {right_side_dist:.0f}mm")
                                print(">>> DECISION PARAMETER: Configured to COUNTER-CLOCKWISE (CCW) Wall Tracking.")
                            
                            track_direction_locked = True
                        
                        # Log turn timeline entry position to compute timeout limit parameters
                        turn_state_start_time = time.time()
                        current_state = STATE_TURNING
                    
                    else:
                        outer_corridor = [
                            scan_records[deg] for deg in side_angles
                            if deg in scan_records and 150.0 < scan_records[deg] < 1500.0
                        ]
                        
                        if len(outer_corridor) > 3:
                            avg_outer_dist = sum(outer_corridor) / len(outer_corridor)
                            distance_error = avg_outer_dist - TARGET_DISTANCE_MM
                            
                            if track_direction == "CW" or not track_direction_locked:
                                steering_output = STEERING_CENTER - (KP_GYRO * yaw_snapshot) - (KP_LIDAR * distance_error)
                            else:
                                steering_output = STEERING_CENTER - (KP_GYRO * yaw_snapshot) + (KP_LIDAR * distance_error)
                                
                            final_steering = max(60, min(120, int(round(steering_output))))
                            send_actuation_packet(esp_ser, final_steering, BASE_SPEED)
                        else:
                            send_actuation_packet(esp_ser, STEERING_CENTER, BASE_SPEED)

                # ====================================================
                # STATE 1: GYRO-LOCKED PIVOT LAYER WITH HARD TIMEOUT
                # ====================================================
                elif current_state == STATE_TURNING:
                    elapsed_turn_time = time.time() - turn_state_start_time
                    
                    # HARD FAILSAFE CONDITION: If turn spends more than 0.8 seconds in corner, break out!
                    if elapsed_turn_time > 0.8:
                        print(f"\n[TIMEOUT FAILSAFE] Gyro lagged at {yaw_snapshot:.1f}°. Forcing recovery sequence breakout.")
                        esp_ser.write(b"RESET_YAW\n")
                        time.sleep(0.01)
                        current_state = STATE_RECOVERY_STRAIGHT
                        continue

                    if track_direction == "CCW":
                        send_actuation_packet(esp_ser, MAX_LEFT_TURN, BASE_SPEED)
                        if yaw_snapshot <= -88.0:
                            print(f"\n[TARGET HIT] Pivot boundary complete ({yaw_snapshot:.1f}°). Zeroing coordinate grid.")
                            esp_ser.write(b"RESET_YAW\n")
                            time.sleep(0.01)
                            current_state = STATE_RECOVERY_STRAIGHT
                            
                    elif track_direction == "CW":
                        send_actuation_packet(esp_ser, MAX_RIGHT_TURN, BASE_SPEED)
                        if yaw_snapshot >= 88.0:
                            print(f"\n[TARGET HIT] Pivot boundary complete ({yaw_snapshot:.1f}°). Zeroing coordinate grid.")
                            esp_ser.write(b"RESET_YAW\n")
                            time.sleep(0.01)
                            current_state = STATE_RECOVERY_STRAIGHT

                # ====================================================
                # STATE 2: BLIND INTERSECTION ESCAPE INTERIM
                # ====================================================
                elif current_state == STATE_RECOVERY_STRAIGHT:
                    send_actuation_packet(esp_ser, STEERING_CENTER, BASE_SPEED)
                    
                    outer_corridor = [
                        scan_records[deg] for deg in side_angles
                        if deg in scan_records and 150.0 < scan_records[deg] < 1500.0
                    ]
                    
                    if len(outer_corridor) > 3:
                        avg_outer_dist = sum(outer_corridor) / len(outer_corridor)
                        if 150.0 < avg_outer_dist < 600.0:
                            print("\n[SUCCESS] Track straightaway captured. Resuming tracking system.")
                            current_state = STATE_WALL_FOLLOW

                # --- TELEMETRY DASHBOARD OUTPUT STREAM ---
                lock_status = "LOCKED" if track_direction_locked else "SEARCHING"
                print(f"[{STATE_NAMES[current_state]}] "
                      f"Mode: {track_direction} ({lock_status}) | "
                      f"Front Wall: {avg_front_dist:4.0f}mm | "
                      f"Gyro Yaw: {yaw_snapshot:6.1f}° | "
                      f"Logic Target: {last_logical_steering:3d}° -> "
                      f"Physical Servo: {last_physical_steering:3d}°")

                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Autonomous run halted manually via terminal instruction.")
    finally:
        send_actuation_packet(esp_ser, STEERING_CENTER, 0)
        esp_ser.close()
        print("[SUCCESS] Hardware lines isolated safely.")

if __name__ == "__main__":
    main()