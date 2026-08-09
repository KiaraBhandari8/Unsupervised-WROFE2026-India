import serial
import struct
import math
import time
import ydlidar
import os
import sys

# --- LiDAR Scanner Class (Your exact working driver configuration) ---
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

            ret = self.laser.initialize()
            if not ret:
                raise IOError(f"LiDAR connection failed: {self.laser.DescribeError()}")

            ret = self.laser.turnOn()
            if not ret:
                raise IOError(f"Failed to turn on YDLIDAR: {self.laser.DescribeError()}")

            print(f"LiDAR: Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            print(f"LiDAR ERROR: Could not connect to LiDAR: {e}")
            raise IOError(f"LiDAR connection failed: {e}")

    def disconnect(self):
        if self.laser:
            print("LiDAR: Disconnecting...")
            self.laser.turnOff()
            self.laser.disconnecting()
            self.laser = None
            print("LiDAR: Disconnected.")

    def get_scan_data(self):
        if not self.laser:
            return None

        self.scan_data = {}
        scan = ydlidar.LaserScan()

        try:
            r = self.laser.doProcessSimple(scan)
            if r:
                for p in scan.points:
                    if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                        angle_degrees = round(math.degrees(p.angle))
                        distance_mm = p.range * 1000
                        self.scan_data[angle_degrees] = distance_mm
                return self.scan_data
            else:
                return None
        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None


# --- Text-Only Terminal Runner ---
def main():
    print("[INFO] Initializing text telemetry terminal...")
    
    try:
        with LidarScanner(port='/dev/ttyUSB0', baudrate=230400) as scanner:
            print("[SUCCESS] Data stream active. Reading raw distance stamps...")
            time.sleep(1) # Allow rotation to steady out
            
            while True:
                scan_records = scanner.get_scan_data()
                
                if scan_records:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"\n--- SCAN FRAME RECEIVED [{timestamp}] | Points: {len(scan_records)} ---")
                    
                    column_count = 0
                    line_buffer = ""
                    
                    # Sort the angles numerically so they print in order from -180° to 180°
                    for angle in sorted(scan_records.keys()):
                        distance = scan_records[angle]
                        
                        # Format into a uniform block: "Angle -> Distance"
                        line_buffer += f"[{angle:4d}°: {int(distance):5d}mm]   "
                        column_count += 1
                        
                        # Print 4 angle blocks per line for a clean readable layout
                        if column_count % 4 == 0:
                            print(line_buffer)
                            line_buffer = ""
                    
                    # Print any remaining points left over in the line buffer
                    if line_buffer:
                        print(line_buffer)
                else:
                    print("[WAITING] Data packet empty or unaligned. Spinning turret...")
                
                # Sleep for 500ms between print cycles to keep the terminal readable
                time.sleep(0.5)
                
    except KeyboardInterrupt:
        print("\n[INFO] Text telemetry stream stopped cleanly via user shortcut.")
    except Exception as e:
        print(f"\n[FATAL ERROR] System Failure: {e}")
    finally:
        print("[INFO] Script execution finalized.")
        sys.exit(0)

if __name__ == "__main__":
    main()