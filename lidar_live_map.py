import serial
import struct
import math
import time
import ydlidar
import numpy as np
import os
import sys

# --- FORCE HEADLESS IMAGE GENERATION ENGINE ---
import matplotlib
matplotlib.use('Agg') # No GUI windows needed, completely headless safe
import matplotlib.pyplot as plt

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


# --- File-Saving Visualization Loop ---
def main():
    output_filename = "live_map.png"
    
    # 1. Pre-build the visual frame properties once to save processing power
    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': 'polar'})
    ax.set_title("YDLIDAR 360° Live Map File Output", pad=20, fontsize=12, fontweight='bold')
    ax.set_xlabel("Angle (degrees)", labelpad=15)
    ax.set_ylabel("Distances (meters)", labelpad=35)
    ax.set_rmax(16.0)
    ax.set_rlabel_position(45) 
    
    # Initialize empty scatter map points
    sensor_dots = ax.scatter([], [], s=6, color='#1f77b4', alpha=0.7)

    print(f"[INFO] Initializing image generation pipeline targets -> ./{output_filename}")
    
    try:
        with LidarScanner(port='/dev/ttyUSB0', baudrate=230400) as scanner:
            print("[SUCCESS] Data stream active. Generating map file frames...")
            frame_count = 0
            
            while True:
                scan_records = scanner.get_scan_data()
                
                if scan_records:
                    angles_rad = []
                    distances_m = []
                    
                    for angle_deg, dist_mm in scan_records.items():
                        dist_m = dist_mm / 1000.0  # Convert mm back to meters
                        if 0.02 <= dist_m <= 16.0:
                            angles_rad.append(math.radians(angle_deg))
                            distances_m.append(dist_m)
                    
                    if angles_rad:
                        # Clear coordinates matrix and refill
                        points = np.c_[angles_rad, distances_m]
                        sensor_dots.set_offsets(points)
                        
                        # Save directly to the directory file path
                        plt.savefig(output_filename, bbox_inches='tight', dpi=100)
                        
                        frame_count += 1
                        if frame_count % 10 == 0:
                            print(f"[FRAME] Updated '{output_filename}' in workspace directory.")
                
                # Sleep briefly to give the system I/O breathing room
                time.sleep(0.1)
                    
    except KeyboardInterrupt:
        print("\n[INFO] Image stream stopped safely by user.")
    except Exception as e:
        print(f"\n[ERROR] Frame routine failure: {e}")
    finally:
        plt.close(fig)
        sys.exit(0)

if __name__ == "__main__":
    main()