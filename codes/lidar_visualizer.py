#!/usr/bin/env python3
import ydlidar
import math
import time
import sys

import subprocess

print("Starting LiDAR ASCII Radar...")
print("Press Ctrl+C to quit\n")

cmd = [sys.executable, "-c", f"""
import ydlidar
import math
import time
import sys
ydlidar.os_init()
l = ydlidar.CYdLidar()
l.setlidaropt(ydlidar.LidarPropSerialPort, '/dev/ttyUSB0')
l.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
l.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
l.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
l.setlidaropt(ydlidar.LidarPropSampleRate, 4)
l.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
l.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
l.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
l.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
l.initialize()
l.turnOn()
print("LiDAR ready")
sys.stdout.flush()
while True:
    scan = ydlidar.LaserScan()
    if l.doProcessSimple(scan):
        pts = [(math.degrees(p.angle), p.range*1000) for p in scan.points if 0 < p.range < 16]
        L = sum(d for a,d in pts if 0 <= a <= 90)
        R = sum(d for a,d in pts if 90 <= a <= 180)
        print(f"L:{{L:.0f}} R:{{R:.0f}}")
        sys.stdout.flush()
    time.sleep(0.1)
"""]

proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

for line in proc.stdout:
    print(line.rstrip())