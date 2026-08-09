#!/usr/bin/env python3
import subprocess, sys, re, time, serial

esp = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
esp.reset_input_buffer()
time.sleep(0.5)

print("=== LiDAR STEER-ONLY (safe on table) ===", flush=True)

proc = subprocess.Popen(
    ["python3", "-u", "/home/pi8/wrofe2025/unwanted/lidar_steering.py"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    cwd="/home/pi8/wrofe2025", bufsize=0
)

try:
    for line in iter(proc.stdout.readline, b''):
        line = line.decode(errors='ignore').strip()
        m = re.search(r'Steer_Angle:\s*([-\d.]+)', line)
        if m:
            angle = float(m.group(1))
            esp_angle = max(0, min(180, int(90 + angle)))
            esp.write(f"STR:{esp_angle},SPD:0\n".encode())
            m2 = re.search(r'L_Sum: [\d.]+(.*)', line)
            print(f"STR:{esp_angle}  {m2.group(1) if m2 else ''}", flush=True)
except KeyboardInterrupt:
    print("\nStop")
    esp.write(b"STR:90,SPD:0\n")
    esp.close()
    proc.terminate()
