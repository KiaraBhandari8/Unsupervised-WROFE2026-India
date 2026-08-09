#!/usr/bin/env python3
import subprocess, re, sys, os

os.chdir('/home/pi8/wrofe2025')

proc = subprocess.Popen(
    ['/home/pi8/wrofe2025/env_test/bin/python3', 'unwanted/lidar_steering.py'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
)

print('╔════════════════════════════════════════════════════╗')
print('║           LiDAR LIVE VISUALIZER                     ║')
print('╚════════════════════════════════════════════════════╝')
print()

try:
    for line in proc.stdout:
        m = re.search(r'L_Sum: ([\d.]+).*R_Sum: ([\d.]+)', line)
        if m:
            L, R = float(m.group(1)), float(m.group(2))
            lb = '█' * min(int(L/2.5), 28)
            rb = '█' * min(int(R/2.5), 28)
            print(f'L:{L:5.1f} |{lb:<28}|   R:{R:5.1f} |{rb:<28}|')
except KeyboardInterrupt:
    print('\nStopped!')
    proc.terminate()