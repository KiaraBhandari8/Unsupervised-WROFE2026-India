import subprocess
import re
import sys

proc = subprocess.Popen(
    ["python3", "unwanted/lidar_steering.py"],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    text=True
)

print("\n=== LiDAR LIVE VISUALIZER ===\n")

for line in proc.stdout:
    match = re.search(r'L_Sum: ([\d.]+).*R_Sum: ([\d.]+)', line)
    if match:
        L = float(match.group(1))
        R = float(match.group(2))
        
        max_bar = 30
        lb = "█" * min(int(L/3), max_bar)
        rb = "█" * min(int(R/3), max_bar)
        
        print(f"L:{L:5.1f} |{lb:<{max_bar}}|  R:{R:5.1f} |{rb:<{max_bar}}|")