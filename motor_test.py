#!/usr/bin/env python3
import serial, time

esp = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
esp.reset_input_buffer()
time.sleep(0.5)

def send(cmd, wait=0.3):
    print(f"  -> {cmd}")
    esp.write((cmd + "\n").encode())
    time.sleep(wait)

def read_esp():
    while esp.in_waiting:
        try:
            r = esp.readline().decode('utf-8', errors='ignore').strip()
            if r: print(f"     {r}")
        except: pass

print("1. Testing SERVO (steering) - safe, no movement")
input("   Press Enter...")
send("INIT")
send("SERVO:45", 1)
print("   Should have turned LEFT (45). Press Enter when ready.")
input()
send("SERVO:135", 1)
print("   Should have turned RIGHT (135). Press Enter when ready.")
input()
send("SERVO:90", 0.5)

print("\n2. Testing MOTOR - **HOLD THE ROBOT IN THE AIR** or prop it up")
input("   Ready? Press Enter for a 1-second burst at 30% speed...")
print("   3...", end="", flush=True); time.sleep(1)
print("2...", end="", flush=True); time.sleep(1)
print("1...", end="", flush=True); time.sleep(1)
print("GO!")
send("FORWARD:30", 1.5)
send("STOP", 0.5)
print("   Done! Did it spin forward?")

print("\n3. Testing BACKWARD - hold the robot")
input("   Press Enter for 1s backward at 30%...")
print("   3...", end="", flush=True); time.sleep(1)
print("2...", end="", flush=True); time.sleep(1)
print("1...", end="", flush=True); time.sleep(1)
print("GO!")
send("BACKWARD:30", 1.5)
send("STOP", 0.5)
print("   Done! Did it spin backward?")

print("\n=== All tests complete ===")
esp.close()
