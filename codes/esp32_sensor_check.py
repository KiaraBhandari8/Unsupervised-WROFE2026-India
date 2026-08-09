import serial
import time

# Use /dev/ttyAMA0 for Pi 5 GPIO UART
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
    ser.reset_input_buffer()
    print("--- [SUCCESS] PI 5 & ESP32 BIDIRECTIONAL LINK ---")
except Exception as e:
    print(f"--- [ERROR] Connection Failed: {e} ---")
    exit()

def run_robot_cycle(angle, speed):
    # Send the command
    cmd = f"S:{angle},M:{speed}\n"
    ser.write(cmd.encode())
    
    # Listen for MPU data
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("DATA:"):
                # line format is "DATA:accel_z,gyro_z"
                raw_vals = line.replace("DATA:", "").split(",")
                print(f"[LIVE SENSOR] Accel_Z: {raw_vals[0]} | Gyro_Z: {raw_vals[1]}")
        except:
            pass

try:
    print("Starting WRO Test: Cycling Steering and Motor...")
    while True:
        # Move Left at speed 150
        print("\nACTION: Left / Drive")
        for _ in range(20): # Run for ~2 seconds
            run_robot_cycle(45, 150)
            time.sleep(0.1)

        # Move Right at speed 150
        print("\nACTION: Right / Drive")
        for _ in range(20):
            run_robot_cycle(135, 150)
            time.sleep(0.1)

except KeyboardInterrupt:
    print("\nShutting down...")
    ser.write(b"S:90,M:0\n") # Center wheels and stop motor
    ser.close()