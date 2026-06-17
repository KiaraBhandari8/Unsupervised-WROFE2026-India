import serial
import time

# Try ttyAMA0 first, then serial0
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

print("Starting Loopback Test... (Short Pin 8 to Pin 10)")

try:
    while True:
        ser.write(b"LOOPBACK_TEST\n")
        time.sleep(0.1)
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print(f"PI HEARD ITSELF: {line}")
        else:
            print("PI IS DEAF: No data received on RX")
        time.sleep(1)
except:
    ser.close()