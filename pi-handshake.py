import serial
import time

# Use the direct hardware address
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

print("--- [PI 5 MASTER] ---")
print("Shouting at ESP32 every 1 second...")

try:
    while True:
        # Clear the buffer before sending
        ser.reset_input_buffer()
        
        ser.write(b"S:90,M:150\n")
        print("PI: 'Hey ESP32, can you hear me?'", end='\r')

        time.sleep(0.2) # Wait for ESP32 to blink and reply

        if ser.in_waiting > 0:
            reply = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"\n[SUCCESS] ESP32 REPLIED: {reply}")
        
        time.sleep(0.8)

except KeyboardInterrupt:
    ser.close()