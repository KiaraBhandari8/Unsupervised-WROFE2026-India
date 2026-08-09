import time
import board
import busio
import adafruit_vl53l0x

# Set up I2C bus and sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vl53l0x.VL53L0X(i2c)

print("Starting distance readings (Ctrl+C to stop)...")

try:
    while True:
        distance_mm = sensor.range
        print(f"Distance: {distance_mm} mm")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped.")