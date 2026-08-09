import serial
import time
import sys

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 230400

def debug_stream():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        print(f"[INFO] Serial port {SERIAL_PORT} opened successfully.")
        
        # Clear any residual power-up garbage
        ser.reset_input_buffer()
        
        print("[INFO] Sending Wake-up Command (0xA5 0x60)...")
        ser.write(b'\xA5\x60')
        time.sleep(0.2)
        
        # Explicitly read and clear the 7-byte reply descriptor
        if ser.in_waiting >= 7:
            descriptor = ser.read(7)
            print(f"[HANDSHAKE] LiDAR Device Descriptor Received: {descriptor.hex().upper()}")
        else:
            print("[WARNING] No device descriptor received, checking for direct stream...")

        print("[INFO] Searching for 0xAA 0x55 point-cloud sync packets. Running for 5 seconds...")
        
        packet_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 5:
            # Look for the 2-byte packet signature
            b1 = ser.read(1)
            if not b1 or b1[0] != 0xAA: 
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != 0x55: 
                continue
            
            # If we pass the guard, we hit a real packet header
            header = ser.read(8)
            if len(header) < 8: 
                continue
                
            package_type = header[0]
            sample_count = header[1]
            packet_count += 1
            
            print(f"[PACKET #{packet_count}] Sync Found! Type: {package_type} | Samples in Payload: {sample_count}")
            
            # Flush out the sample payload bytes to keep the buffer clean
            ser.read(sample_count * 2)

        if packet_count == 0:
            print("[FAIL] Motor is spinning, but zero 0xAA 0x55 data frames were processed.")
        else:
            print(f"\n[SUCCESS] Telemetry verified! Processed {packet_count} packets cleanly.")

    except Exception as e:
        print(f"[ERROR] Diagnostic failed: {e}")
    finally:
        print("[INFO] Sending Sleep Command (0xA5 0x65) to spin down motor...")
        ser.write(b'\xA5\x65')
        ser.close()
        print("[INFO] Port isolated.")

if __name__ == "__main__":
    debug_stream()