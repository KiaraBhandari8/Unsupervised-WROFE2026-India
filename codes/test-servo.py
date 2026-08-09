import serial
import time
import sys
import threading

# --- HARDWARE SYSTEM CONFIGURATION ---
PI_TO_ESP_PORT = "/dev/ttyAMA0"
BAUD_RATE_ESP = 115200

# Global tracking variables for background operations
current_yaw = 0.0
live_stream_active = False
shutdown_event = threading.Event()

def telemetry_receiver_thread(ser):
    # Background worker thread that constantly listens for data from the ESP32
    global current_yaw, live_stream_active
    while not shutdown_event.is_set():
        if ser.in_waiting > 0:
            try:
                raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_line.startswith("YAW:"):
                    current_yaw = float(raw_line.split(":")[1])
                    # If the user explicitly engaged live mode, stream it over the terminal
                    if live_stream_active:
                        print(f"\r[LIVE YAW] Current Sensor Heading: {current_yaw:+.2f}°", end="", flush=True)
            except (ValueError, IndexError):
                pass  # Ignore partial data packets smoothly
        time.sleep(0.01)

def send_actuation_packet(ser, angle, speed=0):
    # Standard output formatting engine matching the ESP32 parse sequence
    packet = f"STR:{angle},SPD:{speed}\n"
    ser.write(packet.encode('utf-8'))
    ser.flush()

def main():
    global live_stream_active
    print("==========================================================")
    print("[DIAGNOSTIC] Unified Servo & Gyro Calibration Dashboard")
    print("==========================================================")
    
    try:
        esp_ser = serial.Serial(PI_TO_ESP_PORT, BAUD_RATE_ESP, timeout=0.1)
        time.sleep(1.5)  # Let the serial line settle completely
        print("[SUCCESS] Communication pipeline initialized.")
    except Exception as e:
        print(f"[FATAL] Cannot mount serial interface on {PI_TO_ESP_PORT}: {e}")
        sys.exit(1)

    # Launch background telemetry collector
    bg_thread = threading.Thread(target=telemetry_receiver_thread, args=(esp_ser,))
    bg_thread.daemon = True
    bg_thread.start()

    # Initial safety initialization sequence
    print("[ACTION] Aligning chassis center to 90°...")
    send_actuation_packet(esp_ser, 90, speed=0)
    print("[ACTION] Zeroing out gyroscope orientation registers...")
    esp_ser.write(b"RST_YAW\n")
    esp_ser.flush()
    time.sleep(0.2)

    print("\n----------------------------------------------------------")
    print("COMMAND ENGINE GUIDE:")
    print("  --> Enter an Integer (0-180)  : Sets steering servo position")
    print("  --> Type 'rst'                 : Resets current position to 0.0°")
    print("  --> Type 'live'                : Activates 10-second hand-turn gyro stream")
    print("  --> Type 'exit'                : Safely centers wheels and terminates")
    print("----------------------------------------------------------")

    try:
        while True:
            # Query the user for interactive input parameters
            user_command = input("\nEnter Command: ").strip().lower()
            
            if user_command == 'exit':
                print("[SHUTDOWN] Terminating control loops...")
                break
                
            elif user_command == 'rst':
                print("[ACTION] Sending zero-point calibration packet down to ESP32...")
                esp_ser.write(b"RST_YAW\n")
                esp_ser.flush()
                time.sleep(0.2)
                print(f"[SUCCESS] Orientation zeroed. Current Yaw: {current_yaw:.2f}°")
                
            elif user_command == 'live':
                print("\n==========================================================")
                print("[LIVE MODE] Starting real-time gyro tracking loop...")
                print("Rotate your robot by hand now to benchmark angular accuracy.")
                print("Loop will automatically close in 10 seconds.")
                print("==========================================================")
                time.sleep(0.5)
                
                # Divert background thread parameters to stream to console screen
                live_stream_active = True
                time.sleep(10.0)  # Stream duration window block
                live_stream_active = False
                
                print("\n==========================================================")
                print("[LIVE MODE] Tracking loop closed. Returning to panel input.")
                print("==========================================================")
                
            else:
                # Attempt to parse input as a standard servo positioning value
                try:
                    target_angle = int(user_command)
                    if 0 <= target_angle <= 180:
                        send_actuation_packet(esp_ser, target_angle, speed=0)
                        time.sleep(0.1)  # Let mechanics snap over
                        print(f"[STATUS] Servo Angle: {target_angle}° | Current Gyro Yaw: {current_yaw:+.2f}°")
                    else:
                        print("[ERROR] Boundaries exceeded! Value must fall between 0 and 180.")
                except ValueError:
                    print("[ERROR] Unrecognized parameter. Use integer, 'rst', 'live', or 'exit'.")

    except KeyboardInterrupt:
        print("\n[STOP] Manual terminal cancellation received.")
    finally:
        shutdown_event.set()
        print("\n[CLEANUP] Straightening wheels back to 90° center alignment...")
        send_actuation_packet(esp_ser, 90, speed=0)
        esp_ser.close()
        print("[CLEANUP] Serial line disconnected safely.")

if __name__ == "__main__":
    main()