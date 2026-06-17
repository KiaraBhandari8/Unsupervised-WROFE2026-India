import serial, time

s = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.2)
s.reset_input_buffer()
time.sleep(1)

def go(cmd, duration):
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    print(f"  -> {cmd}")
    time.sleep(duration)
    r = s.read(200).decode(errors="ignore")
    print(f"  <- {r[:120].strip()}")

print("=== ESP32 TEST ===", flush=True)

print("\n1. CENTER steering", flush=True)
go("STR:90,SPD:0", 1)

print("\n2. LEFT (45 deg)", flush=True)
go("STR:45,SPD:0", 1.5)

print("\n3. RIGHT (135 deg)", flush=True)
go("STR:135,SPD:0", 1.5)

print("\n4. CENTER + FORWARD (SPD:100)", flush=True)
go("STR:90,SPD:100", 2)

print("\n5. STOP", flush=True)
go("STR:90,SPD:0", 1)

print("\n=== DONE ===", flush=True)
s.close()
