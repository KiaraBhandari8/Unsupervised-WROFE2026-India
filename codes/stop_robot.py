import serial, time
s = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
time.sleep(0.3)
s.write(b"STR:102,SPD:0\n")
s.flush()
s.close()
print("Stop sent")
