# esp32.py

import serial
import threading


class ESP32Controller:

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200
    ):

        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=0.1
        )

        self.lock = threading.Lock()

        print("[ESP32] Connected")

    def send(
        self,
        steering,
        speed
    ):

        steering = int(
            max(
                0,
                min(180, steering)
            )
        )

        speed = int(
            max(
                0,
                min(255, speed)
            )
        )

        packet = (
            f"STR:{steering},SPD:{speed}\n"
        )

        with self.lock:
            self.ser.write(
                packet.encode()
            )

    def stop(self):

        self.send(
            90,
            0
        )

    def close(self):

        self.stop()

        try:
            self.ser.close()
        except:
            pass

        print("[ESP32] Disconnected")