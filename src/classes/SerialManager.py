import threading

import serial

from constants import START_BYTE
from globals import is_byte


class SerialManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, port="/dev/ttyACM0", baud=921600):
        if getattr(self, "_initialized", False):
            return

        self.port = port
        self.baud = baud
        self.ser = None
        self._lock = threading.Lock()
        self.connect()

        self._initialized = True

    def connect(self):
        with self._lock:
            self._perform_connect()

    def _perform_connect(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                self.port, self.baud, timeout=0.2, write_timeout=1.0
            )
        except Exception as e:
            print(f"Serial connection error on {self.port}: {e}")
            self.ser = None

    def send(self, command: int, arg1: int, arg2: int, arg3=1):
        if not is_byte(command):
            raise TypeError(f"{command} is not a byte")
        if not is_byte(arg1):
            raise TypeError(f"{arg1} is not a byte")
        if not is_byte(arg2):
            raise TypeError(f"{arg2} is not a byte")
        if not is_byte(arg3):
            raise TypeError(f"{arg3} is not a byte")

        packet = bytearray(5)
        packet[0] = START_BYTE
        packet[1] = command
        packet[2] = arg1
        packet[3] = arg2
        packet[4] = arg3

        with self._lock:
            if not self.ser or not self.ser.is_open:
                self._perform_connect()

            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(packet)
                except (serial.SerialException, serial.SerialTimeoutException) as e:
                    print(f"Serial write error: {e}")
                    self._perform_connect()
