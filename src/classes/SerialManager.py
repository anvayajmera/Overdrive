import threading
import time

import serial

from constants import START_BYTE
from globals import is_byte


class SerialManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, port="/dev/ttyACM0", baud=921600, write_timeout=0.05):
        if getattr(self, "_initialized", False):
            return

        self.port = port
        self.baud = baud
        self.write_timeout = write_timeout
        self.ser = None
        self._lock = threading.Lock()
        self._next_reconnect_t = 0.0
        self._consecutive_write_failures = 0
        self._last_warn_t = 0.0
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
                self.port, self.baud, timeout=0.2, write_timeout=self.write_timeout
            )
            self._consecutive_write_failures = 0
        except Exception as e:
            self._warn(f"Serial connection error on {self.port}: {e}")
            self.ser = None

    def _warn(self, msg: str, cooldown_s: float = 2.0):
        now = time.monotonic()
        if now - self._last_warn_t >= cooldown_s:
            print(msg)
            self._last_warn_t = now

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
            now = time.monotonic()

            if not self.ser or not self.ser.is_open:
                if now >= self._next_reconnect_t:
                    self._perform_connect()
                else:
                    return

            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(packet)
                    self._consecutive_write_failures = 0
                except (serial.SerialException, serial.SerialTimeoutException) as e:
                    self._consecutive_write_failures += 1
                    self._warn(f"Serial write error: {e}")

                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None

                    # Back off reconnect attempts so repeated motor commands do not stall the loop.
                    if self._consecutive_write_failures >= 3:
                        self._next_reconnect_t = time.monotonic() + 1.0
                    else:
                        self._next_reconnect_t = time.monotonic() + 0.1
