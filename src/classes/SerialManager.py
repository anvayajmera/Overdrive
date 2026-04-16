import glob
import os
import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports

from constants import START_BYTE
from globals import is_byte


class SerialManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, port: Optional[str] = None, baud=921600, write_timeout=0.05):
        if getattr(self, "_initialized", False):
            return

        self.port = port or os.getenv("ROBOT_SERIAL_PORT")
        self.baud = baud
        self.write_timeout = write_timeout
        self.ser = None
        self._lock = threading.Lock()
        self._next_reconnect_t = 0.0
        self._consecutive_write_failures = 0
        self._last_warn_t = 0.0
        self.connect()

        self._initialized = True

    def _auto_detect_port(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Usually ESP32/RP2040 show up as ttyUSB or ttyACM
            if "ACM" in p.device or "USB" in p.device:
                print(f"Auto-detected serial port: {p.device}")
                return p.device
        print("Could not auto-detect serial port, falling back to /dev/ttyACM0")
        return "/dev/ttyACM0"  # fallback

    def connect(self):
        with self._lock:
            self._perform_connect()

    def _serial_candidates(self) -> list[str]:
        candidates: list[str] = []
        if self.port:
            candidates.append(self.port)

        env_port = os.getenv("ROBOT_SERIAL_PORT")
        if env_port:
            candidates.append(env_port)

        for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
            candidates.extend(sorted(glob.glob(pattern)))

        deduped: list[str] = []
        seen: set[str] = set()
        for path in candidates:
            if path in seen:
                continue
            seen.add(path)
            deduped.append(path)
        return deduped

    def _perform_connect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

        candidates = self._serial_candidates()
        if not candidates:
            self._warn(
                "Serial connection skipped: no /dev/serial/by-id, /dev/ttyUSB*, or /dev/ttyACM* device found."
            )
            return

        last_error: Optional[Exception] = None
        for candidate in candidates:
            try:
                self.ser = serial.Serial(
                    candidate, self.baud, timeout=0.2, write_timeout=self.write_timeout
                )
                self.port = candidate
                self._consecutive_write_failures = 0
                self._warn(f"Serial connected on {candidate}", cooldown_s=0.5)
                return
            except Exception as e:
                last_error = e

        self._warn(f"Serial connection error on candidates {candidates}: {last_error}")
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
