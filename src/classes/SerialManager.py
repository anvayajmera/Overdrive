from .constants import START_BYTE
from .globals import is_byte
import serial

class SerialManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, port="/dev/ttyACM0", baud=115200):
        if getattr(self, '_initialized', False):
            return

        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.1)

        self._initialized = True

    def send(self, command: int, arg1: int, arg2: int, arg3 = 1):
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

        self.ser.write(packet)
        self.ser.flush()
        
        return self.ser.readline().decode().strip()
