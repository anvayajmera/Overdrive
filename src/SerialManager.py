from constants import START_BYTE
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
        self.ser = serial.Serial(port, baud)

        self._initialized = True

    def send(self, command, arg1, arg2):
        packet = bytearray(5)
        packet[0] = START_BYTE
        packet[1] = command
        packet[2] = arg1
        packet[3] = arg2
        packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3]

        self.ser.write(packet)
        return self.ser.readline().decode().strip()
