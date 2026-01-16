from . import SerialManager

from . import Motor

class Robot:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        self._initialized = True
        self.sm = SerialManager.SerialManager()
        self.motors = [Motor.Motor(i) for i in range(4)]


