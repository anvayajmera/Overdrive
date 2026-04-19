import time

from constants import SET_MOTOR_SPEED

from .SerialManager import SerialManager


class Motor:
    # in1 is PWM, in2 is DIR
    def __init__(self, motor_id, sm2: SerialManager):
        # Setup pins
        self.sm = sm2
        self.id = motor_id
        self.speed = -2000
        self._last_send_t = 0.0

    def set_speed(self, speed: int):
        speed = max(-100, min(speed, 100))
        now = time.monotonic()
        # Always refresh commands periodically so the ESP32 failsafe doesn't stop motors.
        if speed == self.speed and (now - self._last_send_t) < 0.15:
            return
        self.speed = speed

        sign = 2 if speed < 0 else 1

        self._last_send_t = now
        return self.sm.send(SET_MOTOR_SPEED, self.id, abs(speed), sign)
