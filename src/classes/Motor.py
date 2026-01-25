import Jetson.GPIO as GPIO

from .constants import SET_MOTOR_SPEED, MOTORS
from .SerialManager import SerialManager

class Motor:
    # in1 is PWM, in2 is DIR
    def __init__(self, motor_id, sm2: SerialManager):
        # Setup pins
        self.sm = sm2
        self.id = motor_id
        self.speed = -2000

    def set_speed(self, speed: int):
        if not MOTORS:
            return
        if speed == self.speed:
            return  
        self.speed = max(0, min(abs(speed), 100))

        sign = 2 if speed < 0 else 1

        return self.sm.send(SET_MOTOR_SPEED, self.id, self.speed, sign)