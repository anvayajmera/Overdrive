import Jetson.GPIO as GPIO

from .constants import SET_MOTOR_SPEED
from .Robot import Robot

class Motor:
    # in1 is PWM, in2 is DIR
    def __init__(self, motor_id):
        # Setup pins
        self.sm = Robot().sm
        self.id = motor_id
        self.speed = 0

    def set_speed(self, speed: int):
        if speed == self.speed:
            return
        self.speed = speed

        sign = 2 if speed < 0 else 1

        self.sm.send(self, SET_MOTOR_SPEED, self.id, self.speed, sign)