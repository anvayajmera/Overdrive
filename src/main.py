import Robot
from constants import SET_MOTOR_SPEED

r = Robot.Robot()

r.sm.send(SET_MOTOR_SPEED, 0, 100)