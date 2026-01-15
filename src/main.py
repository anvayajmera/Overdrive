import Robot, time
from constants import SET_MOTOR_SPEED

r = Robot.Robot()

print(r.sm.send(SET_MOTOR_SPEED, 0, 100))
time.sleep(2)
print(r.sm.send(SET_MOTOR_SPEED, 0, 0))
