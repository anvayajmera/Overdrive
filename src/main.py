from .Robot import Robot

r = Robot()

r.motors[0].set_speed(100)
r.motors[1].set_speed(100)
r.motors[2].set_speed(-100)
r.motors[3].set_speed(-100)
