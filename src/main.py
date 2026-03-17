import os
import time

import cv2

from classes.Robot import Robot
from constants import GUI, TIMESTEP, X11
from linetrace.linetrace import init, linetrace
from green_square.green_square import green_square

if GUI and not X11:
    os.environ.setdefault("DISPLAY", ":0")

r = Robot()

init()


next_t = time.perf_counter()

# print("TIMESTEP: ", TIMESTEP)

try:
    # for i in range(100):
    #     r.backward()
    #     time.sleep(0.2)
    while True:
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)
        next_t += TIMESTEP
        r.update()

        # r.turnLeft()

        # linetrace()

        green_square()

        if GUI:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
except KeyboardInterrupt:
    print("Stopping bc of ctrl + C.")
    r.stop()

r.cleanup()
