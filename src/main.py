import os
import time

import cv2

from classes.Robot import Robot
from constants import GREEN_CHECK_PERIOD_FRAMES, GUI, TIMESTEP, X11
from gap.gap import gap
from green_square.green_square import green_square
from linetrace.linetrace import init, linetrace
from obstacle.obstacle import obstacle

if GUI and not X11:
    os.environ.setdefault("DISPLAY", ":0")

r = Robot()

init()


next_t = time.perf_counter()
loop_idx = 0

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
        loop_idx += 1

        # Run green marker behavior at a lower frequency to keep line control smooth.
        handled_green = False
        if loop_idx % GREEN_CHECK_PERIOD_FRAMES == 0:
            handled_green = green_square()
        if not handled_green:
            # Primary behavior: line follow
            linetrace()

            # gap()

        if GUI:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                r.turn(90)
            if key == ord("l"):
                r.turn(-90)
            if key == ord("u"):
                r.turn(180)
            if key == ord("q"):
                break
except KeyboardInterrupt:
    print("Stopping bc of ctrl + C.")
finally:
    try:
        r.stop()
    except Exception as e:
        print(f"Error stopping robot: {e}")
    finally:
        r.cleanup()
