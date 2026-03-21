import os
import time

import cv2

from classes.Robot import Robot
from constants import GREEN_KI, GUI, TIMESTEP, X11
from green_square.green_square import green_square
from linetrace.linetrace import init, linetrace

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

        turn_dir, error = green_square()

        if turn_dir != "straight":
            r.stop()
            r.forward()
            time.sleep(error * GREEN_KI)
            r.stop()
            time.sleep(0.2)

            r.update()

            turn_dir, _average_y = green_square()

            if turn_dir == "left":
                r.turn(-90)
            elif turn_dir == "right":
                r.turn(90)
            elif turn_dir == "turn_around":
                r.turn(180)

        if GUI:
            if cv2.waitKey(1) & 0xFF == ord("r"):
                r.turn(90)
            if cv2.waitKey(1) & 0xFF == ord("l"):
                r.turn(-90)
            if cv2.waitKey(1) & 0xFF == ord("u"):
                r.turn(180)
            if cv2.waitKey(1) & 0xFF == ord("q"):
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
