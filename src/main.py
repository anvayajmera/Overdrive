import os
import time

import cv2

from classes.Robot import Robot
from constants import (
    GREEN_CHECK_PERIOD_FRAMES,
    GUI,
    SILVER_CHECK_PERIOD_FRAMES,
    TIMESTEP,
    X11,
)
from gap.gap import gap
from green_square.green_square import green_square
from linetrace.linetrace import init, linetrace
from obstacle.obstacle import obstacle
from silver.silver import silver
from victims.victim import victim, victim_init

if GUI and not X11:
    os.environ.setdefault("DISPLAY", ":0")

r = Robot()

# victim_init()


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

        victim()

        # Silver line detection (Rescue Zone entry)
        if loop_idx % SILVER_CHECK_PERIOD_FRAMES == 0:
            silver()

        # # Run green marker behavior at a lower frequency to keep line control smooth.
        handled_green = False
        if loop_idx % GREEN_CHECK_PERIOD_FRAMES == 0:
            # handled_green = green_square()
            pass
        if not handled_green:
            # Primary behavior: line follow
            # r.forward()

            # r.forward()
            #
            # linetrace()

            # gap()

            # obstacle()

            pass

        if GUI:
            # cv2.imshow("Ball Camera", r.ball_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                r.turn(90)
            if key == ord("l"):
                r.turn(-90)
            if key == ord("u"):
                r.turn(180)
            elif key == ord("c"):
                timestamp = int(time.time())

                save_dir = "images"
                os.makedirs(save_dir, exist_ok=True)

                raw_filename = os.path.join(save_dir, f"capture_raw_{timestamp}.jpg")
                cv2.imwrite(raw_filename, r.ball_frame)
                print(f"Saved raw frame to {raw_filename}")
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
