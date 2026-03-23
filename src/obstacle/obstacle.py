import time

import cv2
import numpy as np

from classes.Robot import Robot
from constants import GUI, LINE_DETECT_SIZE, OBS_DETECT_RANGE, OBS_RANGE, TIMESTEP


def obstacle():
    r = Robot()

    if r.obs_detected:
        r.stop()

        initial_yaw = r.yaw
        # turn left until robot is perpendicular to obstacle
        while r.side_distances[0] > OBS_DETECT_RANGE:
            r.update()
            if GUI:
                cv2.waitKey(1)
            r.turnLeft()

        turn_angle = (r.yaw - initial_yaw + 180.0) % 360.0 - 180.0
        print(f"Turn angle: {turn_angle}")

        # drive around until line is detected
        # while r.line_size < LINE_DETECT_SIZE:
        #     r.update()
        #     if r.side_distances[0] > OBS_RANGE:
        #         r.turnRight()
        #     else:
        #         r.turnLeft()
        #     if GUI:
        #         cv2.waitKey(int(2 * TIMESTEP * 1000))
        #     else:
        #         time.sleep(TIMESTEP * 2)

        # reverse the turn we did at the start, naively, should put us in a better position
        # r.turn(-turn_angle)
