import time

import cv2
import numpy as np

from classes.Robot import Robot
from constants import (
    BASE_SPEED,
    GUI,
    LINE_DETECT_SIZE,
    MAX_SPEED,
    OBS_DETECT_RANGE,
    OBS_RANGE,
    TIMESTEP,
)


def obstacle():
    r = Robot()

    if r.obs_detected:
        r.stop()

        if GUI:
            cv2.waitKey(700)
        else:
            time.sleep(0.7)

        initial_yaw = r.yaw
        # turn left until robot is perpendicular to obstacle
        while r.side_distances[1] > OBS_DETECT_RANGE:
            r.update()
            if GUI:
                cv2.waitKey(1)
            r.turnLeft()

        r.status.log("OBSTACLE", f"Found obstacle on right side: {r.side_distances[1]}")

        r.stop()

        if GUI:
            cv2.waitKey(1500)
        else:
            time.sleep(1.5)

        turn_angle = (r.yaw - initial_yaw + 180.0) % 360.0 - 180.0
        r.status.log("OBSTACLE", f"Turn angle: {turn_angle}")

        # drive around until line is detected
        kp = 3.0
        while r.line_size < LINE_DETECT_SIZE:
            r.update()

            # Proportional wall following
            # error > 0 means too far from wall, error < 0 means too close
            error = r.side_distances[1] - OBS_RANGE

            # Constrain the error so we don't apply an extreme correction if sensor reads infinity
            error = max(-15.0, min(15.0, error))

            correction = int(error * kp)

            # Move forward while adjusting the steering proportionally
            # BASE_SPEED (~50) is high enough to keep the motors spinning!
            # Add a default orbital bias so the robot naturally curves around the obstacle
            orbital_bias = 15
            left_speed = BASE_SPEED + correction + orbital_bias
            right_speed = BASE_SPEED - correction - orbital_bias

            r.set_left_speed(right_speed)
            r.set_right_speed(left_speed)

            if GUI:
                cv2.waitKey(100)
            else:
                time.sleep(TIMESTEP)

        # reverse the turn we did at the start, naively, should put us in a better position
        r.turn(-turn_angle)
