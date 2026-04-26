import time

import cv2
import numpy as np

from classes.Robot import Robot
from constants import (
    BASE_SPEED,
    GUI,
    LINE_DETECT_SIZE,
    OBS_DETECT_RANGE,
    OBS_RANGE,
    TIMESTEP,
)
from linetrace.linetrace import reset_path


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
        kp = 5.0
        while r.line_size < LINE_DETECT_SIZE:
            r.update()

            r.status.log("OBSTACLE", f"Line size: {r.line_size}")

            if r.obs_detected:
                # If the front sensor sees the obstacle again (too sharp of a turn), spin out!
                r.set_left_speed(-35)
                r.set_right_speed(35)
            else:
                # Proportional wall following
                target_dist = 6.0  # Stick closer to the 4-5cm wide obstacle
                dist = r.side_distances[1]

                if dist > 30.0:
                    # We lost the wall! This means we hit the corner of the wide obstacle.
                    # Move mostly forward with a slight right curve to clear the thickness.
                    r.set_left_speed(BASE_SPEED + 20)
                    r.set_right_speed(BASE_SPEED - 5)
                else:
                    # Normal wall hugging
                    error = dist - target_dist
                    error = max(-10.0, min(10.0, error))
                    correction = int(error * kp)

                    orbital_bias = 15
                    left_speed = BASE_SPEED + correction + orbital_bias
                    right_speed = BASE_SPEED - correction - orbital_bias

                    r.set_left_speed(left_speed)
                    r.set_right_speed(right_speed)

            if GUI:
                cv2.waitKey(100)
            else:
                time.sleep(TIMESTEP)

        # reverse the turn we did at the start, naively, should put us in a better position
        r.turn(-turn_angle)

        r.stop()
        reset_path()
