import time

import cv2
import numpy as np

from classes.Robot import Robot
from constants import GAP_LINE_THRESH, GUI
from linetrace.linetrace import reset_path


def align_to_line(r: Robot):
    """Turns the robot in place until the line is pointing straight forward."""
    r.update()  # Get fresh camera frame

    # Get contours of the line
    contours, _ = cv2.findContours(
        r.binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return

    largest = max(contours, key=cv2.contourArea)

    # Fit a bounding box to get the angle of the line
    rect = cv2.minAreaRect(largest)
    angle = rect[-1]

    # minAreaRect angles range from 0 to 90.
    # We adjust it so 0 degrees is straight ahead.
    width, height = rect[1]
    if width < height:
        angle = angle - 90

    # Depending on how your turn() method maps positive/negative degrees
    # to left/right, you might need to make this positive or negative.
    turn_angle = -angle

    # If the line is off by more than a few degrees, turn the robot to face it
    if abs(turn_angle) > 5.0:
        r.status.log(
            "GAP", f"Aligning before gap: {turn_angle:.1f} degrees", force=True
        )
        r.turn(turn_angle)

        # Give it a tiny pause to settle after turning
        if GUI:
            cv2.waitKey(200)
        else:
            time.sleep(0.2)


def gap():
    r = Robot()
    if r.line_size < GAP_LINE_THRESH:
        r.status.log("GAP", "Gap detected possibly", force=True)
        start_time = time.time()

        # 1. Stop at the edge of the gap
        r.stop()
        if GUI:
            cv2.waitKey(400)
        else:
            time.sleep(0.4)

        # 2. Reorient robot with the tail end of the line BEFORE crossing
        align_to_line(r)

        # 3. Drive forward blindly across the gap
        r.forward()
        while r.line_size < GAP_LINE_THRESH:
            r.update()

            if time.time() - start_time > 2.5:
                r.status.log("GAP", "Gap timeout! Stopping.", force=True)
                r.stop()
                return

        # 4. We found the line on the other side! Stop.
        if GUI:
            cv2.waitKey(700)
        else:
            time.sleep(0.7)

        r.status.log("GAP", "Line found, resuming linetrace.", force=True)
        r.stop()

        # Optional: You can align again here just to be perfectly safe
        # align_to_line(r)

        reset_path()

        if GUI:
            cv2.waitKey(800)
        else:
            time.sleep(0.8)
        r.update()
