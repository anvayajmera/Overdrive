import time

import cv2

from classes.Robot import Robot
from constants import GAP_LINE_THRESH, GUI


def gap():
    r = Robot()
    if r.line_size < GAP_LINE_THRESH:
        r.status.log("GAP", "Gap detected possibly", force=True)
        start_time = time.time()
        r.stop()
        if GUI:
            cv2.waitKey(400)
        else:
            time.sleep(0.4)

        r.forward()
        while r.line_size < GAP_LINE_THRESH:
            r.update()

            if time.time() - start_time > 2.5:
                r.status.log("GAP", "Gap timeout! Stopping.", force=True)
                r.stop()
                return

        r.status.log("GAP", "Line found, resuming linetrace.", force=True)
        r.stop()
        if GUI:
            cv2.waitKey(400)
        else:
            time.sleep(0.4)
