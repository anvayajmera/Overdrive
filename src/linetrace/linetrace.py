import math
import time

import cv2
from cv2 import ximgproc

from classes.Robot import Robot
from constants import BASE_SPEED_PIXEL, CROSSTRACK_GAIN, GUI

from .linetrace_helpers import (
    bezier_curve,
    build_mst,
    draw_graph_on_frame,
    draw_stanley_overlay,
    extract_path,
    extract_skeleton_points,
)


def init():
    if GUI:
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Nodes", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Thinning", cv2.WINDOW_NORMAL)

        cv2.resizeWindow("Original", 480, 270)
        # cv2.resizeWindow("Nodes", 480, 270)
        cv2.resizeWindow("Thinning", 480, 270)


# =========================
# Main Linetrace Logic
# =========================
def linetrace():

    r = Robot()

    frame = r.frame.copy()
    og_frame = r.frame.copy()
    h, w, _ = r.frame.shape

    binary_path = r.binary_frame
    thinned = ximgproc.thinning(binary_path)

    key_points = extract_skeleton_points(thinned)
    nodes = build_mst(key_points)
    path_points = extract_path(nodes, w, h)

    if len(path_points) > 5:
        bez = bezier_curve(path_points)
        frame, theta, offset_error = draw_stanley_overlay(frame, bez)

        output = math.degrees(
            theta + math.atan2(CROSSTRACK_GAIN * offset_error, BASE_SPEED_PIXEL)
        )

        r.set_motor_output(round(output))

        if GUI:
            cv2.putText(
                frame,
                f"Angle: {round(math.degrees(theta), 1)}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Output: {round(output, 1)}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        # Experimental turning code.
        if abs(math.degrees(theta)) > 85:
            print("Linetrace turn angle: ", math.degrees(theta))
            r.stop()
            r.update()
            if GUI:
                cv2.waitKey(2000)
            else:
                time.sleep(0.4)
            r.turn(math.degrees(theta))
            r.update()

        if GUI:
            cv2.putText(
                frame,
                f"Angle: {round(math.degrees(theta), 1)}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Output: {round(output, 1)}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

    if GUI:
        if nodes:
            node_frame = draw_graph_on_frame(og_frame, nodes)
            # cv2.imshow("Nodes", node_frame)
        cv2.imshow("Original", frame)
        cv2.imshow("Thinning", thinned)
