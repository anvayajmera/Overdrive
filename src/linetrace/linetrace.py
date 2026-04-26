import math
import time

import cv2
import numpy as np
from cv2 import ximgproc

from classes.Robot import Robot
from constants import BASE_SPEED_PIXEL, CROSSTRACK_GAIN, GUI, TURN_THRESH

from .linetrace_helpers import (
    bezier_curve,
    build_mst,
    draw_graph_on_frame,
    draw_stanley_overlay,
    extract_path,
    extract_skeleton_points,
)

_prev_path_start: np.ndarray | None = None
_last_linetrace_time: float = 0.0


def reset_path():
    global _prev_path_start
    _prev_path_start = None
    r = Robot()
    r.status.log("GAP", "RESET PATH START")


def init():
    if GUI:
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Thinning", cv2.WINDOW_NORMAL)

        cv2.resizeWindow("Original", 480, 270)
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
    cv2.rectangle(thinned, (0, 0), (w - 1, h - 1), 0, 3)

    key_points = extract_skeleton_points(thinned)
    nodes = build_mst(key_points)
    path_points = extract_path(nodes, w, h)

    global _prev_path_start, _last_linetrace_time

    # Reset path continuity if it's been a while since linetrace was last called (e.g. during a gap)
    now = time.time()
    if now - _last_linetrace_time > 0.5:
        _prev_path_start = None
    _last_linetrace_time = now

    if len(path_points) > 1:
        p0 = np.array(path_points[0], dtype=float)
        p_end = np.array(path_points[-1], dtype=float)

        # Default: path should start from bottom of screen (higher Y is closer to bottom)
        should_flip = p_end[1] > p0[1]

        if _prev_path_start is not None:
            dist_to_start = np.linalg.norm(p0 - _prev_path_start)
            dist_to_end = np.linalg.norm(p_end - _prev_path_start)

            if min(dist_to_start, dist_to_end) < 280:
                should_flip = dist_to_end < dist_to_start

        if should_flip:
            path_points = path_points[::-1]

    _prev_path_start = np.array(path_points[0], dtype=float) if path_points else None

    if len(path_points) > 5:
        bez = bezier_curve(path_points)
        frame, theta, offset_error = draw_stanley_overlay(frame, bez)

        output = math.degrees(
            theta + math.atan2(CROSSTRACK_GAIN * offset_error, BASE_SPEED_PIXEL)
        )

        if abs(math.degrees(theta)) > TURN_THRESH:
            output *= 3

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

    if GUI:
        if nodes:
            node_frame = draw_graph_on_frame(og_frame, nodes)
        cv2.imshow("Original", frame)
        cv2.imshow("Thinning", thinned)
