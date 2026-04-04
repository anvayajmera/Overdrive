import math
import time

import cv2
import numpy as np
from cv2 import ximgproc

from classes.Robot import Robot
from constants import (
    BASE_SPEED_PIXEL,
    CROSSTRACK_GAIN,
    GUI,
    LINE_CONF_MAX_AREA,
    LINE_CONF_MIN_AREA,
    LINE_CONF_MIN_POINTS,
    LINE_CONF_MIN_SPAN_PX,
    LINE_CONF_NEAR_BOTTOM_FRAC,
    LINE_GOOD_FRAMES_REQUIRED,
    LINE_MAX_CONTROL_DEG,
    LINE_LOST_STOP_FRAMES,
)

from .linetrace_helpers import (
    bezier_curve,
    build_mst,
    draw_graph_on_frame,
    draw_stanley_overlay,
    extract_path,
    extract_skeleton_points,
)

_line_lost_frames = 0
_line_good_frames = 0


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
    global _line_lost_frames, _line_good_frames

    r = Robot()

    frame = r.frame.copy()
    og_frame = r.frame.copy()
    h, w, _ = r.frame.shape

    binary_path = r.binary_frame
    thinned = ximgproc.thinning(binary_path)

    key_points = extract_skeleton_points(thinned)
    nodes = build_mst(key_points)
    path_points = extract_path(nodes, w, h)

    path_count = len(path_points)
    path_span = 0
    path_bottom_y = 0
    if path_count > 0:
        y_vals = np.array([int(p[1]) for p in path_points], dtype=np.int32)
        path_span = int(y_vals.max() - y_vals.min())
        path_bottom_y = int(y_vals.max())

    line_area = float(getattr(r, "line_size", 0.0))
    min_bottom_y = int(h * LINE_CONF_NEAR_BOTTOM_FRAC)
    line_confident = (
        path_count >= LINE_CONF_MIN_POINTS
        and LINE_CONF_MIN_AREA <= line_area <= LINE_CONF_MAX_AREA
        and path_span >= LINE_CONF_MIN_SPAN_PX
        and path_bottom_y >= min_bottom_y
    )

    if line_confident:
        _line_good_frames += 1
        _line_lost_frames = 0
        if _line_good_frames < LINE_GOOD_FRAMES_REQUIRED:
            r.set_motor_output(0)
            if GUI:
                cv2.putText(
                    frame,
                    "Line acquiring...",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 220, 255),
                    2,
                )
        else:
            bez = bezier_curve(path_points)
            frame, theta, offset_error = draw_stanley_overlay(frame, bez)

            output = math.degrees(
                theta + math.atan2(CROSSTRACK_GAIN * offset_error, BASE_SPEED_PIXEL)
            )
            theta_deg = abs(math.degrees(theta))
            if theta_deg > 25.0:
                output *= 1.25
            output = max(-LINE_MAX_CONTROL_DEG, min(LINE_MAX_CONTROL_DEG, output))

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
                cv2.putText(
                    frame,
                    f"Ln:{int(line_area)} P:{path_count} Sp:{path_span}",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
    else:
        _line_good_frames = 0
        _line_lost_frames += 1
        r.status.log(
            "LINETRACE",
            (
                f"line reject area={int(line_area)} points={path_count} "
                f"span={path_span} bottom={path_bottom_y}/{min_bottom_y}"
            ),
            cooldown_s=1.0,
            cooldown_key="LINETRACE:reject",
        )
        if _line_lost_frames >= LINE_LOST_STOP_FRAMES:
            r.set_motor_output(0)
        if GUI:
            cv2.putText(
                frame,
                "Line lost",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Ln:{int(line_area)} P:{path_count} Sp:{path_span} B:{path_bottom_y}/{min_bottom_y}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
            )

    if GUI:
        if nodes:
            node_frame = draw_graph_on_frame(og_frame, nodes)
            # cv2.imshow("Nodes", node_frame)
        cv2.imshow("Original", frame)
        cv2.imshow("Thinning", thinned)
