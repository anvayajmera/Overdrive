import time
from typing import Optional, Sequence

import cv2
import numpy as np

from classes.Robot import Robot
from constants import (
    GREEN_ACTION_COOLDOWN_S,
    GREEN_CONFIRM_FRAMES,
    GREEN_KI,
    GREEN_MAX,
    GREEN_MIN,
    GREEN_SQUARE_MIN_AREA,
    GREEN_SQUARE_ROI,
    GUI,
    LINE_CAM_HEIGHT,
    LINE_CAM_WIDTH,
)

# Pre-allocate numpy arrays for performance to avoid creating them every frame
_GREEN_MIN_ARR = np.array(GREEN_MIN, dtype=np.uint8)
_GREEN_MAX_ARR = np.array(GREEN_MAX, dtype=np.uint8)

# Pre-compute morphological kernels
# Using cv2.getStructuringElement is generally equivalent to np.ones for MORPH_RECT
_KERNEL_3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
# 11 iterations of 3x3 is equivalent to 1 iteration of 23x23
_KERNEL_23 = cv2.getStructuringElement(cv2.MORPH_RECT, (23, 23))
# 9 iterations of 3x3 is equivalent to 1 iteration of 19x19
_KERNEL_19 = cv2.getStructuringElement(cv2.MORPH_RECT, (19, 19))

_pending_turn_dir = "straight"
_pending_turn_frames = 0
_last_green_action_t = 0.0


def find_green_square(frame: np.ndarray) -> Sequence[np.ndarray]:
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv_image, _GREEN_MIN_ARR, _GREEN_MAX_ARR)

    # Morphological operations optimized: using larger kernels instead of many iterations.
    # This drastically reduces memory read/write overhead while achieving the exact same mathematical effect.
    green_image = cv2.erode(green_image, _KERNEL_3, iterations=1)
    green_image = cv2.dilate(green_image, _KERNEL_23, iterations=1)
    green_image = cv2.erode(green_image, _KERNEL_19, iterations=1)

    contours_grn, _ = cv2.findContours(
        green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    contours_grn = [
        contour
        for contour in contours_grn
        if cv2.contourArea(contour) > GREEN_SQUARE_MIN_AREA
    ]

    return contours_grn


def check_black(
    green_contour: np.ndarray, black_image: np.ndarray, gui_frame: Optional[np.ndarray]
) -> np.ndarray:
    green_rect = cv2.boxPoints(cv2.minAreaRect(green_contour))
    green_rect = green_rect[green_rect[:, 1].argsort()]  # sort by y val

    if GUI and gui_frame is not None:
        draw_rect = np.intp(green_rect)
        cv2.drawContours(gui_frame, [draw_rect], -1, (0, 0, 255), 2)  # type:ignore

    marker_height = green_rect[-1][1] - green_rect[0][1]  # greatest y - least y

    black_around_sign = np.zeros(6, dtype=np.int16)
    black_around_sign[4] = int(green_rect[2][1])
    black_around_sign[5] = int(green_rect[0][1])

    # Bottom
    y1_b = int(green_rect[2][1])
    y2_b = min(int(green_rect[2][1] + (marker_height * 0.8)), LINE_CAM_HEIGHT)
    x1_b = min(int(green_rect[2][0]), int(green_rect[3][0]))
    x2_b = max(int(green_rect[2][0]), int(green_rect[3][0]))
    roi_b = black_image[y1_b:y2_b, x1_b:x2_b]
    if roi_b.size > 0 and np.mean(roi_b) > 125:
        black_around_sign[0] = 1

    # Top
    y1_t = max(int(green_rect[1][1] - (marker_height * 0.8)), 0)
    y2_t = int(green_rect[1][1])
    x1_t = min(max(int(green_rect[0][0]), 0), max(int(green_rect[1][0]), 0))
    x2_t = max(max(int(green_rect[0][0]), 0), max(int(green_rect[1][0]), 0))
    roi_t = black_image[y1_t:y2_t, x1_t:x2_t]
    if roi_t.size > 0 and np.mean(roi_t) > 125:
        black_around_sign[1] = 1

    green_rect = green_rect[green_rect[:, 0].argsort()]

    # Left
    y1_l = min(int(green_rect[0][1]), int(green_rect[1][1]))
    y2_l = max(int(green_rect[0][1]), int(green_rect[1][1]))
    x1_l = max(int(green_rect[1][0] - (marker_height * 0.8)), 0)
    x2_l = int(green_rect[1][0])
    roi_l = black_image[y1_l:y2_l, x1_l:x2_l]
    if roi_l.size > 0 and np.mean(roi_l) > 125:
        black_around_sign[2] = 1

    # Right
    y1_r = min(int(green_rect[2][1]), int(green_rect[3][1]))
    y2_r = max(int(green_rect[2][1]), int(green_rect[3][1]))
    x1_r = int(green_rect[2][0])
    x2_r = min(int(green_rect[2][0] + (marker_height * 0.8)), LINE_CAM_WIDTH)
    roi_r = black_image[y1_r:y2_r, x1_r:x2_r]
    if roi_r.size > 0 and np.mean(roi_r) > 125:
        black_around_sign[3] = 1

    return black_around_sign


def determine_turn_direction(black_borders):
    turn_left = False
    turn_right = False
    left_low = False
    right_low = False

    average_y = 0

    for i in black_borders:
        if np.sum(i[:4]) == 2:
            if i[1] == 1 and i[2] == 1:
                average_y += (i[4] + i[5]) // 2
                turn_right = True
                if i[4] > LINE_CAM_HEIGHT * 0.95:
                    right_low = True
                    average_y -= (i[4] + i[5]) // 2

            elif i[1] == 1 and i[3] == 1:
                average_y += (i[4] + i[5]) // 2
                turn_left = True
                if i[4] > LINE_CAM_HEIGHT * 0.95:
                    left_low = True
                    average_y -= (i[4] + i[5]) // 2

    if len(black_borders) > 0:
        average_y //= len(black_borders)

    return turn_left, turn_right, left_low, right_low, average_y


def read_squares() -> tuple[str, int]:
    r = Robot()
    contours_grn = find_green_square(r.frame)

    black_borders = []

    gui_frame = r.frame.copy() if GUI else None

    for contour in contours_grn:
        black_around_sign = check_black(contour, r.binary_frame, gui_frame)
        if black_around_sign[5] < LINE_CAM_HEIGHT * GREEN_SQUARE_ROI:
            continue
        black_borders.append(black_around_sign)

    turn_left, turn_right, left_bottom, right_bottom, average_y = (
        determine_turn_direction(black_borders)
    )

    res = "straight"

    if turn_left and not turn_right and not left_bottom:
        res = "left"
    elif turn_right and not turn_left and not right_bottom:
        res = "right"
    elif turn_left and turn_right and not (left_bottom and right_bottom):
        res = "turn_around"

    if GUI and gui_frame is not None:
        cv2.putText(
            gui_frame,
            res,
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Green Square", gui_frame)

    return res, average_y - (LINE_CAM_HEIGHT // 2 - 40)


def green_square() -> bool:
    global _pending_turn_dir, _pending_turn_frames, _last_green_action_t

    r = Robot()

    turn_dir, _error = read_squares()
    now = time.monotonic()

    if turn_dir == "straight":
        _pending_turn_dir = "straight"
        _pending_turn_frames = 0
        return False

    # if now - _last_green_action_t < GREEN_ACTION_COOLDOWN_S:
    #     return False

    if turn_dir != _pending_turn_dir:
        _pending_turn_dir = turn_dir
        _pending_turn_frames = 1
        r.status.log(
            "GREEN",
            f"Green square pending action={_pending_turn_dir} ({_pending_turn_frames}/{GREEN_CONFIRM_FRAMES})",
            force=True,
        )
        return False

    _pending_turn_frames += 1
    if _pending_turn_frames < GREEN_CONFIRM_FRAMES:
        r.status.log(
            "GREEN",
            f"Green square pending action={_pending_turn_dir} ({_pending_turn_frames}/{GREEN_CONFIRM_FRAMES})",
            force=True,
        )
        return False

    _pending_turn_dir = "straight"
    _pending_turn_frames = 0
    _last_green_action_t = now

    r.status.log("GREEN", f"Green square action={turn_dir}", force=True)
    r.forward()
    if GUI:
        cv2.waitKey(1500)
    else:
        time.sleep(1.5)
    if turn_dir == "left":
        r.turn(-80)
    elif turn_dir == "right":
        r.turn(80)
    elif turn_dir == "turn_around":
        r.turn(180)
    return True
