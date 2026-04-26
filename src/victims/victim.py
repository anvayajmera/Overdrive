import time

import cv2
import numpy as np

from classes.Robot import Robot
from constants import (
    BALL_CAM_CAPTURE_HEIGHT,
    BALL_CAM_CAPTURE_WIDTH,
    BASE_SPEED,
    GUI,
    RED_MAX,
    RED_MAX_2,
    RED_MIN,
    RED_MIN_2,
    RED_VICTIM_THRESH,
)
from green_square.green_square import find_green_contour

ball_window = "Ball Camera"


def victim_init():
    if GUI:
        cv2.namedWindow(ball_window, cv2.WINDOW_AUTOSIZE)


# Pre-allocate numpy arrays for performance to avoid creating them every frame
_RED_MIN_ARR = np.array(RED_MIN, dtype=np.uint8)
_RED_MAX_ARR = np.array(RED_MAX, dtype=np.uint8)
_RED_MIN_ARR_2 = np.array(RED_MIN_2, dtype=np.uint8)
_RED_MAX_ARR_2 = np.array(RED_MAX_2, dtype=np.uint8)

# Pre-compute morphological kernels
# Using cv2.getStructuringElement is generally equivalent to np.ones for MORPH_RECT
_KERNEL_3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
# 11 iterations of 3x3 is equivalent to 1 iteration of 23x23
_KERNEL_23 = cv2.getStructuringElement(cv2.MORPH_RECT, (23, 23))
# 9 iterations of 3x3 is equivalent to 1 iteration of 19x19
_KERNEL_19 = cv2.getStructuringElement(cv2.MORPH_RECT, (19, 19))


def find_red_contour(frame: np.ndarray) -> tuple[np.ndarray | None, float, int]:
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_image, _RED_MIN_ARR, _RED_MAX_ARR)
    mask2 = cv2.inRange(hsv_image, _RED_MIN_ARR_2, _RED_MAX_ARR_2)
    red_image = cv2.bitwise_or(mask1, mask2)

    # Morphological operations optimized: using larger kernels instead of many iterations.
    # This drastically reduces memory read/write overhead while achieving the exact same mathematical effect.
    red_image = cv2.erode(red_image, _KERNEL_3, iterations=1)
    red_image = cv2.dilate(red_image, _KERNEL_23, iterations=1)
    red_image = cv2.erode(red_image, _KERNEL_19, iterations=1)

    contours_red, _ = cv2.findContours(
        red_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    if GUI:
        frame_copy = frame.copy()
        cv2.drawContours(frame_copy, contours_red, -1, (0, 0, 255), 2)
        cv2.imshow("Red Contours", frame_copy)
        cv2.waitKey(1)

    if not contours_red:
        return None, 0.0, 0

    contours_red = list(contours_red)

    contours_red.sort(key=cv2.contourArea, reverse=True)

    best_contour = contours_red[0]
    area = cv2.contourArea(best_contour)

    M = cv2.moments(best_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
    else:
        cx = 0

    return best_contour, area, cx


def turnToFaceRed():
    r = Robot()

    red_contour, area, cx = find_red_contour(r.ball_frame)

    # 1. Spin until we see a red object large enough
    while area < RED_VICTIM_THRESH:
        r.update()
        r.turnLeft()
        red_contour, area, cx = find_red_contour(r.ball_frame)

    r.stop()

    # 2. Turn until the center of the contour matches the center of the screen
    target_x = BALL_CAM_CAPTURE_WIDTH // 2
    tolerance = 5  # pixel tolerance for center alignment

    while True:
        r.update()
        red_contour, area, cx = find_red_contour(r.ball_frame)

        if area < RED_VICTIM_THRESH:
            # We lost it, stop or continue searching
            r.stop()
            break

        error = cx - target_x

        if abs(error) <= tolerance:
            r.stop()
            break

        if error > 0:
            r.turnLeft()
        else:
            # Target is to the left of the center, turn left
            r.turnRight()


def victim():
    r = Robot()
    # frame2 = cv2.flip(frame2, 0)
    #
    if r.ball_count == 3:
        # First look for red
        turnToFaceRed()

        if GUI:
            cv2.waitKey(500)
        else:
            time.sleep(500)

        while not r.obs_detected:
            r.update()
            r.forward()

        r.stop()
        r.backward()

        if GUI:
            cv2.waitKey(500)
        else:
            time.sleep(500)

        r.turn(180)

        if GUI:
            cv2.waitKey(500)
        else:
            time.sleep(500)

        r.backward()

        if GUI:
            cv2.waitKey(600)
        else:
            time.sleep(600)

        r.stop()

        r.set_left_speed(-BASE_SPEED)
        if GUI:
            cv2.waitKey(600)
        else:
            time.sleep(600)

        r.stop()

        r.set_right_speed(-BASE_SPEED)

        if GUI:
            cv2.waitKey(600)
        else:
            time.sleep(600)

        r.stop()

        time.sleep(100)

        pass

    # # Run inference on the frame
    # ball_results = r.ball_model(r.ball_frame)
    # ball_result = ball_results[0]
    # # silver_results = silver_model(frame2)

    # # Visualize results
    # ball_annotated_frame = ball_results[0].plot()

    # for box in ball_result.boxes:
    #     # 1. Get the class ID and name
    #     class_id = int(box.cls[0])  # The numerical class ID (e.g., 0, 1)
    #     class_name = ball_result.names[
    #         class_id
    #     ]  # Maps ID to the string name (e.g., "orange_ball")

    #     # 2. Get the confidence score
    #     confidence = float(box.conf[0])  # A float between 0 and 1

    #     # 3. Get the bounding box coordinates
    #     # .xyxy gives you [x_min, y_min, x_max, y_max]
    #     x1, y1, x2, y2 = map(int, box.xyxy[0])

    #     if GUI:
    #         cv2.line(
    #             ball_annotated_frame,
    #             ((x1 + x2) // 2, y1),
    #             ((x1 + x2) // 2, y2),
    #             (0, 255, 0),
    #             2,
    #         )

    # # silver_annotated_frame = silver_results[0].plot()
    # #
    # if GUI:
    #     cv2.line(
    #         ball_annotated_frame,
    #         (BALL_CAM_CAPTURE_WIDTH // 2, 0),
    #         (BALL_CAM_CAPTURE_WIDTH // 2, BALL_CAM_CAPTURE_HEIGHT),
    #         (0, 255, 0),
    #         2,
    #     )

    #     cv2.imshow(ball_window, ball_annotated_frame)
