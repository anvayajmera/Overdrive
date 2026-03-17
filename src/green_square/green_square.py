import cv2
import numpy as np
from typing_extensions import Sequence

from src.constants import GREEN_MAX, GREEN_MIN, GREEN_SQUARE_MIN_AREA, GUI, LINE_CAM_HEIGHT, LINE_CAM_WIDTH, GREEN_SQUARE_ROI


def find_green_square(frame: np.ndarray) -> Sequence[np.ndarray]:
    kernal = np.ones((3, 3), np.uint8)

    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv_image, np.array(GREEN_MIN), np.array(GREEN_MAX))

    green_image = cv2.erode(green_image, kernal, iterations=1)
    green_image = cv2.dilate(green_image, kernal, iterations=11)
    green_image = cv2.erode(green_image, kernal, iterations=9)

    contours_grn, _ = cv2.findContours(
        green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    contours_grn = [
        contour
        for contour in contours_grn
        if cv2.contourArea(contour) > GREEN_SQUARE_MIN_AREA 
    ]

    return contours_grn


def check_black(green_contour: np.ndarray, black_image: np.ndarray, gui_frame: np.ndarray) -> np.ndarray:
    green_rect = cv2.boxPoints(cv2.minAreaRect(green_contour))
    green_rect = green_rect[green_rect[:, 1].argsort()] # sort by y val
    
    if GUI:
        draw_rect = np.intp(green_rect)
        cv2.drawContours(gui_frame, [draw_rect], -1, (0, 0, 255), 2) # type:ignore 
        
    marker_height = green_rect[-1][1] - green_rect[0][1] # greatest y - least y 

    black_around_sign = np.zeros(6, dtype=np.int16)
    black_around_sign[4] = int(green_rect[2][1])
    black_around_sign[5] = int(green_rect[0][1])

    # Bottom
    roi_b = black_image[int(green_rect[2][1]):np.minimum(int(green_rect[2][1] + (marker_height * 0.8)), LINE_CAM_HEIGHT), np.minimum(int(green_rect[2][0]), int(green_rect[3][0])):np.maximum(int(green_rect[2][0]), int(green_rect[3][0]))]
    if roi_b.size > 0:
        if np.mean(roi_b[:]) > 125:
            black_around_sign[0] = 1

    # Top
    roi_t = black_image[np.maximum(int(green_rect[1][1] - (marker_height * 0.8)), 0):int(green_rect[1][1]), np.minimum(np.maximum(int(green_rect[0][0]), 0), np.maximum(int(green_rect[1][0]), 0)):np.maximum(np.maximum(int(green_rect[0][0]), 0), np.maximum(int(green_rect[1][0]), 0))]
    if roi_t.size > 0:
        if np.mean(roi_t[:]) > 125:
            black_around_sign[1] = 1

    green_rect = green_rect[green_rect[:, 0].argsort()]

    # Left
    roi_l = black_image[np.minimum(int(green_rect[0][1]), int(green_rect[1][1])):np.maximum(int(green_rect[0][1]), int(green_rect[1][1])), np.maximum(int(green_rect[1][0] - (marker_height * 0.8)), 0):int(green_rect[1][0])]
    if roi_l.size > 0:
        if np.mean(roi_l[:]) > 125:
            black_around_sign[2] = 1

    # Right
    roi_r = black_image[np.minimum(int(green_rect[2][1]), int(green_rect[3][1])):np.maximum(int(green_rect[2][1]), int(green_rect[3][1])), int(green_rect[2][0]):np.minimum(int(green_rect[2][0] + (marker_height * 0.8)), LINE_CAM_WIDTH)]
    if roi_r.size > 0:
        if np.mean(roi_r[:]) > 125:
            black_around_sign[3] = 1

    return black_around_sign
    
def determine_turn_direction(black_borders):
    turn_left = False
    turn_right = False
    left_low = False
    right_low = False

    for i in black_borders:
        if np.sum(i[:4]) == 2:
            if i[1] == 1 and i[2] == 1:
                turn_right = True
                if i[4] > LINE_CAM_HEIGHT * 0.95:
                    right_low = True
            elif i[1] == 1 and i[3] == 1:
                turn_left = True
                if i[4] > LINE_CAM_HEIGHT * 0.95:
                    left_low = True

    return turn_left, turn_right, left_low, right_low

def green_square(frame: np.ndarray, black_frame: np.ndarray, gui_frame: np.ndarray) -> str:
    contours_grn = find_green_square(frame)
    
    black_borders = []
    
    for contour in contours_grn:
        black_around_sign = check_black(contour, black_frame, gui_frame)
        if black_around_sign[5] < LINE_CAM_HEIGHT * GREEN_SQUARE_ROI:
            continue
        black_borders.append(black_around_sign)
    
    turn_left, turn_right, left_bottom, right_bottom = determine_turn_direction(black_borders)
    
    if turn_left and not turn_right and not left_bottom:
        return "left"
    elif turn_right and not turn_left and not right_bottom:
        return "right"
    elif turn_left and turn_right and not (left_bottom and right_bottom):
        return "turn_around"
    else:
        return "straight"
    
    
    
        
    
