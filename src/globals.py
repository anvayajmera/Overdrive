import cv2
import numpy as np

from constants import BLACK_THRESHOLD


def is_byte(num: int):
    return num >= 0 and num <= 255


# =========================
# Image Processing
# =========================
def process_image(img):
    blur = cv2.blur(img, (3, 3))

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return (closing, 0.0)

    largest = max(contours, key=cv2.contourArea)

    mask = np.zeros_like(closing)
    cv2.drawContours(mask, [largest], -1, 255, cv2.FILLED)  # type: ignore

    # Close small gaps/cracks before thinning so they don't create stray skeleton branches
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary_path = cv2.bitwise_and(closing, mask)
    binary_path = cv2.morphologyEx(binary_path, cv2.MORPH_CLOSE, kernel)

    return (binary_path, cv2.contourArea(largest))


def calc_linear_acc(acceleration):
    return np.sqrt(acceleration[0] ** 2 + acceleration[1] ** 2)
