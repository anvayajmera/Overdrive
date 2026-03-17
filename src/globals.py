import cv2, numpy as np

def is_byte(num: int):
    return num >= 0 and num <= 255

# =========================
# Image Processing
# =========================
def process_image(img):
    blur = cv2.blur(img, (3, 3))
    f = blur.astype(np.float32) + 1
    log_img = cv2.log(f)
    log_img = cv2.normalize(log_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)  # type: ignore

    # Faster thresholding
    b, g, r = cv2.split(log_img)
    _, b_bin = cv2.threshold(b, 200, 255, cv2.THRESH_BINARY_INV)
    _, g_bin = cv2.threshold(g, 200, 255, cv2.THRESH_BINARY_INV)
    _, r_bin = cv2.threshold(r, 200, 255, cv2.THRESH_BINARY_INV)

    binary = cv2.bitwise_and(cv2.bitwise_and(b_bin, g_bin), r_bin)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return closing

    largest = max(contours, key=cv2.contourArea)
    mask = np.zeros_like(closing)
    cv2.drawContours(mask, [largest], -1, 255, cv2.FILLED)  # type: ignore

    # Remove edge artifacts by blacking out a small border
    # This prevents the skeleton from branching or flattening at the frame edges
    h, w = mask.shape
    cv2.rectangle(mask, (0, 0), (w - 1, h - 1), 0, 5)

    return cv2.bitwise_and(closing, mask)

