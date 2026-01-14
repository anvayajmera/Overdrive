import cv2
import numpy as np

# =========================================================
# BLACK LINE OFFSET + VISUALIZATION
# =========================================================
def get_black_line_offset(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    b, g, r = cv2.split(blurred)

    debug_mask = np.logical_and.reduce((b < 50, g < 50, r < 50)).astype(np.uint8) * 255

    contours, _ = cv2.findContours(
        debug_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return 0, debug_mask, None

    largest_contour = None
    max_area = 1000.0

    for c in contours:
        area = cv2.contourArea(c)
        if area > max_area:
            max_area = area
            largest_contour = c

    if largest_contour is None:
        return 0, debug_mask, None

    cv2.drawContours(frame, [largest_contour], -1, (0, 0, 255), 2)

    m = cv2.moments(largest_contour)
    if m["m00"] == 0:
        return 0, debug_mask, None

    line_center = (
        int(m["m10"] / m["m00"]),
        int(m["m01"] / m["m00"])
    )

    cv2.circle(frame, line_center, 6, (0, 0, 255), -1)

    center_x = frame.shape[1] // 2
    cv2.line(
        frame,
        (center_x, 0),
        (center_x, frame.shape[0]),
        (255, 0, 0),
        2
    )

    return line_center[0] - center_x, debug_mask, line_center


# =========================================================
# MAIN
# =========================================================
def linetrace():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera error")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Match your original C++ behavior
        frame = cv2.flip(frame, -1)

        offset, black_mask, _ = get_black_line_offset(frame)

        cv2.putText(
            frame,
            f"Offset: {offset}",
            (20, frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        cv2.imshow("Camera + Overlays", frame)
        cv2.imshow("Black Mask", black_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    linetrace()
