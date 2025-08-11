import numpy as np
import cv2
from jetcam.csi_camera import CSICamera

camera1 = CSICamera(width=960, height=540, capture_width=1920, capture_height=1080, capture_fps=30)

camera1.running = True

window_name = "JetCam Video"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

while camera1.running:

    # fetch numpy frame from cam
    frame = camera1.value

    cv2.imshow(window_name, frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
        break

camera1.running = False
cv2.destroyAllWindows()