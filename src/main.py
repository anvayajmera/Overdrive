import numpy as np
import cv2
from jetcam.csi_camera import CSICamera
from ultralytics import YOLO

# Initialize model
ball_model = YOLO("../models/ball_detect_s.pt")

# Initialize Camera
camera1 = CSICamera(width=960, height=540, capture_width=1920, capture_height=1080, capture_fps=30)

# Start camera thread
camera1.running = True

# Weird trick to detect window closing
window_name = "YOLO Ball Inference"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

while camera1.running:
    # Fetch numpy frame from cam
    frame = camera1.value

    # Run inference on the frame
    results = ball_model(frame)

    # Visualize results
    annotated_frame = results[0].plot()

    # Show results in window
    cv2.imshow(window_name, annotated_frame)

    # If q is pressed quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # If window is closed quit
    if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
        break

camera1.running = False
cv2.destroyAllWindows()