import numpy as np
import cv2
from jetcam.csi_camera import CSICamera
from ultralytics import YOLO

# Initialize models
ball_model = YOLO("./models/ball_detect_s.engine", task="detect")
silver_model = YOLO("./models/silver_classify_s.engine", task="classify")

# Initialize Camera
camera1 = CSICamera(width=960, height=540, capture_width=1920, capture_height=1080, capture_fps=30, device=1)

# Start camera thread
camera1.running = True

# Weird trick to detect window closing
ball_window = "YOLO Ball Inference"
silver_window = "YOLO Silver Inference"
cv2.namedWindow(ball_window, cv2.WINDOW_AUTOSIZE)
cv2.namedWindow(silver_window, cv2.WINDOW_AUTOSIZE)

while camera1.running:
    # Fetch numpy frame from cam
    frame = camera1.value

    frame = frame[:, :, :3]

    # Run inference on the frame
    ball_results = ball_model(frame)
    silver_results = silver_model(frame)

    # Visualize results
    ball_annotated_frame = ball_results[0].plot()
    silver_annotated_frame = silver_results[0].plot()

    # Show results in window
    cv2.imshow(ball_window, ball_annotated_frame)
    cv2.imshow(silver_window, silver_annotated_frame)

    # If q is pressed quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # If window is closed quit
    if cv2.getWindowProperty(ball_window, cv2.WND_PROP_AUTOSIZE) < 1:
        break
    if cv2.getWindowProperty(silver_window, cv2.WND_PROP_AUTOSIZE) < 1:
        break

camera1.running = False
cv2.destroyAllWindows()