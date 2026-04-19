import os
import time

os.environ["DISPLAY"] = ":0"  # Weird trick to put windows on the jetson

import cv2
import Jetson.GPIO as GPIO
from ultralytics import YOLO

from classes.Motor import Motor
from classes.Robot import Robot
from constants import (
    BALL_CAM_CAPTURE_FPS,
    BALL_CAM_CAPTURE_HEIGHT,
    BALL_CAM_CAPTURE_WIDTH,
    BALL_CAM_DEVICE,
    LINE_CAM_CAPTURE_FPS,
    LINE_CAM_CAPTURE_HEIGHT,
    LINE_CAM_CAPTURE_WIDTH,
    LINE_CAM_DEVICE,
    USE_GPU_DECODE,
    USE_GSTREAMER,
)

# Setup GPIO + initialize motors
# GPIO.setmode(GPIO.BOARD)
# l1: Motor = Motor(33, 37)
# l2: Motor = Motor(33, 18)
# r1: Motor = Motor(35, 22)
# r2: Motor = Motor(37, 24)

# try:
# l1.set_speed(75)
# l2.set_speed(100)
# r1.set_speed(100)
# r2.set_speed(100)
# print("Motors should have stopped.")
# time.sleep(3)

# l1.reverse()
# l2.reverse()
# r1.reverse()
# r2.reverse()
# time.sleep(3)
# except Exception as e:
#     print(f"Error occurred: {e}")
# finally:
#     del l1
#     GPIO.cleanup()

# Initialize models
ball_model = YOLO("./models/ball_detect_s.engine", task="detect")
silver_model = YOLO("./models/silver_classify_s.engine", task="classify")


def open_camera(device, width, height, fps, fallbacks):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if cap is not None and cap.isOpened():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        return cap

    resolved = os.path.realpath(device)
    if resolved and resolved != device:
        cap = cv2.VideoCapture(resolved, cv2.CAP_V4L2)
        if cap is not None and cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            return cap

    for dev in fallbacks:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap is not None and cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            return cap

    return None


# Initialize Camera
camera1 = open_camera(
    BALL_CAM_DEVICE,
    BALL_CAM_CAPTURE_WIDTH,
    BALL_CAM_CAPTURE_HEIGHT,
    BALL_CAM_CAPTURE_FPS,
    fallbacks=["/dev/video2", "/dev/video3", 2, 3],
)

camera2 = open_camera(
    LINE_CAM_DEVICE,
    LINE_CAM_CAPTURE_WIDTH,
    LINE_CAM_CAPTURE_HEIGHT,
    LINE_CAM_CAPTURE_FPS,
    fallbacks=["/dev/video0", "/dev/video1", 0, 1],
)

# Quit if camera isn't open
if camera1 is None or not camera1.isOpened():
    print("Camera is not opened")
    exit()

if camera2 is None or not camera2.isOpened():
    print("Camera2 is not opened")
    exit()

# Weird trick to detect window closing
ball_window = "YOLO Ball Inference"
silver_window = "YOLO Silver Inference"
cv2.namedWindow(ball_window, cv2.WINDOW_AUTOSIZE)
cv2.namedWindow(silver_window, cv2.WINDOW_AUTOSIZE)

r = Robot()


while True:
    # Fetch numpy frame from cam
    ret, frame = camera1.read()
    ret2, frame2 = camera2.read()

    r.set_relay(not r.relay_on)
    time.sleep(0.2)

    if not ret:
        print("Can't receive frame...")
        break

    if not ret2:
        print("Can't receive frame from camera 2")
        break

    if len(frame.shape) == 3 and frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if len(frame2.shape) == 3 and frame2.shape[2] == 4:
        frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGRA2BGR)

    frame = cv2.flip(frame, 0)
    frame2 = cv2.flip(frame2, 0)

    # Run inference on the frame
    ball_results = ball_model(frame)
    silver_results = silver_model(frame2)

    # Visualize results
    ball_annotated_frame = ball_results[0].plot()
    silver_annotated_frame = silver_results[0].plot()

    # Resize with cuda modules
    gpu_mat = cv2.cuda.GpuMat()

    gpu_mat.upload(ball_annotated_frame)
    resized_ball_mat = cv2.cuda.resize(gpu_mat, (960, 540))

    gpu_mat.upload(silver_annotated_frame)
    resized_line_mat = cv2.cuda.resize(gpu_mat, (960, 540))

    resized_ball = resized_ball_mat.download()
    resized_line = resized_line_mat.download()

    # Show results in window
    cv2.imshow(ball_window, resized_ball)
    cv2.imshow(silver_window, resized_line)

    # If q is pressed quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    # If window is closed quit
    if cv2.getWindowProperty(ball_window, cv2.WND_PROP_AUTOSIZE) < 1:
        break
    if cv2.getWindowProperty(silver_window, cv2.WND_PROP_AUTOSIZE) < 1:
        break

# When everyting is done release camera and close windows
camera1.release()
cv2.destroyAllWindows()
# GPIO.cleanup()
