import os, time
os.environ["DISPLAY"] = ":0" # Weird trick to put windows on the jetson

import cv2
import Jetson.GPIO as GPIO
from ultralytics import YOLO
from motor import Motor

# Setup GPIO + initialize motors
GPIO.setmode(GPIO.BCM)
l1: Motor = Motor(33, 15)
# l2: Motor = Motor(33, 18)
# r1: Motor = Motor(35, 22)
# r2: Motor = Motor(37, 24)

try:
    l1.set_speed(100)
    # l2.set_speed(100)
    # r1.set_speed(100)
    # r2.set_speed(100)

    time.sleep(3)

    l1.reverse()
    # l2.reverse()
    # r1.reverse()
    # r2.reverse()
    time.sleep(3)

finally:
    del l1
    GPIO.cleanup()

# # Initialize models
# ball_model = YOLO("./models/ball_detect_s.engine", task="detect")
# silver_model = YOLO("./models/silver_classify_s.engine", task="classify")

# # Initialize Camera
# camera1 = cv2.VideoCapture(0, cv2.CAP_V4L2)
# camera1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*'MJPG'))
# camera1.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
# camera1.set(cv2.CAP_PROP_FPS, 60)

# # Quit if camera isn't open
# if not camera1.isOpened():
#     print("Camera is not opened")
#     exit()
#
# # Weird trick to detect window closing
# ball_window = "YOLO Ball Inference"
# silver_window = "YOLO Silver Inference"
# cv2.namedWindow(ball_window, cv2.WINDOW_AUTOSIZE)
# cv2.namedWindow(silver_window, cv2.WINDOW_AUTOSIZE)
#
# while True:
#     # Fetch numpy frame from cam
#     ret, frame = camera1.read()
#
#     if not ret:
#         print("Can't receive frame...")
#         break
#
#     # Run inference on the frame
#     ball_results = ball_model(frame)
#     silver_results = silver_model(frame)
#
#     # Visualize results
#     ball_annotated_frame = ball_results[0].plot()
#     silver_annotated_frame = silver_results[0].plot()
#
#     # Resize with cuda modules
#     gpu_mat = cv2.cuda.GpuMat()
#
#     gpu_mat.upload(ball_annotated_frame)
#     resized_ball_mat = cv2.cuda.resize(gpu_mat, (960, 540))
#
#     gpu_mat.upload(silver_annotated_frame)
#     resized_line_mat = cv2.cuda.resize(gpu_mat, (960, 540))
#
#     resized_ball = resized_ball_mat.download()
#     resized_line = resized_line_mat.download()
#
#     # Show results in window
#     cv2.imshow(ball_window, resized_ball)
#     cv2.imshow(silver_window, resized_line)
#
#     # If q is pressed quit
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#     # If window is closed quit
#     if cv2.getWindowProperty(ball_window, cv2.WND_PROP_AUTOSIZE) < 1:
#         break
#     if cv2.getWindowProperty(silver_window, cv2.WND_PROP_AUTOSIZE) < 1:
#         break

# When everyting is done release camera and close windows
# camera1.release()
# cv2.destroyAllWindows()
# GPIO.cleanup()