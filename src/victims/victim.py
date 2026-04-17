import cv2

from classes.Robot import Robot
from constants import BALL_CAM_CAPTURE_HEIGHT, BALL_CAM_CAPTURE_WIDTH, GUI

ball_window = "Ball Camera"


def victim_init():
    if GUI:
        cv2.namedWindow(ball_window, cv2.WINDOW_AUTOSIZE)


def victim():
    r = Robot()

    if not r.ball_cam:
        print("Ball camera not opened")
        return

    ret, frame = r.ball_cam.read()
    # ret2, frame2 = camera2.read()

    if not ret:
        print("Can't receive frame...")
        return

    # if not ret2:
    #     print("Can't receive frame from camera 2")
    #     break

    if len(frame.shape) == 3 and frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    # if len(frame2.shape) == 3 and frame2.shape[2] == 4:
    #     frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGRA2BGR)

    frame = cv2.flip(frame, 0)
    # frame2 = cv2.flip(frame2, 0)

    # Run inference on the frame
    ball_results = r.ball_model(frame)
    ball_result = ball_results[0]
    # silver_results = silver_model(frame2)

    # Visualize results
    ball_annotated_frame = ball_results[0].plot()

    for box in ball_result.boxes:
        # 1. Get the class ID and name
        class_id = int(box.cls[0])  # The numerical class ID (e.g., 0, 1)
        class_name = ball_result.names[
            class_id
        ]  # Maps ID to the string name (e.g., "orange_ball")

        # 2. Get the confidence score
        confidence = float(box.conf[0])  # A float between 0 and 1

        # 3. Get the bounding box coordinates
        # .xyxy gives you [x_min, y_min, x_max, y_max]
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        if GUI:
            cv2.line(
                ball_annotated_frame,
                ((x1 + x2) // 2, y1),
                ((x1 + x2) // 2, y2),
                (0, 255, 0),
                2,
            )

    # silver_annotated_frame = silver_results[0].plot()
    #
    if GUI:
        cv2.line(
            ball_annotated_frame,
            (BALL_CAM_CAPTURE_WIDTH // 2, 0),
            (BALL_CAM_CAPTURE_WIDTH // 2, BALL_CAM_CAPTURE_HEIGHT),
            (0, 255, 0),
            2,
        )

        cv2.imshow(ball_window, ball_annotated_frame)
