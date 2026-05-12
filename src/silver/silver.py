import cv2
from ultralytics import YOLO

from classes.Robot import Robot
from constants import (
    ENABLE_SILVER,
    GUI,
    SILVER_MODEL_PATH,
    SILVER_THRESHOLD,
)

_silver_model = None


def silver_init():
    """
    Initializes the silver classification model.
    Loads the model into a global variable if it hasn't been loaded already.
    """
    global _silver_model
    if _silver_model is None:
        try:
            print(f"Loading silver classification model from {SILVER_MODEL_PATH}...")
            _silver_model = YOLO(SILVER_MODEL_PATH, task="classify")
        except Exception as e:
            print(f"Failed to load silver model: {e}")


def silver() -> bool:
    """
    Checks the current camera frame for a silver line representing a rescue zone marker.

    Returns:
        bool: True if a silver line is detected with confidence >= SILVER_THRESHOLD,
              False otherwise.
    """
    if not ENABLE_SILVER:
        return False

    r = Robot()

    # Ensure we have a valid frame to process
    if r.frame is None or not hasattr(r.frame, "shape") or r.frame.size == 0:
        return False

    # Ensure model is initialized
    silver_init()
    if _silver_model is None:
        return False

    # Perform inference on the current line camera frame
    # verbose=False prevents YOLO from flooding the console with inference times
    results = _silver_model(r.frame, verbose=False)
    result = results[0]

    # Find the specific confidence for the "silver" class
    silver_conf = 0.0
    for idx, name in result.names.items():
        if name.lower() == "silver":
            # result.probs.data is the tensor of class probabilities
            silver_conf = float(result.probs.data[idx])
            break

    is_detected = silver_conf >= SILVER_THRESHOLD

    # Visualization for debugging
    if GUI:
        # result.plot() provides the standard YOLO classification overlay
        debug_frame = result.plot()

        # Add custom status and confidence overlay
        color = (0, 255, 0) if is_detected else (0, 0, 255)
        status_text = "DETECTED" if is_detected else "NONE"
        label = f"SILVER: {silver_conf:.2f} [{status_text}]"

        # Draw a black background for the text to make it readable
        cv2.rectangle(debug_frame, (10, 10), (450, 60), (0, 0, 0), -1)
        cv2.putText(
            debug_frame,
            label,
            (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            color,
            2,
        )

        cv2.imshow("Silver Detection Debug", debug_frame)

    if is_detected:
        r.status.log(
            "SILVER", f"Silver line detected! Confidence: {silver_conf:.2f}", force=True
        )
        return True

    return False
