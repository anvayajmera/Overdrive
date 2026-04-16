import os
import time


def main():
    os.environ["OVERDRIVE_DISABLE_CAMERAS"] = "1"
    from classes.Robot import Robot

    r = Robot()
    try:
        # Drive in a circle by setting left speed higher than right speed.
        left_speed = 60
        right_speed = 30
        print(f"Driving circle: left={left_speed} right={right_speed}")

        while True:
            # Keep sending commands so the ESP32 failsafe doesn't stop the motors.
            r.set_left_speed(left_speed)
            r.set_right_speed(right_speed)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stopping circle drive.")
    finally:
        try:
            r.stop()
        except Exception as e:
            print(f"Error stopping robot: {e}")
        finally:
            r.cleanup()


if __name__ == "__main__":
    main()
