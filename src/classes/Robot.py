import threading
import time
from typing import List

import board
import cv2
import Jetson.GPIO as GPIO
import numpy as np
from adafruit_bno055 import BNO055_I2C
from adafruit_tca9548a import TCA9548A
from adafruit_vl53l4cd import VL53L4CD
from simple_pid import PID

from constants import (
    BASE_SPEED,
    FPS,
    FRONT_THRESH,
    GUI,
    M_KD,
    M_KI,
    M_KP,
    MAX_SPEED,
    MOTORS,
    TIMESTEP,
)
from globals import process_image

from .Motor import Motor
from .SerialManager import SerialManager


class Robot:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        board_to_tegra = {
            k: list(GPIO.gpio_pin_data.get_data()[-1]["TEGRA_SOC"].keys())[i]  # type:ignore
            for i, k in enumerate(GPIO.gpio_pin_data.get_data()[-1]["BOARD"])  # type:ignore
        }  # type: ignore

        self._initialized = True
        self.sm = SerialManager()
        self.motors = [Motor(i, self.sm) for i in range(4)]
        self.motors_active = MOTORS

        # Use absolute hardware paths so camera indices never swap again!
        import os

        ball_path = "/dev/v4l/by-id/usb-16MP_Camera_Mamufacture_16MP_USB_Camera_2022050701-video-index0"
        if os.path.exists(ball_path):
            self.ball_cam = cv2.VideoCapture(ball_path, cv2.CAP_V4L2)
        else:
            print(f"WARNING: {ball_path} NOT FOUND! Falling back to 0")
            self.ball_cam = cv2.VideoCapture(0, cv2.CAP_V4L2)

        line_path = (
            "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_2020042001-video-index0"
        )
        if os.path.exists(line_path):
            self.line_cam = cv2.VideoCapture(line_path, cv2.CAP_V4L2)
        else:
            print(f"WARNING: {line_path} NOT FOUND! Falling back to 2")
            self.line_cam = cv2.VideoCapture(2, cv2.CAP_V4L2)

        self.ball_cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        self.ball_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.ball_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.ball_cam.set(cv2.CAP_PROP_FPS, FPS)

        self.line_cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        self.line_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.line_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)
        self.line_cam.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.line_cam.set(cv2.CAP_PROP_FPS, FPS)

        self.prev_steer = 0.0

        i2c = board.I2C()

        self.mux = TCA9548A(i2c)
        self.IMU = BNO055_I2C(self.mux[7])

        self.yaw: float = 0.0

        self.m_pid = PID(M_KP, M_KI, M_KD, setpoint=0.0)

        # Facing the robot, 0 is Left front and 3 is right front, 7 is left, and 4 is right
        self.distance_ids = [0, 3, 4, 7]
        self.front_ids = [0, 3]
        self.side_ids = [7, 4]

        self.distance_sensors: List[VL53L4CD] = []

        self.frame: np.ndarray = []  # type: ignore
        self.gui_frame: np.ndarray = []  # type: ignore
        self.binary_frame: np.ndarray = []  # type: ignore
        self.line_size: float = 0.0

        self.front_distances: List[float] = [1000000] * len(self.front_ids)
        self.side_distances: List[float] = [1000000] * len(self.side_ids)
        for ch in self.distance_ids:
            vl53 = VL53L4CD(self.mux[ch])  # type: ignore
            vl53.timing_budget = 200
            vl53.inter_measurement = 0
            vl53.start_ranging()
            self.distance_sensors.append(vl53)

        self.obs_detected = False

    def set_left_speed(self, speed: int):
        if not self.motors_active:
            return
        self.motors[0].set_speed(speed)
        self.motors[1].set_speed(speed)

    def set_right_speed(self, speed: int):
        if not self.motors_active:
            return
        self.motors[2].set_speed(speed)
        self.motors[3].set_speed(speed)

    # Control represents some value to be passed into pid
    def set_motor_output(self, control: int):
        output = int(self.m_pid(control))  # type: ignore
        # if abs(control) > 250:
        # print(f"Control: {control} -> Output: {output}")

        self.set_left_speed(BASE_SPEED + output)
        self.set_right_speed(BASE_SPEED - output)

    def forward(self):
        self.set_left_speed(MAX_SPEED)
        self.set_right_speed(MAX_SPEED)

    def backward(self):
        self.set_left_speed(-MAX_SPEED)
        self.set_right_speed(-MAX_SPEED)

    def turnLeft(self):
        self.set_left_speed(MAX_SPEED)
        self.set_right_speed(-MAX_SPEED)

    def turnRight(self):
        self.set_left_speed(-MAX_SPEED)
        self.set_right_speed(MAX_SPEED)

    def update_gui(self):

        if GUI:
            cv2.putText(
                self.gui_frame,
                f"Left Sensor: {self.front_distances[0]}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                self.gui_frame,
                f"Right Sensor: {self.front_distances[1]}",
                (400, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                self.gui_frame,
                f"Obstacle Detected: {self.obs_detected}",
                (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                self.gui_frame,
                f"Yaw: {self.yaw}",
                (20, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Sensor Debug", self.gui_frame)

    def turn(self, degrees: float, tol=1.0):
        target: float = (self.yaw + degrees) % 360  # pyright: ignore[reportOptionalOperand]
        self.stop()
        time.sleep(TIMESTEP)
        while True:
            self.update()
            if GUI:
                cv2.waitKey(1)

            diff = (target - self.yaw + 180.0) % 360.0 - 180.0
            # print(
            #     f"Turn debug -> Target: {target:.1f}, Yaw: {self.yaw:.1f}, Diff: {diff:.1f}"
            # )
            if abs(diff) <= tol:
                break

            speed = max(70, min(MAX_SPEED, int(abs(diff) * 1.5)))

            if diff > 0:
                self.set_left_speed(-speed)
                self.set_right_speed(speed)
            else:
                self.set_left_speed(speed)
                self.set_right_speed(-speed)
            time.sleep(TIMESTEP)

        self.stop()
        time.sleep(TIMESTEP)

    def turnToTarget(self, degrees: float, tol=1.0):
        angle: float = (degrees - self.yaw + 180.0) % 360.0 - 180.0
        self.turn(angle, tol)

    def stop(self):
        self.set_left_speed(0)
        self.set_right_speed(0)

    def cleanup(self):
        try:
            self.ball_cam.release()
            self.line_cam.release()
        except Exception as e:
            print(f"Error in cleanup: {e}")
        finally:
            self.line_cam.release()

    def update(self):
        if not hasattr(self, "_last_imu_yaw"):
            self._last_imu_yaw = -999.0
            self._imu_freeze_frames = 0

        try:
            yaw = self.IMU.euler[0]  # type: ignore
            if yaw is not None:
                # The BNO055 outputs floats like 0.625. If it's literally identical, it's frozen.
                if abs(yaw - self._last_imu_yaw) < 0.0001:
                    self._imu_freeze_frames += 1
                else:
                    self._imu_freeze_frames = 0
                    self._last_imu_yaw = yaw
                self.yaw = yaw
            else:
                self._imu_freeze_frames += 1
        except Exception as e:
            self._imu_freeze_frames += 1

        motors_spinning = any(
            abs(m.speed) > 20 and m.speed != -2000 for m in self.motors
        )

        # If motors have been actively spinning but IMU hasn't changed for 30 frames (0.5 sec)
        if motors_spinning and self._imu_freeze_frames > 30:
            print(
                f"!!! IMU CRASH DETECTED (stuck at {self.yaw:.1f}) !!! REBOOTING BNO055 !!!"
            )
            try:
                import time

                # Turn off motors to recover battery voltage
                self.set_left_speed(0)
                self.set_right_speed(0)
                time.sleep(0.1)
                # Force hardware reset via register 0x3F (SYS_TRIGGER) bit 5
                self.IMU._reset()
                time.sleep(0.1)
                self.IMU.mode = 0x0C  # NDOF mode
                print("!!! IMU REBOOTED SUCCESSFULLY !!!")
            except Exception as err:
                print(f"IMU Reboot Failed: {err}")
            self._imu_freeze_frames = 0

        # # print("update: reading cam")
        ret, frame = self.line_cam.read()
        if not ret:
            # # print("update: cam failed, returning")
            return

        self.frame = cv2.flip(frame, -1)
        self.gui_frame = self.frame.copy()
        # # print("update: processing frame")
        self.binary_frame, self.line_size = process_image(self.frame)

        # # print("update: reading vl53l4cd")
        for idx, sensor in enumerate(self.distance_sensors):
            try:
                # print(f"update: checking sensor {idx} data_ready")
                if sensor.data_ready:
                    # print(f"update: clearing sensor {idx} interrupt")
                    sensor.clear_interrupt()

                    sensor_id = self.distance_ids[idx]
                    if sensor_id in self.front_ids:
                        # print(f"update: reading sensor {idx} distance")
                        dist = sensor.distance
                        self.front_distances[self.front_ids.index(sensor_id)] = dist
                    elif sensor_id in self.side_ids:
                        # print(f"update: reading sensor {idx} distance")
                        dist = sensor.distance
                        self.side_distances[self.side_ids.index(sensor_id)] = dist
            except OSError:
                # print(f"update: OSError on sensor {idx}")
                continue
            except Exception as e:
                print(f"update: Unknown Exception on sensor {idx}: {e}")
                continue

        self.obs_detected = any(d < FRONT_THRESH for d in self.front_distances)

        if GUI:
            self.update_gui()
        # # print("update: finished")

    def pause_motors(self, seconds: float):
        if not self.motors_active:
            return

        self.stop()

        self.motors_active = False

        def re_enable():
            self.motors_active = MOTORS

        threading.Timer(seconds, re_enable).start()
