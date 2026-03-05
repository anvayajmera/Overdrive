from .SerialManager import SerialManager
from .Motor import Motor

from simple_pid import PID
from adafruit_bno055 import BNO055_I2C
from .constants import TIMESTEP, M_KD, M_KI, M_KP, BASE_SPEED, MAX_SPEED, FPS, MOTORS

import cv2, board, time, threading
import Jetson.GPIO as GPIO


class Robot:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        board_to_tegra = {k: list(GPIO.gpio_pin_data.get_data()[-1]["TEGRA_SOC"].keys())[i] for i, k in enumerate(GPIO.gpio_pin_data.get_data()[-1]["BOARD"])}  # type: ignore

        self._initialized = True
        self.sm = SerialManager()
        self.motors = [Motor(i, self.sm) for i in range(4)]
        self.motors_active = MOTORS

        self.ball_cam = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.line_cam = cv2.VideoCapture(0, cv2.CAP_V4L2)

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
        # self.IMU = BNO055_I2C(i2c)

        self.yaw: float = 0.0

        self.m_pid = PID(M_KP, M_KI, M_KD, setpoint=0.0)

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

    def turn(self, degrees: float, tol=1.0):
        target: float = (
            self.yaw + degrees
        ) % 360  # pyright: ignore[reportOptionalOperand]
        self.stop()
        time.sleep(TIMESTEP)
        while abs(target - self.yaw) > tol:
            self.update()
            if degrees > 0:
                self.turnRight()
            else:
                self.turnLeft()
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
        self.ball_cam.release()
        self.line_cam.release()

    def update(self):
        self.yaw = self.IMU.gyro[2]  # type: ignore

    def pause_motors(self, seconds: float):
        if not self.motors_active:
            return

        self.stop()

        self.motors_active = False

        def re_enable():
            self.motors_active = MOTORS

        threading.Timer(seconds, re_enable).start()
