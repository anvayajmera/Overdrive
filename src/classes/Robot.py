import threading
import time
from typing import List, Optional

import board
import cv2
import numpy as np
from adafruit_bno055 import BNO055_I2C
from adafruit_tca9548a import TCA9548A
from adafruit_vl53l4cd import VL53L4CD
from simple_pid import PID
from ultralytics import YOLO

from constants import (
    BALL_CAM_CAPTURE_FPS,
    BALL_CAM_CAPTURE_HEIGHT,
    BALL_CAM_CAPTURE_WIDTH,
    BALL_CAM_DEVICE,
    BASE_SPEED,
    DISABLE_CAMERAS,
    DISABLE_LINE_CAM,
    ENABLE_BALL_CAM,
    FPS,
    FRONT_OBS_ENTER_THRESH,
    FRONT_OBS_EXIT_THRESH,
    GUI,
    IMU_AUTO_RESET_ENABLED,
    LINE_CAM_BUFFERSIZE,
    LINE_CAM_CAPTURE_FPS,
    LINE_CAM_CAPTURE_HEIGHT,
    LINE_CAM_CAPTURE_WIDTH,
    LINE_CAM_DEVICE,
    LINE_CAM_HEIGHT,
    LINE_CAM_WIDTH,
    LOW_FPS_WARN_THRESHOLD,
    M_KD,
    M_KI,
    M_KP,
    MAX_SPEED,
    MIN_WHEEL_SPEED,
    MOTORS,
    OLED_ADDR,
    OLED_HEIGHT,
    OLED_MUX_CHANNEL,
    OLED_WIDTH,
    RELAY_ACTIVE_LOW,
    RELAY_DEFAULT_ON,
    SERVO_ARM,
    SERVO_CLAW_LEFT,
    SERVO_CLAW_RIGHT,
    SERVO_COUNT,
    SERVO_DEFAULT_ANGLES,
    SERVO_PLATFORM_BACK,
    SET_RELAY_STATE,
    SET_SERVO_ANGLE,
    STATUS_HEARTBEAT_S,
    TIMESTEP,
    USE_GPU_DECODE,
    USE_GSTREAMER,
)
from globals import process_image

from .Motor import Motor
from .SerialManager import SerialManager
from .StatusCenter import get_status_center


class Robot:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        self._initialized = True
        self.status = get_status_center()
        self.status.start_background_updates()
        self.status.log("BOOT", "Robot initialization started", force=True)

        self.sm = SerialManager()
        self.motors = [Motor(i, self.sm) for i in range(4)]
        self.motors_active = MOTORS
        self.status.log(
            "MOTOR",
            f"Motor driver ready (motors_active={self.motors_active})",
            force=True,
        )

        self.servo_angles: List[int] = [-1] * SERVO_COUNT
        for servo_id in range(SERVO_COUNT):
            default_angle = (
                SERVO_DEFAULT_ANGLES[servo_id]
                if servo_id < len(SERVO_DEFAULT_ANGLES)
                else 100
            )
            self.set_servo_angle(servo_id, default_angle)

        self.relay_on: Optional[bool] = None
        self.status.log(
            "ACT",
            f"ESP32 actuator bridge initialized (servos={SERVO_COUNT}, relay_active_low={RELAY_ACTIVE_LOW})",
            force=True,
        )

        # Use absolute hardware paths so camera indices never swap again.
        import os

        disable_cameras = DISABLE_CAMERAS or os.getenv("OVERDRIVE_DISABLE_CAMERAS") in {
            "1",
            "true",
            "TRUE",
            "yes",
            "YES",
            "on",
            "ON",
        }

        if disable_cameras:
            self.ball_cam = None
            self.line_cam = None
            self.status.log(
                "CAM",
                "Cameras disabled (OVERDRIVE_DISABLE_CAMERAS=1)",
                force=True,
            )
        elif DISABLE_LINE_CAM:
            self.status.log(
                "CAM",
                "Line camera disabled (DISABLE_LINE_CAM=True)",
                force=True,
            )
        else:
            self.ball_cam = None

        def open_camera(
            label: str,
            by_id_path: str,
            fallback_devices: list[object],
            width: int,
            height: int,
            fps: int,
            buffersize: Optional[int] = None,
        ):
            cap = None
            if os.path.exists(by_id_path):
                cap = cv2.VideoCapture(by_id_path, cv2.CAP_V4L2)
                if cap is not None and cap.isOpened():
                    self.status.log(
                        "CAM", f"{label} camera opened: {by_id_path}", force=True
                    )
                else:
                    # Some OpenCV/V4L2 builds cannot open by-id symlink paths reliably.
                    # Try the resolved /dev/videoX target before generic index fallback.
                    resolved_path = os.path.realpath(by_id_path)
                    if resolved_path and resolved_path != by_id_path:
                        cap = cv2.VideoCapture(resolved_path, cv2.CAP_V4L2)
                        if cap is not None and cap.isOpened():
                            self.status.log(
                                "CAM",
                                f"{label} camera opened via resolved path {resolved_path}",
                                force=True,
                            )
                        else:
                            self.status.log(
                                "CAM",
                                f"{label} by-id path failed, trying fallback devices {fallback_devices}",
                                level="WARN",
                                force=True,
                            )
                    else:
                        self.status.log(
                            "CAM",
                            f"{label} by-id path failed, trying fallback devices {fallback_devices}",
                            level="WARN",
                            force=True,
                        )
            else:
                self.status.log(
                    "CAM",
                    f"{label} by-id path missing, trying fallback devices {fallback_devices}",
                    level="WARN",
                    force=True,
                )

            if cap is None or not cap.isOpened():
                for dev in fallback_devices:
                    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)  # type: ignore
                    if cap is not None and cap.isOpened():
                        self.status.log(
                            "CAM",
                            f"{label} camera opened on fallback {dev}",
                            force=True,
                        )
                        break

            if cap is None or not cap.isOpened():
                raise RuntimeError(f"{label} camera could not be opened")

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            if buffersize is not None:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, buffersize)

            return cap

        if not disable_cameras:
            if ENABLE_BALL_CAM:
                self.ball_model = YOLO("./models/ball_detect_s.engine", task="detect")

                self.ball_cam = open_camera(
                    "Ball",
                    BALL_CAM_DEVICE,
                    fallback_devices=["/dev/video2", "/dev/video3", 2, 3, 0, 1],
                    width=BALL_CAM_CAPTURE_WIDTH,
                    height=BALL_CAM_CAPTURE_HEIGHT,
                    fps=BALL_CAM_CAPTURE_FPS,
                )
            else:
                self.status.log(
                    "CAM",
                    "Ball camera disabled to prioritize line camera stability",
                    force=True,
                )

            if not DISABLE_LINE_CAM:
                self.line_cam = open_camera(
                    "Line",
                    LINE_CAM_DEVICE,
                    fallback_devices=["/dev/video0", "/dev/video1", 0, 1],
                    width=LINE_CAM_CAPTURE_WIDTH,
                    height=LINE_CAM_CAPTURE_HEIGHT,
                    fps=LINE_CAM_CAPTURE_FPS,
                    buffersize=LINE_CAM_BUFFERSIZE,
                )
            else:
                self.line_cam = None

        if self.ball_cam is not None:
            ball_w = int(self.ball_cam.get(cv2.CAP_PROP_FRAME_WIDTH))
            ball_h = int(self.ball_cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.status.log(
                "CAM",
                f"Ball camera negotiated resolution {ball_w}x{ball_h}",
                force=True,
            )
            self.status.log(
                "CAM",
                f"Ball camera capture target {BALL_CAM_CAPTURE_WIDTH}x{BALL_CAM_CAPTURE_HEIGHT} @ {BALL_CAM_CAPTURE_FPS}fps",
                force=True,
            )
        if self.line_cam is not None:
            line_w = int(self.line_cam.get(cv2.CAP_PROP_FRAME_WIDTH))
            line_h = int(self.line_cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.status.log(
                "CAM",
                f"Line camera negotiated resolution {line_w}x{line_h}",
                force=True,
            )
            self.status.log(
                "CAM",
                f"Line camera capture target {LINE_CAM_CAPTURE_WIDTH}x{LINE_CAM_CAPTURE_HEIGHT} @ {LINE_CAM_CAPTURE_FPS}fps",
                force=True,
            )
            self.status.log(
                "CAM",
                f"Line processing resolution {LINE_CAM_WIDTH}x{LINE_CAM_HEIGHT}",
                force=True,
            )

        self.prev_steer = 0.0

        i2c = board.I2C()
        self.mux = TCA9548A(i2c)
        self.status.log("I2C", "TCA9548A mux initialized", force=True)

        # Bring OLED online as early as possible so health is visible immediately.
        self.status.attach_oled(
            self.mux,
            channel=OLED_MUX_CHANNEL,
            address=OLED_ADDR,
            width=OLED_WIDTH,
            height=OLED_HEIGHT,
        )
        self.status.update(None)

        self.IMU: Optional[BNO055_I2C] = None
        imu_found = False
        imu_channels = [7, 1, 0, 2, 3, 4, 5, 6]
        for ch in imu_channels:
            for addr in (0x28, 0x29):
                try:
                    candidate = BNO055_I2C(self.mux[ch], address=addr)  # type: ignore
                    _yaw = candidate.euler[0]  # type: ignore
                    self.IMU = candidate
                    self.status.log(
                        "IMU",
                        f"BNO055 connected on mux channel {ch}, addr 0x{addr:02X}",
                        force=True,
                    )
                    imu_found = True
                    break
                except Exception:
                    continue
            if imu_found:
                break

        if not imu_found:
            for addr in (0x28, 0x29):
                try:
                    candidate = BNO055_I2C(i2c, address=addr)
                    _yaw = candidate.euler[0]  # type: ignore
                    self.IMU = candidate
                    self.status.log(
                        "IMU",
                        f"BNO055 connected on base I2C, addr 0x{addr:02X}",
                        force=True,
                    )
                    imu_found = True
                    break
                except Exception:
                    continue

        if not imu_found:
            self.status.log(
                "IMU",
                "BNO055 unavailable on mux scan and base I2C scan",
                level="WARN",
                force=True,
            )

        self.yaw: float = 0.0

        self.m_pid = PID(M_KP, M_KI, M_KD, setpoint=0.0)

        # Facing the robot, 0 is Left front and 3 is right front, 7 is left, and 4 is right.
        self.distance_ids = [0, 3, 4, 7]
        self.front_ids = [3, 0]
        self.side_ids = [4, 7]

        self.distance_sensors: List[VL53L4CD] = []

        self.frame: np.ndarray = []  # type: ignore
        self.gui_frame: np.ndarray = []  # type: ignore
        self.binary_frame: np.ndarray = []  # type: ignore
        self.line_size: float = 0.0

        self.front_distances: List[float] = [1000000] * len(self.front_ids)
        self.side_distances: List[float] = [1000000] * len(self.side_ids)

        self._sensor_channel_by_index: List[int] = []
        for ch in self.distance_ids:
            try:
                vl53 = VL53L4CD(self.mux[ch])  # type: ignore
                vl53.timing_budget = 200
                vl53.inter_measurement = 0
                vl53.start_ranging()
                self.distance_sensors.append(vl53)
                self._sensor_channel_by_index.append(ch)
                self.status.log(
                    "SENSOR", f"VL53L4CD online on mux channel {ch}", force=True
                )
            except Exception as e:
                self.status.log(
                    "SENSOR",
                    f"VL53L4CD init failed on mux channel {ch}: {e}",
                    level="WARN",
                    force=True,
                )

        self.obs_detected = False
        self._last_obs_state = False
        self._obs_detect_hits = 0
        self._obs_clear_hits = 0
        self._last_heartbeat_t = 0.0

        self._last_imu_yaw = -999.0
        self._imu_freeze_frames = 0
        self._last_no_imu_log_t = 0.0
        self._last_update_end_t = time.perf_counter()
        self._latest_fps = 0.0
        self.line_cam_fps = 0.0
        self.line_cam_read_ms = 0.0
        self.line_cam_fail_count = 0
        self._last_line_cam_t = 0.0

        self.ball_count = 3

        self.status.log("BOOT", "Robot initialization completed", force=True)

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

    def set_servo_angle(self, servo_id: int, angle: int):
        if not 0 <= servo_id < SERVO_COUNT:
            raise ValueError(
                f"Invalid servo index {servo_id}, expected 0-{SERVO_COUNT - 1}"
            )

        bounded = max(0, min(int(angle), 180))
        if self.servo_angles[servo_id] == bounded:
            return
        self.servo_angles[servo_id] = bounded
        return self.sm.send(SET_SERVO_ANGLE, servo_id, bounded, 0)

    def set_arm_servo(self, angle: int):
        return self.set_servo_angle(SERVO_ARM, angle)

    def set_claw_left_servo(self, angle: int):
        return self.set_servo_angle(SERVO_CLAW_LEFT, angle)

    def set_claw_right_servo(self, angle: int):
        return self.set_servo_angle(SERVO_CLAW_RIGHT, angle)

    def set_platform_back_servo(self, angle: int):
        return self.set_servo_angle(SERVO_PLATFORM_BACK, angle)

    def set_relay(self, on: bool):
        desired = bool(on)
        if self.relay_on == desired:
            return
        self.relay_on = desired
        self.sm.send(SET_RELAY_STATE, 1 if desired else 0, 0, 0)
        output_level = "LOW" if (desired and RELAY_ACTIVE_LOW) else "HIGH"
        if not desired:
            output_level = "HIGH" if RELAY_ACTIVE_LOW else "LOW"
        self.status.log(
            "RELAY",
            f"relay {'ON' if desired else 'OFF'} (ESP32 pin level {output_level})",
            cooldown_s=0.5,
        )

    # Control represents some value to be passed into pid.
    def set_motor_output(self, control: int):
        output = int(self.m_pid(control))  # type: ignore

        left_spd = BASE_SPEED - output
        right_spd = BASE_SPEED + output

        if 0 < abs(left_spd) < MIN_WHEEL_SPEED:
            left_spd = MIN_WHEEL_SPEED * (1 if left_spd > 0 else -1)
        if 0 < abs(right_spd) < MIN_WHEEL_SPEED:
            right_spd = MIN_WHEEL_SPEED * (1 if right_spd > 0 else -1)

        self.set_left_speed(left_spd)
        self.set_right_speed(right_spd)

        self.status.log(
            "CTRL",
            f"set_motor_output control={control} steer={output} left_spd={left_spd} right_spd={right_spd}",
            cooldown_s=1.5,
            cooldown_key="CTRL:pid-output",
        )

    def reset_steering(self):
        self.prev_steer = 0.0
        self.m_pid.reset()

    def forward(self):
        self.set_left_speed(MAX_SPEED)
        self.set_right_speed(MAX_SPEED)
        self.status.log("MOTOR", "forward()", cooldown_s=1.5)

    def backward(self):
        self.set_left_speed(-MAX_SPEED)
        self.set_right_speed(-MAX_SPEED)
        self.status.log("MOTOR", "backward()", cooldown_s=1.5)

    def turnLeft(self):
        self.set_left_speed(-MAX_SPEED)
        self.set_right_speed(MAX_SPEED)
        self.status.log("MOTOR", "turnLeft()", cooldown_s=1.5)

    def turnRight(self):
        self.set_left_speed(MAX_SPEED)
        self.set_right_speed(-MAX_SPEED)
        self.status.log("MOTOR", "turnRight()", cooldown_s=1.5)

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

            cv2.putText(
                self.gui_frame,
                f"Line size: {self.line_size}",
                (280, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                self.gui_frame,
                f"Left Side: {self.side_distances[0]}",
                (20, 250),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                self.gui_frame,
                f"Right Side: {self.side_distances[1]}",
                (20, 270),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Sensor Debug", self.gui_frame)

    def turn(self, degrees: float, tol=1.0):
        self.status.log(
            "TURN",
            f"turn request degrees={degrees:.1f}, tol={tol}",
            force=True,
        )

        # If IMU is unavailable, fallback to a short timed turn so we never deadlock.
        if self.IMU is None:
            duration = min(2.0, max(0.2, abs(degrees) / 180.0))
            if degrees >= 0:
                self.turnLeft()
            else:
                self.turnRight()
            time.sleep(duration)
            self.stop()
            self.status.log(
                "TURN",
                f"turn fallback (no IMU), duration={duration:.2f}s",
                level="WARN",
                force=True,
            )
            return

        target: float = (self.yaw + degrees) % 360
        self.stop()
        time.sleep(TIMESTEP)
        while True:
            self.update()
            if GUI:
                cv2.waitKey(1)

            diff = (target - self.yaw + 180.0) % 360.0 - 180.0
            if abs(diff) <= tol:
                break

            speed = max(70, min(MAX_SPEED, int(abs(diff) * 1.5)))

            if diff > 0:
                self.set_left_speed(speed)
                self.set_right_speed(-speed)
            else:
                self.set_left_speed(-speed)
                self.set_right_speed(speed)
            time.sleep(TIMESTEP)

        self.stop()
        time.sleep(TIMESTEP)
        self.status.log("TURN", f"turn completed, yaw={self.yaw:.1f}", force=True)

    def turnToTarget(self, degrees: float, tol=1.0):
        angle: float = (degrees - self.yaw + 180.0) % 360.0 - 180.0
        self.turn(angle, tol)

    def stop(self):
        self.set_left_speed(0)
        self.set_right_speed(0)
        self.status.log("MOTOR", "stop()", cooldown_s=1.0)

    def cleanup(self):
        self.status.log("BOOT", "Robot cleanup started", force=True)
        try:
            if self.ball_cam is not None:
                self.ball_cam.release()
            if self.line_cam is not None:
                self.line_cam.release()
        except Exception as e:
            self.status.log("BOOT", f"Cleanup error: {e}", level="WARN", force=True)
        finally:
            if self.ball_cam is not None:
                self.ball_cam.release()
            if self.line_cam is not None:
                self.line_cam.release()
            self.status.log("BOOT", "Robot cleanup completed", force=True)

    def update(self):
        update_start = time.perf_counter()

        if self.IMU is not None:
            try:
                yaw = self.IMU.euler[0]  # type: ignore
                if yaw is not None:
                    # The BNO055 outputs floats like 0.625. If it's literally identical, it is frozen.
                    if abs(yaw - self._last_imu_yaw) < 0.0001:
                        self._imu_freeze_frames += 1
                    else:
                        self._imu_freeze_frames = 0
                        self._last_imu_yaw = yaw
                    self.yaw = yaw
                else:
                    self._imu_freeze_frames += 1
                    self.status.log(
                        "IMU",
                        "IMU returned None yaw sample",
                        level="WARN",
                        cooldown_s=2.0,
                    )
            except Exception as e:
                self._imu_freeze_frames += 1
                self.status.log(
                    "IMU",
                    f"IMU read error: {e}",
                    level="WARN",
                    cooldown_s=2.0,
                )
        else:
            now = time.monotonic()
            if now - self._last_no_imu_log_t > 10.0:
                self.status.log(
                    "IMU",
                    "IMU unavailable; yaw feedback disabled",
                    level="WARN",
                    force=True,
                )
                self._last_no_imu_log_t = now

        motors_spinning = any(
            abs(m.speed) > 20 and m.speed != -2000 for m in self.motors
        )
        left_cmd = 0
        right_cmd = 0
        if len(self.motors) >= 4:
            left_cmd = self.motors[0].speed + self.motors[1].speed
            right_cmd = self.motors[2].speed + self.motors[3].speed
        turning_commanded = abs(left_cmd - right_cmd) > 30

        # If motors have been actively spinning but IMU has not changed for 30 frames (0.5 sec).
        if (
            IMU_AUTO_RESET_ENABLED
            and self.IMU is not None
            and motors_spinning
            and turning_commanded
            and self._imu_freeze_frames > 30
        ):
            self.status.log(
                "IMU",
                f"IMU freeze detected (yaw={self.yaw:.1f}), rebooting BNO055",
                level="WARN",
                force=True,
            )
            try:
                # Turn off motors to recover battery voltage.
                self.set_left_speed(0)
                self.set_right_speed(0)
                time.sleep(0.1)
                # Force hardware reset via register 0x3F (SYS_TRIGGER) bit 5.
                self.IMU._reset()
                time.sleep(0.1)
                self.IMU.mode = 0x0C  # NDOF mode
                self.status.log("IMU", "IMU reboot completed", force=True)
            except Exception as err:
                self.status.log(
                    "IMU",
                    f"IMU reboot failed: {err}",
                    level="WARN",
                    force=True,
                )
            self._imu_freeze_frames = 0

        cam_read_start = time.perf_counter()

        if self.line_cam is not None:
            ret, frame = self.line_cam.read()

            self.line_cam_read_ms = (time.perf_counter() - cam_read_start) * 1000.0
            if not ret:
                self.line_cam_fail_count += 1
                self.status.log(
                    "CAM", "Line camera frame read failed", level="WARN", cooldown_s=1.0
                )
                self.status.update(self)
                return
            now_mono = time.monotonic()
            if self._last_line_cam_t > 0:
                dt = now_mono - self._last_line_cam_t
                if dt > 0:
                    inst = 1.0 / dt
                    if self.line_cam_fps <= 0:
                        self.line_cam_fps = inst
                    else:
                        self.line_cam_fps = (0.85 * self.line_cam_fps) + (0.15 * inst)
            self._last_line_cam_t = now_mono

            frame = cv2.flip(frame, -1)

            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            self.frame = frame

        if self.ball_cam is not None:
            ret2, ball_frame = self.ball_cam.read()
            ball_frame = cv2.flip(ball_frame, 0)
            if len(ball_frame.shape) == 3 and ball_frame.shape[2] == 4:
                ball_frame = cv2.cvtColor(ball_frame, cv2.COLOR_BGRA2BGR)

            self.ball_frame = ball_frame

        if self.line_cam is not None:
            self.gui_frame = self.frame.copy()
            self.binary_frame, self.line_size = process_image(self.frame)

        for idx, sensor in enumerate(self.distance_sensors):
            try:
                if sensor.data_ready:
                    sensor.clear_interrupt()
                    dist = sensor.distance
                    status = sensor.range_status
                    sensor_id = self._sensor_channel_by_index[idx]
                    if status == 0 and dist > 1:
                        if sensor_id in self.front_ids:
                            self.front_distances[self.front_ids.index(sensor_id)] = dist
                        elif sensor_id in self.side_ids:
                            self.side_distances[self.side_ids.index(sensor_id)] = dist
                    else:
                        if sensor_id in self.front_ids:
                            self.front_distances[self.front_ids.index(sensor_id)] = (
                                1000000
                            )
                        elif sensor_id in self.side_ids:
                            self.side_distances[self.side_ids.index(sensor_id)] = (
                                1000000
                            )
            except OSError:
                self.status.log(
                    "SENSOR",
                    f"Transient OSError from sensor idx={idx}, attempting recovery",
                    level="WARN",
                    cooldown_s=2.0,
                )
                try:
                    sensor.clear_interrupt()
                    sensor.start_ranging()
                except Exception:
                    pass
                continue
            except Exception as e:
                self.status.log(
                    "SENSOR",
                    f"Sensor idx={idx} exception: {e}",
                    level="WARN",
                    cooldown_s=2.0,
                )
                continue

        min_front = min(self.front_distances) if self.front_distances else 1_000_000

        if min_front < FRONT_OBS_ENTER_THRESH:
            self.obs_detected = True
        else:
            self.obs_detected = False

        # if not self.obs_detected:
        #     if min_front < FRONT_OBS_ENTER_THRESH:
        #         self._obs_detect_hits += 1
        #     else:
        #         self._obs_detect_hits = 0

        #     if self._obs_detect_hits >= 2:
        #         self.obs_detected = True
        #         self._obs_clear_hits = 0
        # else:
        #     if min_front < FRONT_OBS_EXIT_THRESH:
        #         self._obs_clear_hits = 0
        #     else:
        #         self._obs_clear_hits += 1

        #     if self._obs_clear_hits >= 3:
        #         self.obs_detected = False
        #         self._obs_detect_hits = 0
        if self.obs_detected != self._last_obs_state:
            if self.obs_detected:
                self.status.log(
                    "OBS",
                    f"Obstacle detected front={self.front_distances}",
                    level="WARN",
                    cooldown_s=1.0,
                    cooldown_key="OBS:state-change",
                )
            else:
                self.status.log(
                    "OBS",
                    "Obstacle cleared",
                    cooldown_s=1.0,
                    cooldown_key="OBS:state-change",
                )
            self._last_obs_state = self.obs_detected

        now = time.monotonic()
        if now - self._last_heartbeat_t >= STATUS_HEARTBEAT_S:
            self.status.log(
                "HEALTH",
                (
                    f"yaw={self.yaw:.1f} line={self.line_size:.0f} "
                    f"front={self.front_distances} side={self.side_distances} "
                    f"obs={self.obs_detected} fps={self._latest_fps:.1f} "
                    f"camfps={self.line_cam_fps:.1f} readms={self.line_cam_read_ms:.1f} "
                    f"drops={self.line_cam_fail_count}"
                ),
                force=True,
            )
            self._last_heartbeat_t = now

        # if GUI:
        # self.update_gui()

        self.status.update(self)

        update_end = time.perf_counter()
        loop_dt = update_end - self._last_update_end_t
        self._last_update_end_t = update_end
        if loop_dt > 0:
            self._latest_fps = 1.0 / loop_dt
            if self._latest_fps < LOW_FPS_WARN_THRESHOLD:
                self.status.log(
                    "PERF",
                    (
                        f"Low update FPS {self._latest_fps:.1f} "
                        f"(update took {(update_end - update_start) * 1000:.1f} ms)"
                    ),
                    level="WARN",
                    cooldown_s=5.0,
                    cooldown_key="PERF:low-fps",
                )

    def pause_motors(self, seconds: float):
        if not self.motors_active:
            return

        self.stop()
        self.motors_active = False
        self.status.log("MOTOR", f"Motors paused for {seconds:.2f}s", force=True)

        def re_enable():
            self.motors_active = MOTORS
            self.status.log("MOTOR", "Motors re-enabled", force=True)

        threading.Timer(seconds, re_enable).start()
