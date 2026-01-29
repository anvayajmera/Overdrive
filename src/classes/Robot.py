from .SerialManager import SerialManager
from .Motor import Motor
from simple_pid import PID

import cv2

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
        self.sm = SerialManager()
        self.motors = [Motor(i, self.sm) for i in range(4)]
        self.base_speed = 50
        self.max_speed = 100
        
        self.ball_cam = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.line_cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        self.ball_cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*'MJPG'))
        self.ball_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.ball_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.ball_cam.set(cv2.CAP_PROP_FPS, 60)
        
        self.line_cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*'MJPG'))
        self.line_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.line_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.line_cam.set(cv2.CAP_PROP_FPS, 60)
        
        self.lt_start: tuple[int, int] = (0, 0)
        self.lt_end: tuple[int, int] = (0, 0)
        self.dir: tuple[float, float] = (0.0, 1.0)
        
        self.m_pid = PID(0.1, 0, 0.05)
       
    def set_left_speed(self, speed: int):
        self.motors[0].set_speed(speed)
        self.motors[1].set_speed(speed)
        
    def set_right_speed(self, speed: int):
        self.motors[2].set_speed(speed)
        self.motors[3].set_speed(speed)
     
    # Control represents some value to be passed into pid
    def set_motor_output(self, control: int):
        output = int(self.m_pid(control)) # type: ignore
        
        self.set_left_speed(self.base_speed + output)
        self.set_right_speed(self.base_speed - output)
            
    def turnLeft(self):
        self.set_left_speed(self.max_speed)
        self.set_right_speed(-self.max_speed)
    
    def turnRight(self):
        self.set_left_speed(-self.max_speed)
        self.set_right_speed(self.max_speed)
            
    def stop(self):
        for i in range(4):
            self.motors[i].set_speed(0)
            
    def cleanup(self):
        self.ball_cam.release()
        self.line_cam.release()
    
        
        
        
        


