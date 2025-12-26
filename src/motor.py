import Jetson.GPIO as GPIO

class Motor:
    # in1 is PWM, in2 is DIR
    def __init__(self, in1: int, in2: int):
        # Setup pins
        try:
            GPIO.setup(in1, GPIO.OUT)
            GPIO.setup(in2, GPIO.OUT)
        except Exception as e:
            print(f"Error occurred: {e}")

        self.IN1 = in1
        self.IN2 = in2
        self.speed = 0
        self.pwm = GPIO.PWM(self.IN1, 1000)
        self.pwm.start(0)

    def set_speed(self, speed: int):
        self.speed = speed
        GPIO.output(self.IN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(self.speed)

    def reverse(self):
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.IN2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(self.speed)

    def __del__(self):
        self.pwm.stop()