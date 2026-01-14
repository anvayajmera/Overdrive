import Jetson.GPIO as GPIO
import board, time
import adafruit_vl53l4cd
import adafruit_tca9548a
import adafruit_ssd1306
import adafruit_mpu6050
from busio import I2C
from typing import List
from PIL import Image, ImageDraw, ImageFont

# print("Mode is", GPIO.getmode())

i2c = board.I2C()

pca = adafruit_tca9548a.TCA9548A(i2c)

board_to_tegra = {
k: list(GPIO.gpio_pin_data.get_data()[-1]['TEGRA_SOC'].keys())[i] for i, k in enumerate(GPIO.gpio_pin_data.get_data()[-1]['BOARD'])}

for k, v in board_to_tegra.items():
    print('board #:', k, 'tegra:', v)

touchPin = board_to_tegra[15]
# print(touchPin)
GPIO.setup(touchPin, GPIO.IN)

# relayPin = board_to_tegra[13]
# GPIO.setup(relayPin, GPIO.OUT)

# for channel in range(8):
#     if pca[channel].try_lock():
#         print("Channel {}".format(channel), end="")
#         addresses = pca[channel].scan()
#         print([hex(address) for address in addresses if address != 0x70])
#         pca[channel].unlock()
#
# print(pca[5])

# oledChannel = 5
# oled = adafruit_ssd1306.SSD1306_I2C(128, 64, pca[oledChannel], addr=0x3D)

# GPIO.output(relayPin, GPIO.LOW)

# time.sleep(1)

# GPIO.output(relayPin, GPIO.HIGH)

# mpuChannel = 1
# # Monkey-patch the _CHIP_ID check
# original_init = adafruit_mpu6050.MPU6050.__init__
#
# def patched_init(self, i2c_bus: I2C, address: int = 0x68):
#     from adafruit_bus_device import i2c_device
#     from time import sleep
#     from adafruit_mpu6050 import Bandwidth, GyroRange, Range, ClockSource
#
#     self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
#
#     self.reset()
#
#     self._sample_rate_divisor = 0
#     self._filter_bandwidth = Bandwidth.BAND_260_HZ
#     self._gyro_range = GyroRange.RANGE_500_DPS
#     self._accel_range = Range.RANGE_2_G
#     self._accel_scale = 1.0 / [16384, 8192, 4096, 2048][self._accel_range]
#     sleep(0.100)
#     self.clock_source = ClockSource.CLKSEL_INTERNAL_X  # set to use gyro x-axis as reference
#     sleep(0.100)
#     self.sleep = False
#     sleep(0.010)
#
# # Apply the patch
# adafruit_mpu6050.MPU6050.__init__ = patched_init
#
# mpu = adafruit_mpu6050.MPU6050(pca[mpuChannel], address=0x68)

# oled.fill(0)
# oled.show()
#
# image = Image.new("1", (oled.width, oled.height))
# draw = ImageDraw.Draw(image)
#
# font = ImageFont.load_default()
#
# draw.text((0, 0), "Hello!", font=font, fill=255)
#
# oled.image(image)
# oled.show()

# Facing the robot, 0 is Left and 3 is right,
# distanceSensors = [0, 3]
# sensors: List[adafruit_vl53l4cd.VL53L4CD] = []
# for ch in distanceSensors:
#     vl53 = adafruit_vl53l4cd.VL53L4CD(pca[ch])
#     vl53.timing_budget = 200
#     vl53.inter_measurement = 0
#     vl53.start_ranging()
#     sensors.append(vl53)

while True:
    if GPIO.input(touchPin):
        print("Touch sensor is touched.")
    time.sleep(0.1)

    # print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.gyro))
    # print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % (mpu.acceleration))

    time.sleep(0.1)

    # for idx, sensor in enumerate(sensors):
    #     while not sensor.data_ready:
    #         pass
    #     sensor.clear_interrupt()
    #     print(f"Sensor {idx}: {sensor.distance} cm", end="; ")

    # print("\n")

