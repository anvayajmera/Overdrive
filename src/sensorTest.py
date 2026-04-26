import time
from typing import List

import adafruit_bno055
import adafruit_ssd1306
import adafruit_tca9548a
import adafruit_vl53l4cd
import board
import Jetson.GPIO as GPIO
from busio import I2C
from PIL import Image, ImageDraw, ImageFont

# print("Mode is", GPIO.getmode())

i2c = board.I2C()

pca = adafruit_tca9548a.TCA9548A(i2c)

board_to_tegra = {
    k: list(GPIO.gpio_pin_data.get_data()[-1]["TEGRA_SOC"].keys())[i]
    for i, k in enumerate(GPIO.gpio_pin_data.get_data()[-1]["BOARD"])
}  # type: ignore

# for k, v in board_to_tegra.items():
#     print("board #:", k, "tegra:", v)

touchPin = board_to_tegra[15]
# print(touchPin)
GPIO.setup(touchPin, GPIO.IN)

relayPin = board_to_tegra[21]
GPIO.setup(relayPin, GPIO.OUT)


for channel in range(8):
    if pca[channel].try_lock():
        print("Channel {}".format(channel), end="")
        addresses = pca[channel].scan()
        print([hex(address) for address in addresses if address != 0x70])
        pca[channel].unlock()

# oledChannel = 5
# oled = adafruit_ssd1306.SSD1306_I2C(128, 64, pca[oledChannel], addr=0x3D)

# GPIO.output(relayPin, GPIO.LOW)

# time.sleep(1)

# GPIO.output(relayPin, GPIO.HIGH)

GPIO.output(relayPin, GPIO.HIGH)
time.sleep(3)
GPIO.output(relayPin, GPIO.LOW)
time.sleep(3)

bno_channel = 4
bno = adafruit_bno055.BNO055_I2C(pca[bno_channel])

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

# Facing the robot, 0 is Left front and 3 is right front, 7 is left, and 4 is right
distanceSensors = [
    # 0,
    # 3,
    # 4,
    4,
]
sensors: List[adafruit_vl53l4cd.VL53L4CD] = []
for ch in distanceSensors:
    vl53 = adafruit_vl53l4cd.VL53L4CD(pca[ch])
    vl53.timing_budget = 200
    vl53.inter_measurement = 0
    vl53.start_ranging()
    sensors.append(vl53)

while True:
    time.sleep(0.2)

    # if not GPIO.input(touchPin):
    #     print("Touch sensor is touched.")
    # else:
    #     print("Touch sensor not touched.")

    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (bno.acceleration))
    # print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % (mpu.acceleration))
    # print(bno.euler)

    # for idx, sensor in enumerate(sensors):
    #     while not sensor.data_ready:
    #         pass
    #     sensor.clear_interrupt()
    #     print(
    #         f"Sensor {idx}: {sensor.distance} cm, Range Status: {sensor.range_status}",
    #         end="; ",
    #     )

    print("\n")
