from ultralytics import YOLO

model = YOLO("./models/silver_classify_s.pt")

model.export(format="engine")


# Sensor testing code.
# from smbus2 import SMBus
#
# bus = SMBus(7)
# address = 0x29
#
# while True:
#     value = bus.read_byte_data(address, 0x00)
#     print(f"Value read: {value}")
#
#
#
# bus.close()
