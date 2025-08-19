from ultralytics import YOLO

model = YOLO("./models/silver_classify_s.pt")

model.export(format="engine")