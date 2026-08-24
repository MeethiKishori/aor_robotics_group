from ultralytics import YOLO
model = YOLO("yolov8n.pt")
print(model.names)  # {0: 'person', 1: 'bicycle', ...}
