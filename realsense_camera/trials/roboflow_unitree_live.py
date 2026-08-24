import os
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
MODEL_PATH = os.path.join(MODELS_DIR, "best.pt")

model_unitree = YOLO(MODEL_PATH)
model_human   = YOLO(os.path.join(MODELS_DIR, "yolov8n.pt"))

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        # --- unitree detection (green box) ---
        for box in model_unitree(frame)[0].boxes: # list of all detected bouding boxes
            x1, y1, x2, y2 = map(int, box.xyxy[0]) # map use to converts float to integers so opencv uses
            label = f"{model_unitree.names[int(box.cls[0])]} {float(box.conf[0]):.2f}" # box.conf is confidence score and box.clas is is class index, then .names is the name called go1
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2) # generate rectangle
            cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- human detection (blue box) ---
        for box in model_human(frame, classes=[0])[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = f"person {float(box.conf[0]):.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
