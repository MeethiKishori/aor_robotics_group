from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # fix: removed extra `1`

pipeline.start(config)

model_human   = YOLO("yolov8n.pt")                  # COCO pretrained — for person detection
model_unitree = YOLO("/home/ssingh/Finroc/singh_files/aor_robotics_group/realsense_camera/models/best.pt")  # custom-trained on Unitree Go1 robot

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        # --- human detection (green box) ---
        human_results = model_human(frame, classes=[0])[0]
        for box in human_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf  = float(box.conf[0])
            label = f"person {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- unitree detection (blue box) ---
        robot_results = model_unitree(frame)[0]
        for box in robot_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = f"{model_unitree.names[cls]} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()                  # fix: stop pipeline properly
    cv2.destroyAllWindows()
