from ultralytics import YOLO
import cv2

IMAGE_PATH   = "/home/ssingh/Finroc/singh_files/aor_robotics_group/recordings/dog_human3.webp"
MODEL_PATH   = "/home/ssingh/Finroc/singh_files/aor_robotics_group/realsense_camera/models/best.pt"  # custom-trained on Unitree Go1 robot

model_unitree = YOLO(MODEL_PATH)
model_human   = YOLO("yolov8n.pt")

frame = cv2.imread(IMAGE_PATH)

# --- unitree detection (green box) ---
for box in model_unitree(frame)[0].boxes:
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    label = f"{model_unitree.names[int(box.cls[0])]} {float(box.conf[0]):.2f}"
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# --- human detection (blue box) ---
for box in model_human(frame, classes=[0])[0].boxes:
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    label = f"person {float(box.conf[0]):.2f}"
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

cv2.imshow("Detection", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
