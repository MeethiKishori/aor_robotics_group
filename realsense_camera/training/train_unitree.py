from roboflow import Roboflow
from ultralytics import YOLO

# ── Step 1: Download dataset from Roboflow ────────────────────────────────────
rf = Roboflow(api_key="SZWxiFpVOJN6mid3QCXk")
project = rf.workspace("shubham-8f3y0").project("unitree-go1-ba7qp")
version = project.version(1)
dataset = version.download("yolov8")

# dataset.location → path to downloaded dataset, e.g. "unitree-go1-ba7qp-1/"
# folder structure created automatically:
#   unitree-go1-ba7qp-1/
#   ├── data.yaml          ← class names + paths
#   ├── train/
#   │   ├── images/
#   │   └── labels/
#   ├── valid/
#   │   ├── images/
#   │   └── labels/
#   └── test/
#       ├── images/
#       └── labels/

# ── Step 2: Train ─────────────────────────────────────────────────────────────
model = YOLO("yolov8n.pt")  # start from COCO pretrained weights

model.train(
    data=f"{dataset.location}/data.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    name="unitree_go1",
    project="training/runs",
)

# trained weights saved to:
#   training/runs/unitree_go1/weights/best.pt   ← use this
#   training/runs/unitree_go1/weights/last.pt
