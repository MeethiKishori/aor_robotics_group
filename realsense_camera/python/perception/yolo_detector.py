import os
from ultralytics import YOLO

THIS_DIR    = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR  = os.path.dirname(THIS_DIR)
REALSENSE_DIR = os.path.dirname(PYTHON_DIR)

HUMAN_MODEL_PATH = os.path.join(REALSENSE_DIR, "models", "yolov8n.pt")  # COCO pretrained, class 0 = person
ROBOT_MODEL_PATH = os.path.join(REALSENSE_DIR, "models", "best.pt")  # fine-tuned on Unitree Go1/Go2

LABEL_HUMAN = "HUMAN"
LABEL_ROBOT = "ROBOT"
DRAW_COLOR  = {LABEL_HUMAN: (0, 255, 0), LABEL_ROBOT: (255, 0, 0)}


class YoloDetector:
    """Object detection only (no distance). Identifies humans and the Unitree robot in a BGR frame."""

    def __init__(self, human_model_path=HUMAN_MODEL_PATH, robot_model_path=ROBOT_MODEL_PATH,
                 human_conf=0.4, robot_conf=0.4):
        self.model_human = YOLO(human_model_path)
        self.model_robot = YOLO(robot_model_path)
        self.human_conf  = human_conf
        self.robot_conf  = robot_conf

    def detect(self, frame_bgr):
        """Run both models on one frame and return a flat list of detection dicts:
        {label, box (x, y, w, h), confidence, draw_color}.
        """
        detections = []

        human_results = self.model_human(frame_bgr, classes=[0], conf=self.human_conf, verbose=False)[0]
        for box in human_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append({
                "label":      LABEL_HUMAN,
                "box":        (x1, y1, x2 - x1, y2 - y1),
                "confidence": float(box.conf[0]),
                "draw_color": DRAW_COLOR[LABEL_HUMAN],
            })

        robot_results = self.model_robot(frame_bgr, conf=self.robot_conf, verbose=False)[0]
        for box in robot_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append({
                "label":      LABEL_ROBOT,
                "box":        (x1, y1, x2 - x1, y2 - y1),
                "confidence": float(box.conf[0]),
                "draw_color": DRAW_COLOR[LABEL_ROBOT],
            })

        return detections
