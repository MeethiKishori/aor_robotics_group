"""
Runs the YOLO human/robot detector + depth distance lookup + hazard list live.

Run from project root:
    cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera
    source realsense_env/bin/activate
    python3 python/launch/run_yolo_hazard.py
"""

import os
import sys
import cv2
import numpy as np

THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(THIS_DIR)
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from perception.yolo_detector import YoloDetector
from perception.depth_estimation import attach_distances
from perception.hazard import HazardRepresentation
from camera.realsense_stream import start_aligned_pipeline
from risk.runtime_risk_model import compute_object_risks

COLOR_WIDTH,  COLOR_HEIGHT = 1280, 720
DEPTH_WIDTH,  DEPTH_HEIGHT = 640,  480
FPS = 30


def draw_detections(frame, object_risks):
    for obj, risk in object_risks:
        x, y, w, h = obj.box
        color  = (0, 255, 0) if obj.category == "HUMAN" else (255, 0, 0)
        dist_s = f"{obj.distance_m:.2f}m" if obj.distance_m > 0 else "?"
        label  = f"{obj.category} {obj.confidence:.2f} {dist_s} R={risk:.1f}"
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, label, (x, max(12, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)


def main():
    pipeline, align = start_aligned_pipeline(
        COLOR_WIDTH, COLOR_HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, FPS
    )
    detector = YoloDetector()
    hazard   = HazardRepresentation()

    print(f"Running {COLOR_WIDTH}x{COLOR_HEIGHT} | press q to quit")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            detections = detector.detect(frame)
            attach_distances(detections, depth_frame, COLOR_WIDTH, COLOR_HEIGHT)
            hazard.update(detections)

            # velocity_norm left at 0.0 (robot velocity not tracked yet) -> R_i = w(c_i) * g(d_i)
            object_risks = compute_object_risks(hazard.objects)
            risk_env     = max((r for _, r in object_risks), default=0.0)

            draw_detections(frame, object_risks)

            nearest = hazard.nearest()
            nearest_s = f"{nearest.category} {nearest.distance_m:.2f}m" if nearest else "none"
            cv2.putText(frame, f"R_env={risk_env:.1f}  nearest: {nearest_s}  count={len(hazard.objects)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            print(f"R_env={risk_env:.2f}  distances={hazard.distances()}  nearest={nearest_s}    \r", end="", flush=True)

            cv2.imshow("YOLO Hazard", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
