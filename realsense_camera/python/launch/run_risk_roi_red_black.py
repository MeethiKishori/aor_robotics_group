import os
import sys

import cv2
import numpy as np

# Allow imports from ../perception when run as a script.
THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(THIS_DIR)
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from perception.red_black_roi import detect_red_black_in_roi   # red/black detector module
from camera.realsense_stream import read_accel_magnitude, start_aligned_pipeline
from risk.risk_score import compute_ttc_risk, nearest_distance_from_detections, risk_label
from risk.runtime import RiskRuntimeState


# ======================
# Easy-to-change settings
# ======================
# Use separate resolutions for color and depth. Many D435i setups support
# 1920x1080 color but depth is commonly 640x480 at 30 FPS.
COLOR_WIDTH  = 1920
COLOR_HEIGHT = 1080
DEPTH_WIDTH  = 640
DEPTH_HEIGHT = 480
FPS          = 30
ROI_FRAC = 0.50    # fraction of image used as center ROI (0.30 to 0.50)
MIN_AREA = 10000   # minimum blob pixel area to count as a detection (increase to ignore small blobs)


# ──────────────────────────────────────────────
# HELPER: compute center ROI pixel rectangle
# ──────────────────────────────────────────────

def roi_rect_from_frac(width, height, roi_frac):
    # Compute (x1, y1, x2, y2) of a center-aligned ROI.
    # roi_frac = 0.5 means the ROI is 50% of image width and height.
    roi_w = int(width  * roi_frac)
    roi_h = int(height * roi_frac)
    x1 = (width  - roi_w) // 2   # left edge
    y1 = (height - roi_h) // 2   # top edge
    x2 = x1 + roi_w
    y2 = y1 + roi_h
    return x1, y1, x2, y2


# ──────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────

def main():
    pipeline, align, use_imu = start_aligned_pipeline(
        COLOR_WIDTH,
        COLOR_HEIGHT,
        DEPTH_WIDTH,
        DEPTH_HEIGHT,
        FPS,
        try_imu=True,
    )
    if not use_imu:
        print("IMU stream config not available. Running depth+color mode.")

    state = RiskRuntimeState()

    print("Running: ROI + red/black detection + TTC risk. Press q to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()   # block until next frame set arrives
            frames = align.process(frames)        # align depth pixels to color pixels

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue   # skip if either frame is missing

            # Runtime updates (time delta, accel read, speed estimate).
            dt = state.next_dt()
            if use_imu:
                state.accel_mag = read_accel_magnitude(frames, fallback=state.accel_mag)
            speed_est = state.update_speed(use_imu, dt)

            # ── Color + depth image processing ───────────────────────────────
            frame = np.asanyarray(color_frame.get_data())   # BGR image as numpy array
            h, w  = frame.shape[:2]
            roi   = roi_rect_from_frac(w, h, ROI_FRAC)
            x1, y1, x2, y2 = roi

            # Run red/black detector only inside the ROI.
            detections = detect_red_black_in_roi(frame, depth_frame, roi, MIN_AREA)

            # ── Draw ROI box ─────────────────────────────────────────────────
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"ROI {int(ROI_FRAC * 100)}%",
                        (x1 + 5, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

            # ── Draw detection boxes ─────────────────────────────────────────
            nearest = float("inf")
            for d in detections:
                x, y, bw, bh = d["box"]
                dist = d["distance_m"]
                if dist > 0:
                    nearest = min(nearest, dist)   # track nearest detected object

                cv2.rectangle(frame, (x, y), (x + bw, y + bh), d["draw_color"], 2)
                cv2.putText(frame, f"{d['label']} D:{dist:.2f}m",
                            (x, max(20, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, d["draw_color"], 2, cv2.LINE_AA)

            # ── Risk from RED/BLACK detections only (modular) ───────────────
            nearest = nearest_distance_from_detections(detections)

            # Closure rate based on nearest detected target distance across frames.
            closure_rate = state.compute_closure_rate(nearest, dt)

            # Use whichever is higher: camera speed or target closure speed.
            effective_speed = max(speed_est, closure_rate)
            risk, ttc = compute_ttc_risk(nearest, effective_speed)
            level, level_color = risk_label(risk)

            # ── Draw multi-line status at top-left ────────────────────────────
            if nearest < float("inf"):
                dist_txt = f"distance: {nearest:.2f} m"
                ttc_txt = f"ttc: {ttc:.2f} s"
            else:
                dist_txt = "distance: none"
                ttc_txt = "ttc: none"

            cv2.putText(frame, f"risk: {risk}/10 ({level})",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, level_color, 2, cv2.LINE_AA)
            cv2.putText(frame, dist_txt,
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, ttc_txt,
                        (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"speed: {effective_speed:.2f} m/s",
                        (10, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)

            # Show MIN_AREA as reminder of current sensitivity.
            cv2.putText(frame, f"MIN_AREA={MIN_AREA}px",
                        (10, 134), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2, cv2.LINE_AA)

            # Print live telemetry in terminal too (single updating line).
            if nearest < float("inf"):
                print(
                    f"risk={risk}/10 {level} | dist={nearest:.2f}m | ttc={ttc:.2f}s | speed={effective_speed:.2f}m/s    \r",
                    end="", flush=True,
                )
            else:
                print(
                    f"risk=0/10 LOW | dist=none | ttc=none | speed={effective_speed:.2f}m/s    \r",
                    end="", flush=True,
                )

            cv2.imshow("Risk ROI Red/Black", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
