import os
import sys
import argparse
import threading
import time as _time_mod

import cv2
import numpy as np

# Allow imports from ../perception when run as a script.
THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(THIS_DIR)
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from perception.red_black_roi import detect_red_black_in_roi   # red/black detector module
from camera.realsense_stream import start_aligned_pipeline
from risk.risk_score import (
    compute_ttc_risk,
    nearest_distance_from_detections,
    risk_label,
    select_strongest_detections,
)
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
ROI_FRAC = 0.70    # fraction of image used as center ROI (0.30 to 0.50)
MIN_AREA = 10000   # minimum blob pixel area to count as a detection (increase to ignore small blobs)
TOP_OBJECTS = 3    # draw/analyze only top strongest detections
ASSUME_STABLE_CAMERA = True  # True: ignore IMU speed (avoids drift), use object closure only
TEST_MODE = True #False     #: synthetic circle (no camera) | False: live RealSense camera


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


def detection_track_key(detection, grid=80):
    # Stable-ish key for object tracking across frames using label + quantized center.
    x, y, w, h = detection["box"]
    cx = x + w // 2
    cy = y + h // 2
    return f"{detection['label']}:{cx // grid}:{cy // grid}"


def draw_object_overlay(frame, x, y, w, h, color, line1, line2):
    # Draw a readable two-line label block slightly away from the bounding box.
    font = cv2.FONT_HERSHEY_SIMPLEX
    fs = 0.58
    th = 1
    pad = 6

    (w1, h1), _ = cv2.getTextSize(line1, font, fs, th)
    (w2, h2), _ = cv2.getTextSize(line2, font, fs, th)
    label_w = max(w1, w2) + 2 * pad
    label_h = h1 + h2 + 3 * pad

    # Place text block below box with gap. If near bottom, place above box.
    gap = 10
    lx = max(0, min(frame.shape[1] - label_w - 1, x))
    ly = y + h + gap
    if ly + label_h >= frame.shape[0]:
        ly = max(0, y - gap - label_h)

    # Filled dark background for readability.
    cv2.rectangle(frame, (lx, ly), (lx + label_w, ly + label_h), (20, 20, 20), -1)
    cv2.rectangle(frame, (lx, ly), (lx + label_w, ly + label_h), color, 1)

    t1y = ly + pad + h1
    t2y = ly + 2 * pad + h1 + h2
    cv2.putText(frame, line1, (lx + pad, t1y), font, fs, color, th, cv2.LINE_AA)
    cv2.putText(frame, line2, (lx + pad, t2y), font, fs, color, th, cv2.LINE_AA)


# ──────────────────────────────────────────────
# SYNTHETIC TEST HELPERS
# ──────────────────────────────────────────────

class FakeDepthFrame:
    """Duck-typed depth frame: returns depth of nearest circle region at (x,y)."""
    def __init__(self, regions):
        # regions: list of (cx, cy, radius, depth_m)
        self._regions = regions

    def get_distance(self, x, y):
        best_depth = 0.0
        best_dist2 = float("inf")
        for (cx, cy, r, depth_m) in self._regions:
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if d2 < best_dist2:
                best_dist2 = d2
                best_depth = depth_m
        return best_depth


def make_test_frame(width, height, black_scale, red_scale):
    """White frame with a black circle (left) and red circle (right)."""
    img = np.full((height, width, 3), 255, dtype=np.uint8)
    # Black circle — left-center
    br = max(20, int(60 * black_scale))
    bx, by = width // 3, height // 2
    cv2.circle(img, (bx, by), br, (0, 0, 0), -1)
    # Red circle — right-center
    rr = max(20, int(55 * red_scale))
    rx, ry = 2 * width // 3, height // 2
    cv2.circle(img, (rx, ry), rr, (0, 0, 200), -1)
    return img


def approach_to_depth(approach_scale):
    """Map approach_scale [0.8 .. 2.6] → depth [3.0 .. 0.1] m."""
    t = (approach_scale - 0.8) / (2.6 - 0.8)
    return 3.0 - t * 2.99


# ──────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Run red/black ROI risk pipeline")
    parser.add_argument("--color-width",  type=int, default=COLOR_WIDTH)
    parser.add_argument("--color-height", type=int, default=COLOR_HEIGHT)
    parser.add_argument("--depth-width",  type=int, default=DEPTH_WIDTH)
    parser.add_argument("--depth-height", type=int, default=DEPTH_HEIGHT)
    parser.add_argument("--fps",          type=int, default=FPS)
    parser.add_argument("--test", action="store_true",
                        help="Synthetic mode: no camera, black circle oscillates with fake depth")
    args = parser.parse_args()

    test_mode = args.test or TEST_MODE
    color_w = max(320, args.color_width)
    color_h = max(240, args.color_height)
    fps     = max(1, args.fps)

    # Effective min_area: smaller for test (circle starts small)
    min_area = 500 if test_mode else MIN_AREA

    pipeline = align = None
    if not test_mode:
        depth_w = max(320, args.depth_width)
        depth_h = max(240, args.depth_height)
        pipeline, align = start_aligned_pipeline(color_w, color_h, depth_w, depth_h, fps)

    state   = RiskRuntimeState()

    # Synthetic oscillation state — black and red oscillate independently
    black_scale = 1.0;  black_dir = 1.0;  black_speed = 0.6
    red_scale   = 1.5;  red_dir   = 1.0;  red_speed   = 0.9   # red starts closer, faster
    last_t = _time_mod.monotonic()

    mode_label = "TEST (synthetic)" if test_mode else f"LIVE {color_w}x{color_h}"
    print(f"Running: {mode_label} | Press q to quit.")
    if test_mode:
        print("  BLACK circle left, RED circle right.  +/- black speed,  [/] red speed,  q quit.")

    try:
        while True:
            if test_mode:
                # ── Synthetic frame + depth ───────────────────────────────
                now = _time_mod.monotonic()
                dt  = max(now - last_t, 1e-3)
                last_t = now

                black_scale += black_dir * black_speed * dt
                if black_scale > 2.6: black_scale = 2.6; black_dir = -1.0
                if black_scale < 0.8: black_scale = 0.8; black_dir =  1.0

                red_scale += red_dir * red_speed * dt
                if red_scale > 2.6: red_scale = 2.6; red_dir = -1.0
                if red_scale < 0.8: red_scale = 0.8; red_dir =  1.0

                frame = make_test_frame(color_w, color_h, black_scale, red_scale)

                # Each circle region gets its own depth
                bx, by = color_w // 3,     color_h // 2
                rx, ry = 2 * color_w // 3, color_h // 2
                br = max(20, int(60 * black_scale))
                rr = max(20, int(55 * red_scale))
                depth_src = FakeDepthFrame([
                    (bx, by, br, approach_to_depth(black_scale)),
                    (rx, ry, rr, approach_to_depth(red_scale)),
                ])

                key = cv2.waitKey(int(1000 / fps)) & 0xFF
                if key == ord('q'):
                    break
                if key in (ord('+'), ord('=')):
                    black_speed = min(4.0, black_speed + 0.1)
                if key in (ord('-'), ord('_')):
                    black_speed = max(0.1, black_speed - 0.1)
                if key == ord(']'):
                    red_speed = min(4.0, red_speed + 0.1)
                if key == ord('['):
                    red_speed = max(0.1, red_speed - 0.1)

            else:
                # ── Live camera frame ─────────────────────────────────────
                frames = pipeline.wait_for_frames()
                frames = align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                dt        = state.next_dt()
                frame     = np.asanyarray(color_frame.get_data())
                depth_src = depth_frame

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            h, w = frame.shape[:2]
            if not test_mode:
                dt = state.next_dt()
            roi   = roi_rect_from_frac(w, h, ROI_FRAC)
            x1, y1, x2, y2 = roi

            # Run red/black detector only inside the ROI.
            detections = detect_red_black_in_roi(frame, depth_src, roi, min_area)

            # ── Draw ROI box ─────────────────────────────────────────────────
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"ROI {int(ROI_FRAC * 100)}%",
                        (x1 + 5, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)

            # Keep only strongest N detections for further analysis and display.
            strongest = select_strongest_detections(detections, max_count=TOP_OBJECTS)

            # Track only keys that are still active this frame.
            active_keys = set(detection_track_key(d) for d in strongest)
            state.keep_only_keys(active_keys)

            # ── Draw strongest detection boxes with per-object metrics ───────
            nearest_vz   = 0.0
            nearest_dist_for_speed = float("inf")
            for d in strongest:
                x, y, bw, bh = d["box"]
                dist = float(d.get("distance_m", 0.0))
                dist_for_calc = dist if dist > 0 else float("inf")

                obj_key = detection_track_key(d)

                # Read previous depth BEFORE updating state, then store current.
                prev_d = state.prev_object_distances.get(obj_key, None)
                state.prev_object_distances[obj_key] = dist_for_calc

                # Signed approach velocity: positive = coming toward camera.
                if prev_d is not None and prev_d < float("inf") and dist_for_calc < float("inf") and dt > 0:
                    vz_signed = (prev_d - dist_for_calc) / dt
                else:
                    vz_signed = 0.0

                obj_risk, obj_ttc = compute_ttc_risk(dist_for_calc, max(0.0, vz_signed))
                # RED carries higher danger — add 2 risk points (capped at 10)
                if d.get("label") == "RED":
                    obj_risk = min(10, obj_risk + 2)
                obj_level, obj_risk_color = risk_label(obj_risk)

                if dist_for_calc < nearest_dist_for_speed:
                    nearest_dist_for_speed = dist_for_calc
                    nearest_vz = vz_signed

                dist_txt = f"{dist:.2f} m" if dist_for_calc < float("inf") else "none"
                ttc_txt  = f"{obj_ttc:.2f} s" if obj_ttc < float("inf") else "none"
                vz_txt   = f"{vz_signed:+.2f} m/s"

                # BB color reflects per-object risk: green / yellow / red
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), obj_risk_color, 2)
                line1 = f"{d['label']}  dist: {dist_txt}  [{obj_risk}/10 {obj_level}]"
                line2 = f"approach: {vz_txt}  TTC: {ttc_txt}"
                draw_object_overlay(frame, x, y, bw, bh, obj_risk_color, line1, line2)

            # ── Risk from RED/BLACK detections only (modular) ───────────────
            nearest = nearest_distance_from_detections(strongest)

            # Use depth-only approach velocity for global risk.
            effective_speed = max(0.0, nearest_vz)
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
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, level_color, 1, cv2.LINE_AA)
            cv2.putText(frame, dist_txt,
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, ttc_txt,
                        (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, f"speed: {effective_speed:+.2f} m/s",
                        (10, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)

            # Show MIN_AREA as reminder of current sensitivity.
            cv2.putText(frame, f"MIN_AREA={MIN_AREA}px",
                        (10, 134), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.putText(frame, f"TOP_OBJECTS={TOP_OBJECTS}",
                        (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

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
        if pipeline:
            pipeline.stop()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
