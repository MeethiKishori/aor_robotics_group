import os
import sys
import argparse

import cv2
import numpy as np

# Allow imports from ../perception when run as a script. it helps in running script from anywhere without worrying is im at correct cd 
THIS_DIR   = os.path.dirname(os.path.abspath(__file__)) # curret file to absolute path from root , keeps only the folder name uptill launch THIS_DIR: "/home/ssingh/.../realsense_camera/python/launch"
PYTHON_DIR = os.path.dirname(THIS_DIR)  # it takes the directory name just one folder up PYTHON_DIR: "/home/ssingh/.../realsense_camera/python"
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from perception.red_black_roi import (
    detect_red_black_in_roi,
    LOWER_RED_1, UPPER_RED_1, LOWER_RED_2, UPPER_RED_2,
    LOWER_BLACK, UPPER_BLACK,
)
from actuators.tower import SignalTowerController
from camera.realsense_stream import start_aligned_pipeline
from risk.risk_score import (
    compute_ttc_risk,
    nearest_distance_from_detections,
    risk_label,
    select_strongest_detections,
)
from risk.runtime import RiskRuntimeState


# ──────────────────────────────────────────────
# HSV SLIDER WINDOWS
# ──────────────────────────────────────────────

WIN_RED   = 'Tune RED'
WIN_BLACK = 'Tune BLACK'


def create_sliders():
    cv2.namedWindow(WIN_RED, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_RED, 700, 700)
    cv2.createTrackbar('Hue1-Low  orange-red (0-20)',  WIN_RED, int(LOWER_RED_1[0]), 20,  lambda x: None)
    cv2.createTrackbar('Hue2-Low  pink-red  (0-180)',  WIN_RED, int(LOWER_RED_2[0]), 180, lambda x: None)
    cv2.createTrackbar('Sat-Min   vividness (0-255)',  WIN_RED, int(LOWER_RED_1[1]), 255, lambda x: None)
    cv2.createTrackbar('Val-Min   brightness(0-255)',  WIN_RED, int(LOWER_RED_1[2]), 255, lambda x: None)

    cv2.namedWindow(WIN_BLACK, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_BLACK, 700, 700)
    cv2.createTrackbar('Sat-Max   vividness (0-255)',  WIN_BLACK, int(UPPER_BLACK[1]), 255, lambda x: None)
    cv2.createTrackbar('Val-Max   darkness  (0-100)',  WIN_BLACK, int(UPPER_BLACK[2]), 100, lambda x: None)
    cv2.createTrackbar('Min-Area  blob size (0-5000)', WIN_BLACK, 300,                5000, lambda x: None)

    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(blank, "Waiting for camera...", (60, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
    cv2.imshow(WIN_RED,   blank)
    cv2.imshow(WIN_BLACK, blank)
    cv2.waitKey(1)


def read_sliders():
    h_l1     = cv2.getTrackbarPos('Hue1-Low  orange-red (0-20)',  WIN_RED)
    h_l2     = cv2.getTrackbarPos('Hue2-Low  pink-red  (0-180)',  WIN_RED)
    s_min    = cv2.getTrackbarPos('Sat-Min   vividness (0-255)',  WIN_RED)
    v_min    = cv2.getTrackbarPos('Val-Min   brightness(0-255)',  WIN_RED)
    b_sat    = cv2.getTrackbarPos('Sat-Max   vividness (0-255)',  WIN_BLACK)
    b_val    = cv2.getTrackbarPos('Val-Max   darkness  (0-100)',  WIN_BLACK)
    min_area = cv2.getTrackbarPos('Min-Area  blob size (0-5000)', WIN_BLACK)
    return h_l1, h_l2, s_min, v_min, b_sat, b_val, max(1, min_area)


def show_masks(red_mask, black_mask, h_l1, h_l2, s_min, v_min, b_sat, b_val, min_area):
    H, W = red_mask.shape[:2]

    red_disp = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
    for i, t in enumerate([
        f"Hue1-Low = {h_l1:3d}  | orange-red lower edge. Raise to cut out orange.",
        f"Hue2-Low = {h_l2:3d}  | pink-red  lower edge. Raise to cut out magenta.",
        f"Sat-Min  = {s_min:3d}  | min vividness. Raise to reject washed-out/grey red.",
        f"Val-Min  = {v_min:3d}  | min brightness. Raise to reject dark/shadowed red.",
    ]):
        cv2.putText(red_disp, t, (6, 28 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 80, 255), 1, cv2.LINE_AA)
    cv2.imshow(WIN_RED, red_disp)

    blk_disp = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
    for i, t in enumerate([
        f"Sat-Max  = {b_sat:3d}  | max vividness. Lower to exclude coloured objects.",
        f"Val-Max  = {b_val:3d}  | max brightness. Lower to detect only very dark pixels.",
        f"Min-Area = {min_area:4d} | min blob size. Raise to ignore small noise dots.",
    ]):
        cv2.putText(blk_disp, t, (6, 28 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow(WIN_BLACK, blk_disp)


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
ROI_FRAC = 0.70        # fraction of image used as center ROI (0.30 to 0.50)
MIN_AREA = 10000       # minimum blob pixel area to count as a detection (increase to ignore small blobs)
TOP_OBJECTS = 3        # draw/analyze only top strongest detections
ASSUME_STABLE_CAMERA = True  # True: ignore IMU speed (avoids drift), use object closure only
VELOCITY_DEAD_BAND = 0.05  # m/s — ignore velocity below this to suppress depth jitter on stationary objects
TOWER_PORT = "/dev/ttyUSB0"
TOWER_BAUD = 9600


# ──────────────────────────────────────────────
# TOWER STATE HELPER
# ──────────────────────────────────────────────

def set_tower_state(tower, risk_level):
    """Map risk level to tower light, buzzer, and flash states."""
    if risk_level == "LOW":
        tower.green()
        #tower.buzzer(False)
        #tower.flash(1)  # 1 = continuous/solid
    elif risk_level == "MODERATE":
        tower.yellow()
        tower.buzzer(False)
        #tower.flash(3)  # 3 = fast flash
    elif risk_level == "DANGER":
        tower.red()
        #tower.buzzer(True)
       # tower.flash(3)  # 3 = fast flash


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
# MAIN LOOP
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Run red/black ROI risk pipeline")
    parser.add_argument("--color-width",  type=int, default=COLOR_WIDTH)
    parser.add_argument("--color-height", type=int, default=COLOR_HEIGHT)
    parser.add_argument("--depth-width",  type=int, default=DEPTH_WIDTH)
    parser.add_argument("--depth-height", type=int, default=DEPTH_HEIGHT)
    parser.add_argument("--fps",          type=int, default=FPS)
    parser.add_argument("--tower-port", default=TOWER_PORT,
                        help="Serial device for tower light/buzzer (empty disables it)")
    parser.add_argument("--tower-baud", type=int, default=TOWER_BAUD,
                        help="Baud rate for the tower light/buzzer")
    args = parser.parse_args()

    color_w = max(320, args.color_width)
    color_h = max(240, args.color_height)
    fps     = max(1, args.fps)

    pipeline = align = None
    tower = None
    try:
        tower = SignalTowerController(port=args.tower_port, baudrate=args.tower_baud)
    except Exception as e:
        print(f"Warning: Failed to initialize tower controller on {args.tower_port}: {e}")
        print("Tower control disabled; risk pipeline will run without LED/buzzer output.")

    depth_w = max(320, args.depth_width)
    depth_h = max(240, args.depth_height)
    pipeline, align = start_aligned_pipeline(color_w, color_h, depth_w, depth_h, fps)

    state   = RiskRuntimeState()

    create_sliders()

    mode_label = f"LIVE {color_w}x{color_h}"
    print(f"Running: {mode_label} | Press q to quit.")

    try:
        while True:
            # ── Live camera frame ─────────────────────────────────────
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame     = np.asanyarray(color_frame.get_data())
            depth_src = depth_frame
            h, w = frame.shape[:2]
            dt = state.next_dt()
            roi   = roi_rect_from_frac(w, h, ROI_FRAC)
            x1, y1, x2, y2 = roi

            # Read slider values and build HSV override arrays
            h_l1, h_l2, s_min, v_min, b_sat, b_val, min_area = read_sliders()
            red_lower1 = np.array([h_l1, s_min, v_min], dtype=np.uint8)
            red_upper1 = np.array([20,   255,   255],   dtype=np.uint8)
            red_lower2 = np.array([h_l2, s_min, v_min], dtype=np.uint8)
            red_upper2 = np.array([180,  255,   255],   dtype=np.uint8)
            blk_lower  = np.array([0,    0,     0],     dtype=np.uint8)
            blk_upper  = np.array([180,  b_sat, b_val], dtype=np.uint8)

            detections, red_mask, black_mask = detect_red_black_in_roi(
                frame, depth_src, roi, min_area,
                red_lower1, red_upper1, red_lower2, red_upper2,
                blk_lower, blk_upper,
            )
            show_masks(red_mask, black_mask, h_l1, h_l2, s_min, v_min, b_sat, b_val, min_area)

            # ── Draw ROI box ─────────────────────────────────────────────────
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"ROI {int(ROI_FRAC * 100)}%",
                        (x1 + 5, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)

            # Filter by min area, then pick top-1 RED and top-1 BLACK separately.
            detections = [d for d in detections if d["area"] >= min_area]
            top_red   = max((d for d in detections if d["label"] == "RED"),
                            key=lambda d: d["area"], default=None)
            top_black = max((d for d in detections if d["label"] == "BLACK"),
                            key=lambda d: d["area"], default=None)
            show_two  = [d for d in [top_red, top_black] if d is not None]

            active_keys = set(detection_track_key(d) for d in show_two)
            state.keep_only_keys(active_keys)

            # ── Per-detection velocity, risk, draw ───────────────────────────
            nearest_vz             = 0.0
            nearest_dist_for_speed = float("inf")

            for d in show_two:
                x, y, bw, bh = d["box"]
                dist          = float(d.get("distance_m", 0.0))
                dist_for_calc = dist if dist > 0 else float("inf")
                obj_key       = detection_track_key(d)

                prev_d = state.prev_object_distances.get(obj_key, None)
                state.prev_object_distances[obj_key] = dist_for_calc

                if prev_d is not None and prev_d < float("inf") and dist_for_calc < float("inf") and dt > 0:
                    raw_vz = (prev_d - dist_for_calc) / dt
                else:
                    raw_vz = 0.0

                # EMA smoothing kills jitter; dead-band zeroes residual noise.
                vz_signed = state.smooth_velocity(obj_key, raw_vz)
                if abs(vz_signed) < VELOCITY_DEAD_BAND:
                    vz_signed = 0.0

                obj_risk, obj_ttc = compute_ttc_risk(dist_for_calc, max(0.0, vz_signed))
                if d["label"] == "RED":
                    obj_risk = min(10, obj_risk + 2)
                obj_level, _ = risk_label(obj_risk)

                if dist_for_calc < nearest_dist_for_speed:
                    nearest_dist_for_speed = dist_for_calc
                    nearest_vz = vz_signed

                dist_txt = f"{dist:.2f} m" if dist_for_calc < float("inf") else "none"
                ttc_txt  = f"{obj_ttc:.2f} s" if obj_ttc < float("inf") else "none"
                vz_txt   = f"{vz_signed:+.2f} m/s"

                # Fixed colours per label so RED is always red, BLACK always cyan.
                box_color = (0, 0, 255) if d["label"] == "RED" else (255, 255, 0)
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), box_color, 3)
                cx, cy = x + bw // 2, y + bh // 2
                cv2.drawMarker(frame, (cx, cy), box_color, cv2.MARKER_CROSS, 20, 2)
                line1 = f"{d['label']}  dist: {dist_txt}  [{obj_risk}/10 {obj_level}]"
                line2 = f"approach: {vz_txt}  TTC: {ttc_txt}"
                draw_object_overlay(frame, x, y, bw, bh, box_color, line1, line2)

            # ── Global risk from both detections ─────────────────────────────
            nearest = nearest_distance_from_detections(show_two)
            effective_speed = max(0.0, nearest_vz)
            risk, ttc = compute_ttc_risk(nearest, effective_speed)
            level, level_color = risk_label(risk)
            if tower:
                set_tower_state(tower, level)

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

            # Show current sensitivity from slider.
            cv2.putText(frame, f"MIN_AREA={min_area}px",
                        (10, 134), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.putText(frame, f"showing top-1 RED + top-1 BLACK ({len(show_two)} visible)",
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
        if tower:
            tower.close()
        if pipeline:
            pipeline.stop()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
