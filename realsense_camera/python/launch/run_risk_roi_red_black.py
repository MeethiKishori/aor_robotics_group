import os, sys
import cv2
import numpy as np

THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(THIS_DIR)
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from perception.red_black_roi import detect_red_black_in_roi, load_hsv_config, save_hsv_config
from actuators.tower import SignalTowerController
from camera.realsense_stream import start_aligned_pipeline
from risk.risk_score import compute_ttc_risk, nearest_distance_from_detections, risk_label
from risk.runtime import RiskRuntimeState

WIN_RED   = 'Tune RED'
WIN_BLACK = 'Tune BLACK'

COLOR_WIDTH,  COLOR_HEIGHT = 1920, 1080
DEPTH_WIDTH,  DEPTH_HEIGHT = 640,  480
FPS               = 30
ROI_FRAC          = 0.70
MIN_AREA          = 10000
VELOCITY_DEAD_BAND = 0.05
TOWER_PORT        = "/dev/ttyUSB0"
TOWER_BAUD        = 9600
LABEL_COLOR       = {"RED": (0, 0, 255), "BLACK": (255, 255, 0)}


def create_sliders(r, b, d):
    for win in (WIN_RED, WIN_BLACK):
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 700, 700)
    cv2.createTrackbar('Hue1-Low  orange-red (0-20)',  WIN_RED,   r["hue1_low"], 20,   lambda x: None)
    cv2.createTrackbar('Hue2-Low  pink-red  (0-180)',  WIN_RED,   r["hue2_low"], 180,  lambda x: None)
    cv2.createTrackbar('Sat-Min   vividness (0-255)',  WIN_RED,   r["sat_min"],  255,  lambda x: None)
    cv2.createTrackbar('Val-Min   brightness(0-255)',  WIN_RED,   r["val_min"],  255,  lambda x: None)
    cv2.createTrackbar('Sat-Max   vividness (0-255)',  WIN_BLACK, b["sat_max"],  255,  lambda x: None)
    cv2.createTrackbar('Val-Max   darkness  (0-100)',  WIN_BLACK, b["val_max"],  100,  lambda x: None)
    cv2.createTrackbar('Min-Area  blob size (0-5000)', WIN_BLACK, d["min_area"], 5000, lambda x: None)


def read_sliders():
    return (
        cv2.getTrackbarPos('Hue1-Low  orange-red (0-20)',  WIN_RED),
        cv2.getTrackbarPos('Hue2-Low  pink-red  (0-180)',  WIN_RED),
        cv2.getTrackbarPos('Sat-Min   vividness (0-255)',  WIN_RED),
        cv2.getTrackbarPos('Val-Min   brightness(0-255)',  WIN_RED),
        cv2.getTrackbarPos('Sat-Max   vividness (0-255)',  WIN_BLACK),
        cv2.getTrackbarPos('Val-Max   darkness  (0-100)',  WIN_BLACK),
        max(1, cv2.getTrackbarPos('Min-Area  blob size (0-5000)', WIN_BLACK)),
    )


def obj_key(d, grid=80):
    x, y, w, h = d["box"]
    return f"{d['label']}:{(x + w//2)//grid}:{(y + h//2)//grid}"


def set_tower_state(tower, level):
    if level == "LOW":
        tower.green()
    elif level == "MODERATE":
        tower.yellow()
        tower.buzzer(False)
    elif level == "DANGER":
        tower.red()


def main():
    tower = None
    try:
        tower = SignalTowerController(port=TOWER_PORT, baudrate=TOWER_BAUD)
    except Exception as e:
        print(f"Warning: tower not available: {e}")

    pipeline, align = start_aligned_pipeline(
        COLOR_WIDTH, COLOR_HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, FPS
    )
    color_w, color_h = COLOR_WIDTH, COLOR_HEIGHT
    state = RiskRuntimeState()

    r_cfg, b_cfg, d_cfg = load_hsv_config()
    create_sliders(r_cfg, b_cfg, d_cfg)

    # ROI rectangle (center-aligned)
    rw, rh   = int(color_w * ROI_FRAC), int(color_h * ROI_FRAC)
    x1, y1   = (color_w - rw) // 2, (color_h - rh) // 2
    x2, y2   = x1 + rw, y1 + rh
    roi      = (x1, y1, x2, y2)

    # Constant HSV bounds (never change at runtime)
    red_upper1 = np.array([r_cfg["hue1_high"], r_cfg["sat_max"], r_cfg["val_max"]], dtype=np.uint8)
    red_upper2 = np.array([r_cfg["hue2_high"], r_cfg["sat_max"], r_cfg["val_max"]], dtype=np.uint8)
    blk_lower  = np.array([b_cfg["hue_low"],   b_cfg["sat_min"], b_cfg["val_min"]], dtype=np.uint8)
    prev_sv    = None
    red_lower1 = red_lower2 = blk_upper = None

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
            dt    = state.next_dt()

            h_l1, h_l2, s_min, v_min, b_sat, b_val, min_area = read_sliders()
            sv = (h_l1, h_l2, s_min, v_min, b_sat, b_val)
            if sv != prev_sv:
                red_lower1 = np.array([h_l1, s_min, v_min],              dtype=np.uint8)
                red_lower2 = np.array([h_l2, s_min, v_min],              dtype=np.uint8)
                blk_upper  = np.array([b_cfg["hue_high"], b_sat, b_val], dtype=np.uint8)
                prev_sv    = sv

            detections, red_mask, black_mask = detect_red_black_in_roi(
                frame, depth_frame, roi, min_area,
                red_lower1, red_upper1, red_lower2, red_upper2, blk_lower, blk_upper,
            )
            cv2.imshow(WIN_RED,   red_mask)
            cv2.imshow(WIN_BLACK, black_mask)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            # Top-1 RED + top-1 BLACK by area
            detections = [d for d in detections if d["area"] >= min_area]
            top_red = top_black = None
            for d in detections:
                if d["label"] == "RED"   and (top_red   is None or d["area"] > top_red["area"]):   top_red   = d
                if d["label"] == "BLACK" and (top_black is None or d["area"] > top_black["area"]): top_black = d
            show_two = [d for d in [top_red, top_black] if d is not None]
            state.keep_only_keys({obj_key(d) for d in show_two})

            nearest_vz   = 0.0
            nearest_dist = float("inf")

            for d in show_two:
                x, y, bw, bh = d["box"]
                dist  = float(d.get("distance_m", 0.0))
                dcalc = dist if dist > 0 else float("inf")
                k     = obj_key(d)

                prev_d = state.prev_object_distances.get(k, float("inf"))
                state.prev_object_distances[k] = dcalc
                raw_vz = (prev_d - dcalc) / dt if prev_d < float("inf") and dcalc < float("inf") and dt > 0 else 0.0
                vz = state.smooth_velocity(k, raw_vz)
                if abs(vz) < VELOCITY_DEAD_BAND:
                    vz = 0.0

                obj_risk, _ = compute_ttc_risk(dcalc, max(0.0, vz))
                if d["label"] == "RED":
                    obj_risk = min(10, obj_risk + 2)
                obj_level, _ = risk_label(obj_risk)

                if dcalc < nearest_dist:
                    nearest_dist, nearest_vz = dcalc, vz

                color = LABEL_COLOR[d["label"]]
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 3)
                dist_s = f"{dist:.2f}m" if dist > 0 else "?"
                cv2.putText(frame, f"{d['label']}  {dist_s}  [{obj_risk}/10 {obj_level}]",
                            (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

            risk, ttc      = compute_ttc_risk(nearest_dist, max(0.0, nearest_vz))
            level, l_color = risk_label(risk)
            if tower:
                set_tower_state(tower, level)

            dist_s = f"{nearest_dist:.2f}m" if nearest_dist < float("inf") else "none"
            ttc_s  = f"{ttc:.1f}s"          if ttc           < float("inf") else "none"
            cv2.putText(frame, f"RISK {risk}/10 {level}  dist={dist_s}  ttc={ttc_s}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, l_color, 2, cv2.LINE_AA)
            print(f"risk={risk}/10 {level}  dist={dist_s}  ttc={ttc_s}    \r", end="", flush=True)

            cv2.imshow("Risk ROI", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        try:
            h_l1, h_l2, s_min, v_min, b_sat, b_val, min_area = read_sliders()
            r_cfg.update({"hue1_low": h_l1, "hue2_low": h_l2, "sat_min": s_min, "val_min": v_min})
            b_cfg.update({"sat_max": b_sat, "val_max": b_val})
            d_cfg["min_area"] = min_area
            save_hsv_config(r_cfg, b_cfg, d_cfg)
        except Exception as exc:
            print(f"\nWarning: could not save HSV config: {exc}")
        if tower:    tower.close()
        if pipeline: pipeline.stop()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
