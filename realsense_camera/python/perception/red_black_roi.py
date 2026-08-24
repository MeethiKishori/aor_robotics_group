import os
import cv2
import numpy as np
import yaml


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
HSV_CONFIG_PATH = os.path.join(THIS_DIR, "hsv_config.yaml")

# HSV = Hue, Saturation, Value.
#   H = color itself  (0-180 in OpenCV; red is near 0 AND near 180, so two ranges needed)
#   S = how vivid     (0=grey, 255=pure color)
#   V = brightness    (0=black, 255=bright)



# Fallback values used only when hsv_config.yaml does not exist yet.
# Edit hsv_config.yaml (created on first quit) to change any of these.
# Values marked (slider) are also overridden live by the slider windows.
_DEFAULTS = {  # dictionary of default HSV config values; also serves as a template for the YAML file structure
    "red": {
        "hue1_low":  0,    # orange-red lower hue edge  (slider)
        "hue1_high": 10,   # orange-red upper hue edge
        "hue2_low":  170,  # pink-red   lower hue edge  (slider)
        "hue2_high": 180,  # pink-red   upper hue edge
        "sat_min":   120,  # min saturation             (slider)
        "sat_max":   255,  # max saturation
        "val_min":   70,   # min brightness             (slider)
        "val_max":   255,  # max brightness
    },
    "black": {
        "hue_low":  0,     # lower hue (any hue counts as black)
        "hue_high": 180,   # upper hue
        "sat_min":  0,     # min saturation
        "sat_max":  255,   # max saturation             (slider)
        "val_min":  0,     # min brightness
        "val_max":  55,    # max brightness / darkness  (slider)
    },
    "detection": {
        "min_area": 300,   # minimum blob pixel area    (slider)
    },
}


def load_hsv_config():
    with open(HSV_CONFIG_PATH, "r") as f: # returns sting content of the file
        cfg = yaml.safe_load(f) or {} # parse YAML string into Python dict; if file is empty, use empty dict
    r = {**_DEFAULTS["red"],       **cfg.get("red", {})}
    b = {**_DEFAULTS["black"],     **cfg.get("black", {})}
    d = {**_DEFAULTS["detection"], **cfg.get("detection", {})}
    return r, b, d


def save_hsv_config(r_cfg, b_cfg, d_cfg):
    cfg = {"red": dict(r_cfg), "black": dict(b_cfg), "detection": dict(d_cfg)}
    with open(HSV_CONFIG_PATH, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False)
    print(f"HSV config saved to {HSV_CONFIG_PATH}")


# Load YAML once at import; mask functions use these as defaults.
_r_cfg, _b_cfg, _d_cfg = load_hsv_config()


def _mask_red(hsv_img, lower1=None, upper1=None, lower2=None, upper2=None):
    l1 = lower1 if lower1 is not None else np.array([_r_cfg["hue1_low"],  _r_cfg["sat_min"], _r_cfg["val_min"]], dtype=np.uint8)
    u1 = upper1 if upper1 is not None else np.array([_r_cfg["hue1_high"], _r_cfg["sat_max"], _r_cfg["val_max"]], dtype=np.uint8)
    l2 = lower2 if lower2 is not None else np.array([_r_cfg["hue2_low"],  _r_cfg["sat_min"], _r_cfg["val_min"]], dtype=np.uint8)
    u2 = upper2 if upper2 is not None else np.array([_r_cfg["hue2_high"], _r_cfg["sat_max"], _r_cfg["val_max"]], dtype=np.uint8)
    return cv2.bitwise_or(cv2.inRange(hsv_img, l1, u1), cv2.inRange(hsv_img, l2, u2))


def _mask_black(hsv_img, lower=None, upper=None):
    l = lower if lower is not None else np.array([_b_cfg["hue_low"],  _b_cfg["sat_min"], _b_cfg["val_min"]], dtype=np.uint8)
    u = upper if upper is not None else np.array([_b_cfg["hue_high"], _b_cfg["sat_max"], _b_cfg["val_max"]], dtype=np.uint8)
    return cv2.inRange(hsv_img, l, u)


def _depth_median(depth_frame, x, y, w, h, width, height):
    cx = x + w // 2
    cy = y + h // 2
    pw = max(3, w // 5)
    ph = max(3, h // 5)
    x1 = max(0, cx - pw // 2)
    x2 = min(width - 1, cx + pw // 2)
    y1 = max(0, cy - ph // 2)
    y2 = min(height - 1, cy + ph // 2)
    vals = []
    for yy in range(y1, y2 + 1):
        for xx in range(x1, x2 + 1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0


def _extract(mask, depth_frame, min_area, x_off, y_off, full_w, full_h, name, color):
    detections = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(full_w * full_h)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        gx, gy = x + x_off, y + y_off
        detections.append({
            "label":      name,
            "box":        (gx, gy, w, h),
            "area":       float(area),
            "area_pct":   100.0 * area / frame_area,
            "distance_m": float(_depth_median(depth_frame, gx, gy, w, h, full_w, full_h)),
            "draw_color": color,
        })
    return detections


def detect_red_black_in_roi(frame_bgr, depth_frame, roi_rect, min_area,
                             red_lower1=None, red_upper1=None,
                             red_lower2=None, red_upper2=None,
                             black_lower=None, black_upper=None):
    x1, y1, x2, y2 = roi_rect
    roi     = frame_bgr[y1:y2, x1:x2]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red_mask   = _mask_red(hsv_roi, red_lower1, red_upper1, red_lower2, red_upper2)
    black_mask = _mask_black(hsv_roi, black_lower, black_upper)

    h, w = frame_bgr.shape[:2]
    out = []
    out.extend(_extract(red_mask,   depth_frame, min_area, x1, y1, w, h, "RED",   (0, 0, 255)))
    out.extend(_extract(black_mask, depth_frame, min_area, x1, y1, w, h, "BLACK", (0, 255, 255)))
    return out, red_mask, black_mask
