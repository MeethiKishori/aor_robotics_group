import cv2
import numpy as np


# HSV = Hue, Saturation, Value.
#   H = color itself  (0-180 in OpenCV; red is near 0 AND near 180, so two ranges needed)
#   S = how vivid - saturation    (0=grey, 255=pure color)
#   V = brightness    (0=black, 255=bright)
#
# cv2.inRange keeps only pixels where all three values fall between LOWER and UPPER. white is true
# Everything outside becomes 0 (black in the mask).

# Red range 1: hue 0-10 (orange-red side)
LOWER_RED_1 = np.array([0,   120,  70], dtype=np.uint8)
UPPER_RED_1 = np.array([10,  255, 255], dtype=np.uint8)

# Red range 2: hue 170-180 (pink-red side, same physical red wraps around)
LOWER_RED_2 = np.array([170, 120,  70], dtype=np.uint8)
UPPER_RED_2 = np.array([180, 255, 255], dtype=np.uint8)

# Black: any hue, any saturation, but very low brightness
LOWER_BLACK = np.array([0,   0,   0], dtype=np.uint8)
UPPER_BLACK = np.array([180, 255, 55], dtype=np.uint8)

# Morphology kernel — removes noise dots and fills small holes in masks
KERNEL = np.ones((5, 5), np.uint8)


def _mask_red(hsv_img, lower1=None, upper1=None, lower2=None, upper2=None):
    l1 = lower1 if lower1 is not None else LOWER_RED_1
    u1 = upper1 if upper1 is not None else UPPER_RED_1
    l2 = lower2 if lower2 is not None else LOWER_RED_2
    u2 = upper2 if upper2 is not None else UPPER_RED_2
    m = cv2.bitwise_or(cv2.inRange(hsv_img, l1, u1), cv2.inRange(hsv_img, l2, u2))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  KERNEL)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, KERNEL)
    return m


def _mask_black(hsv_img, lower=None, upper=None):
    l = lower if lower is not None else LOWER_BLACK
    u = upper if upper is not None else UPPER_BLACK
    m = cv2.inRange(hsv_img, l, u)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  KERNEL)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, KERNEL)
    return m


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
