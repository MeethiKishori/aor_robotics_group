import cv2
import numpy as np


# HSV = Hue, Saturation, Value.
#   H = color itself  (0-180 in OpenCV; red is near 0 AND near 180, so two ranges needed)
#   S = how vivid     (0=grey, 255=pure color)
#   V = brightness    (0=black, 255=bright)
#
# cv2.inRange keeps only pixels where all three values fall between LOWER and UPPER.
# Everything outside becomes 0 (black in the mask).

# Red range 1: hue 0-10 (orange-red side)
LOWER_RED_1 = np.array([0,   120,  70], dtype=np.uint8)   # H=0,  S=120, V=70
UPPER_RED_1 = np.array([10,  255, 255], dtype=np.uint8)   # H=10, S=255, V=255

# Red range 2: hue 170-180 (pink-red side, same physical red wraps around)
LOWER_RED_2 = np.array([170, 120,  70], dtype=np.uint8)   # H=170, S=120, V=70
UPPER_RED_2 = np.array([180, 255, 255], dtype=np.uint8)   # H=180, S=255, V=255

# Black: any hue (H=0-180), any saturation (S=0-255), but very low brightness (V=0-55)
LOWER_BLACK = np.array([0,   0,   0], dtype=np.uint8)     # H=any, S=any, V=0
UPPER_BLACK = np.array([180, 255, 55], dtype=np.uint8)    # H=any, S=any, V=30 (dark only) 55--> 30-->55 for more strict black

# Morphology kernel: 5x5 block used to clean up noisy masks.
# OPEN  = erode then dilate  -> removes small white dots (noise)
# CLOSE = dilate then erode  -> fills small holes inside objects
KERNEL = np.ones((5, 5), np.uint8)


def _mask_red(hsv_img):
    # Get a binary mask: white=red pixel, black=not red pixel.
    m1 = cv2.inRange(hsv_img, LOWER_RED_1, UPPER_RED_1)   # lower red hue range
    m2 = cv2.inRange(hsv_img, LOWER_RED_2, UPPER_RED_2)   # upper red hue range
    m = cv2.bitwise_or(m1, m2)                            # combine both: pixel is red if it's in either range
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  KERNEL)      # remove small noise dots
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, KERNEL)      # fill small holes
    return m


def _mask_black(hsv_img):
    # Get a binary mask: white=black pixel, black=not black pixel.
    m = cv2.inRange(hsv_img, LOWER_BLACK, UPPER_BLACK)    # low brightness = black
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  KERNEL)      # remove small noise dots
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, KERNEL)      # fill small holes
    return m


def _depth_median(depth_frame, x, y, w, h, width, height):
    # Compute median depth of a small patch at the center of the bounding box.
    # Using median instead of single pixel avoids outlier/invalid depth values.
    cx = x + w // 2   # center x of the bounding box
    cy = y + h // 2   # center y of the bounding box

    # Patch size = 1/5 of box size, minimum 3 pixels.
    pw = max(3, w // 5)
    ph = max(3, h // 5)

    # Patch corners, clamped to image boundaries.
    x1 = max(0, cx - pw // 2)
    x2 = min(width - 1, cx + pw // 2)
    y1 = max(0, cy - ph // 2)
    y2 = min(height - 1, cy + ph // 2)

    vals = []
    for yy in range(y1, y2 + 1):
        for xx in range(x1, x2 + 1):
            d = depth_frame.get_distance(xx, yy)   # returns metres, 0.0 if invalid
            if d > 0:
                vals.append(d)

    if not vals:
        return 0.0                       # no valid depth in patch
    return float(np.median(vals))        # median distance in metres


def _extract(mask, depth_frame, min_area, x_off, y_off, full_w, full_h, name, color):
    # Find all separate white blobs in the mask and convert each to a detection dict.
    detections = []

    # findContours returns outlines of white regions.
    # RETR_EXTERNAL: only outer contours (ignore holes inside blobs).
    # CHAIN_APPROX_SIMPLE: store only corner points, not every point on edge.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    frame_area = float(full_w * full_h)   # total pixels in full frame (for % calculation)

    for c in contours:
        area = cv2.contourArea(c)   # number of pixels inside this contour
        if area < min_area:
            continue                # skip objects that are too small

        x, y, w, h = cv2.boundingRect(c)        # get bounding box of the contour
        gx, gy = x + x_off, y + y_off           # convert ROI-local coords to full-frame coords

        dist_m = _depth_median(depth_frame, gx, gy, w, h, full_w, full_h)
        area_pct = 100.0 * area / frame_area    # how many % of full image this blob covers

        detections.append({
            "label":      name,
            "box":        (gx, gy, w, h),        # (x, y, width, height) in full-frame coords
            "area":       float(area),            # blob area in pixels
            "area_pct":   float(area_pct),        # blob area as % of full frame
            "distance_m": float(dist_m),          # median distance in metres
            "draw_color": color,                  # BGR color for drawing the box
        })

    return detections


def detect_red_black_in_roi(frame_bgr, depth_frame, roi_rect, min_area):
    # Main function called by the launch script each frame.
    # Returns a list of detection dicts for red and black objects found inside the ROI.
    x1, y1, x2, y2 = roi_rect
    roi = frame_bgr[y1:y2, x1:x2]   # crop full frame to ROI region only

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)   # convert cropped region to HSV
    red_mask   = _mask_red(hsv_roi)
    black_mask = _mask_black(hsv_roi)

    h, w = frame_bgr.shape[:2]   # full frame dimensions for coordinate conversion
    out = []
    out.extend(_extract(red_mask,   depth_frame, min_area, x1, y1, w, h, "RED",   (0, 0, 255)))
    out.extend(_extract(black_mask, depth_frame, min_area, x1, y1, w, h, "BLACK", (0, 255, 255)))
    return out
