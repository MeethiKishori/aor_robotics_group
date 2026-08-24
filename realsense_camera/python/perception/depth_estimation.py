import numpy as np


def median_depth_in_box(depth_frame, box, frame_width, frame_height, sample_frac=0.2):
    """Median depth (metres) over a small patch at the centre of a box, for noise robustness."""
    x, y, w, h = box
    cx, cy = x + w // 2, y + h // 2
    pw = max(3, int(w * sample_frac))
    ph = max(3, int(h * sample_frac))
    x1 = max(0, cx - pw // 2)
    x2 = min(frame_width - 1, cx + pw // 2)
    y1 = max(0, cy - ph // 2)
    y2 = min(frame_height - 1, cy + ph // 2)

    vals = []
    for yy in range(y1, y2 + 1):
        for xx in range(x1, x2 + 1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0


def attach_distances(detections, depth_frame, frame_width, frame_height):
    """Compute and set distance_m on each detection dict in place. Returns the same list."""
    for det in detections:
        det["distance_m"] = median_depth_in_box(depth_frame, det["box"], frame_width, frame_height)
    return detections
