import numpy as np
import cv2
import pyrealsense2 as rs

# ======================
# Easy-to-change settings
# ======================
WIDTH = 640
HEIGHT = 480
FPS = 30

# Ignore tiny objects. Increase if you only want bigger objects.
MIN_AREA = 10000  # pixels

# HSV ranges (tune later if needed)
# Red wraps around hue, so use two ranges.
LOWER_RED_1 = np.array([0, 120, 70], dtype=np.uint8)
UPPER_RED_1 = np.array([10, 255, 255], dtype=np.uint8)
LOWER_RED_2 = np.array([170, 120, 70], dtype=np.uint8)
UPPER_RED_2 = np.array([180, 255, 255], dtype=np.uint8)

# Black: low brightness (V). Keep S open so dark gray/black can be included.
LOWER_BLACK = np.array([0, 0, 0], dtype=np.uint8)
UPPER_BLACK = np.array([180, 255, 55], dtype=np.uint8)


def robust_distance_m(depth_frame, x, y, w, h):
    """Estimate distance using median depth in a small center patch of the box."""
    cx = x + w // 2
    cy = y + h // 2

    pw = max(3, w // 5)
    ph = max(3, h // 5)
    x1 = max(0, cx - pw // 2)
    y1 = max(0, cy - ph // 2)
    x2 = min(WIDTH - 1, cx + pw // 2)
    y2 = min(HEIGHT - 1, cy + ph // 2)

    vals = []
    for yy in range(y1, y2 + 1):
        for xx in range(x1, x2 + 1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)

    if not vals:
        return 0.0
    return float(np.median(vals))


def draw_detections(mask, frame, depth_frame, name, box_color):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    frame_area = frame.shape[0] * frame.shape[1]

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(c)
        dist_m = robust_distance_m(depth_frame, x, y, w, h)
        area_pct = 100.0 * area / frame_area

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
        label = f"{name} A:{int(area)} ({area_pct:.1f}%) D:{dist_m:.2f}m"
        cv2.putText(frame, label, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2, cv2.LINE_AA)


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Clean binary masks to reduce small noisy pixels.
    kernel = np.ones((5, 5), np.uint8)

    print("Detecting red and black objects. Press q to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            red_mask_1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
            red_mask_2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
            red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

            black_mask = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)

            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

            draw_detections(red_mask, frame, depth_frame, "RED", (0, 0, 255))
            draw_detections(black_mask, frame, depth_frame, "BLACK", (0, 255, 255))

            cv2.putText(frame, f"MIN_AREA={MIN_AREA}px", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("Red/Black Detection + Distance", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
