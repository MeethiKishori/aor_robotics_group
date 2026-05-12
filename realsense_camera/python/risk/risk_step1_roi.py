import numpy as np
import pyrealsense2 as rs
import cv2

# ── ROI size: fraction of image width/height (change this to tune) ──
ROI_FRAC = 0.35   # 0.30 = 30%, 0.35 = 35%, 0.40 = 40%
# ────────────────────────────────────────────────────────────────────

pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Live ROI view — press Q to quit")

while True:
    frames      = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    img = np.asanyarray(color_frame.get_data())   # (480, 640, 3) BGR

    h, w = img.shape[:2]

    # ── compute ROI corners centred in the image ─────────────────────
    roi_h = int(h * ROI_FRAC)
    roi_w = int(w * ROI_FRAC)
    y1 = (h - roi_h) // 2
    y2 = y1 + roi_h
    x1 = (w - roi_w) // 2
    x2 = x1 + roi_w
    # ─────────────────────────────────────────────────────────────────

    # draw red rectangle for ROI
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # small label so you know what the box is
    cv2.putText(img, f"ROI {int(ROI_FRAC*100)}%",
                (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 0, 255), 1, cv2.LINE_AA)

    cv2.imshow("ROI", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
