import argparse
import time

import cv2
import numpy as np


def make_white_background(width, height):
    return np.full((height, width, 3), 255, dtype=np.uint8)


def draw_scene(base_bg, object_scale, approach_scale):
    frame = base_bg.copy()
    h, w = frame.shape[:2]

    # Single black circle: changes size over time to simulate near/far motion.
    br3 = max(20, int(55 * object_scale * approach_scale))
    bx3 = int(w * 0.50)
    by3 = int(h * 0.50)
    cv2.circle(frame, (bx3, by3), br3, (0, 0, 0), -1)

    return frame


def main():
    parser = argparse.ArgumentParser(description="Synthetic single black object approach test scene")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=0.8, help="Approach oscillation speed")
    parser.add_argument("--object-scale", type=float, default=1.0, help="Base object scale")
    args = parser.parse_args()

    width = max(320, args.width)
    height = max(240, args.height)
    fps = max(1.0, args.fps)
    speed = max(0.0, args.speed)
    object_scale = max(0.4, args.object_scale)

    bg = make_white_background(width, height)

    approach_scale = 1.0
    approach_dir = 1.0

    last_t = time.time()

    print("Synthetic single-object scene running.")
    print("Controls: +/- speed, [ ] size, q quit")

    while True:
        now = time.time()
        dt = max(now - last_t, 1e-3)
        last_t = now

        # Simulate near/far by changing apparent size.
        approach_scale += approach_dir * speed * dt
        if approach_scale > 2.6:
            approach_scale = 2.6
            approach_dir = -1.0
        if approach_scale < 0.8:
            approach_scale = 0.8
            approach_dir = 1.0

        frame = draw_scene(bg, object_scale, approach_scale)

        # HUD
        # Fixed-width formatting keeps text layout stable when values change.
        speed_line = f"speed: {speed:5.2f}"
        cv2.putText(frame, speed_line, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"size-scale: {object_scale:.2f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (30, 30, 30), 2, cv2.LINE_AA)
        cv2.putText(frame, f"approach-scale: {approach_scale:4.2f}", (20, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (30, 30, 30), 2, cv2.LINE_AA)
        cv2.putText(frame, "keys: +/- speed, [ ] size, q quit", (20, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (30, 30, 30), 2, cv2.LINE_AA)

        cv2.imshow("Synthetic Red/Black Test Scene", frame)

        key = cv2.waitKey(int(1000.0 / fps)) & 0xFF
        if key == ord('q'):
            break
        if key in (ord('+'), ord('=')):
            speed = min(6.0, speed + 0.1)
        if key in (ord('-'), ord('_')):
            speed = max(0.0, speed - 0.1)
        if key == ord(']'):
            object_scale = min(2.5, object_scale + 0.05)
        if key == ord('['):
            object_scale = max(0.4, object_scale - 0.05)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
