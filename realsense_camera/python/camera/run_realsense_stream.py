"""
Simple runner to start the RealSense aligned color+depth pipeline and display frames.
Run from project root:
    python3 python/camera/run_realsense_stream.py

Requires: pyrealsense2, numpy, opencv-python
"""

import sys
import time
import cv2
import numpy as np

from realsense_stream import start_aligned_pipeline, read_accel_magnitude
import pyrealsense2 as rs


def colorize_depth(depth_frame):
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    return depth_colormap


def main():
    try:
        pipeline, align = start_aligned_pipeline(640, 480, 640, 480, 30)
    except Exception as e:
        print(f"Failed to start RealSense pipeline: {e}")
        sys.exit(1)

    try:
        print("Streaming... press 'q' in the window or Ctrl-C to stop")
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = colorize_depth(depth_frame)

            accel_mag = read_accel_magnitude(frames, fallback=0.0)

            # Stack side-by-side for quick inspection
            combined = np.hstack((color_image, depth_colormap))
            cv2.putText(combined, f"Accel: {accel_mag:.2f} m/s^2", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

            cv2.imshow("RealSense Color | Depth", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
