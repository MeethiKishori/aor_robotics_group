import os
import sys
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
PYTHON_SRC = os.path.join(PROJECT_ROOT, "python")
if PYTHON_SRC not in sys.path:
    sys.path.insert(0, PYTHON_SRC)

from camera.realsense_stream import start_aligned_pipeline, read_accel_magnitude


def main():

    try:
        pipeline, align = start_aligned_pipeline(640, 480, 640, 480, 30)
        pipeline.stop()

    except Exception as e :
        print(f"Error occurred: {e}")


if __name__ == '__main__':
    main()  