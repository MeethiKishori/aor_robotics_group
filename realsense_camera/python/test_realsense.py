import sys
import pyrealsense2 as rs

# ANSI color codes
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
RESET  = "\033[0m"

def distance_color(dist):
    if dist <= 0:
        return RED, "INVALID"
    elif dist < 0.3:
        return RED, "DANGER"
    elif dist < 0.6:
        return YELLOW, "CAUTION"
    else:
        return GREEN, "SAFE"

try:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    print("Starting pipeline...")
    pipeline.start(config)

    print("Streaming depth — press Ctrl+C to stop.\n")
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        w = depth_frame.get_width()
        h = depth_frame.get_height()
        dist = depth_frame.get_distance(w // 2, h // 2)

        color, label = distance_color(dist)
        print(f"{color}[{label}] Centre distance: {dist:.3f} m{RESET}    \r", end="")

except KeyboardInterrupt:
    print("\nStopped.", flush=True)
except Exception as e:
    print(f"\nError: {e}", flush=True)
    sys.exit(1)
finally:
    try:
        pipeline.stop()
    except Exception:
        pass
