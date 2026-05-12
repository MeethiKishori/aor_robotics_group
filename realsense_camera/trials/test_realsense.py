import sys
import pyrealsense2 as rs

# ANSI escape codes for terminal colors.
# \033 starts an ANSI escape sequence, [..m sets style/color.
RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"  # Return terminal text to default color/style.


def distance_color(dist):
    """Map a measured distance (meters) to a color and safety label."""
    # No valid depth value received from camera.
    if dist <= 0:
        return RED, "INVALID"
    # Very close object.
    elif dist < 0.3:
        return RED, "DANGER"
    # Medium distance.
    elif dist < 0.6:
        return YELLOW, "CAUTION"
    # Safe distance.
    else:
        return GREEN, "SAFE"



try:
    # Build a RealSense pipeline (data stream manager).
    pipeline = rs.pipeline()
    # Create stream configuration object.
    config = rs.config()
    # Enable depth stream: 1280x720, 16-bit depth format, 30 FPS.
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    # Enable RGB (color) stream: 1280x720, BGR8 format, 30 FPS.
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    # Enable IR stream (left IR camera): 1280x720, 8-bit grayscale, 30 FPS.
    config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)

    print("Starting pipeline...")
    # Start camera streaming with the configuration above.
    pipeline.start(config)

    print("Streaming depth — press Ctrl+C to stop.\n")
    # Read frames continuously until interrupted.
    while True:
        # Block until a new frameset arrives.
        frames = pipeline.wait_for_frames()
        # Extract only the depth frame from the frameset.
        depth_frame = frames.get_depth_frame()
        # Extract the RGB (color) frame.
        color_frame = frames.get_color_frame()
        # Extract the IR (infrared) frame.
        ir_frame = frames.get_infrared_frame(1)  # 1 = left IR camera
        # Skip iteration if depth frame is missing.
        if not depth_frame:
            continue

        # RGB frame: shape (720, 1280, 3), values are Blue/Green/Red 0-255.
        if color_frame:
            color_image = color_frame.get_data()  # raw pixel data
            rgb_center = color_frame.get_data()   # access pixel: color_image[h//2][w//2] -> [B, G, R]

        # IR frame: shape (720, 1280), values are grayscale 0-255.
        if ir_frame:
            ir_image = ir_frame.get_data()        # access pixel: ir_image[h//2][w//2] -> brightness

        # Get depth image dimensions.
        w = depth_frame.get_width()
        h = depth_frame.get_height()
        # Read depth value at center pixel (returns meters).
        dist = depth_frame.get_distance(w // 2, h // 2)

        # Convert numeric distance to human-friendly status.
        color, label = distance_color(dist)
        # Print one updating status line (\r returns cursor to line start).
        print(f"{color}[{label}] Centre distance: {dist:.3f} m{RESET}    \r", end="")

# Handle Ctrl+C cleanly.
except KeyboardInterrupt:
    print("\nStopped.", flush=True)
# Handle all other runtime errors.
except Exception as e:
    print(f"\nError: {e}", flush=True)
    sys.exit(1)
finally:
    # Always try to stop the pipeline to release camera resources.
    try:
        pipeline.stop()
    except Exception:
        pass
