import numpy as np
import pyrealsense2 as rs


def start_aligned_pipeline(
    color_width,
    color_height,
    depth_width,
    depth_height,
    fps,
):
    """Start RealSense pipeline with color+depth+IMU and return (pipeline, align)."""
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    pipeline.start(config)

    align = rs.align(rs.stream.color)
    return pipeline, align


def read_accel_magnitude(frames, fallback=0.0):
    """Read accelerometer magnitude from a frameset. Returns fallback if not found."""
    accel_mag = fallback
    for f in frames:
        if f.is_motion_frame() and f.get_profile().stream_type() == rs.stream.accel:
            m = f.as_motion_frame().get_motion_data()
            accel_mag = float(np.linalg.norm([m.x, m.y, m.z]))
    return accel_mag
