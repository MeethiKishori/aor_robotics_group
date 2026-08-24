import sys
import numpy as np
import pyrealsense2 as rs
import open3d as o3d

# ── Choose mode: comment out one ──────────────────────────────────
#LIVE = True   # live stream: updates continuously, S=save, Q=quit
LIVE = False  # single frame: captures one frame, saves, shows static
# ─────────────────────────────────────────────────────────────────

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align   = rs.align(rs.stream.color)
pc_calc = rs.pointcloud()

def grab_cloud():
    frames     = pipeline.wait_for_frames()
    frames     = align.process(frames)
    depth      = frames.get_depth_frame()
    color      = frames.get_color_frame()
    pc_calc.map_to(color)
    pts        = pc_calc.calculate(depth)
    verts      = np.asanyarray(pts.get_vertices()).view(np.float32).reshape(-1, 3)
    texcoords  = np.asanyarray(pts.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
    img        = np.asanyarray(color.get_data())
    h, w, _    = img.shape
    u = np.clip((texcoords[:, 0] * w).astype(np.int32), 0, w - 1)
    v = np.clip((texcoords[:, 1] * h).astype(np.int32), 0, h - 1)
    valid      = np.isfinite(verts).all(axis=1) & (verts[:, 2] > 0)
    cloud      = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(verts[valid])
    cloud.colors = o3d.utility.Vector3dVector(img[v[valid], u[valid]][:, ::-1].astype(np.float64) / 255.0)
    return cloud

if LIVE:
    # ── Live mode ─────────────────────────────────────────────────
    print("Live mode  |  S = save pointcloud.ply  |  Q = quit")
    cloud = o3d.geometry.PointCloud()
    vis   = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="RealSense Live  |  S=save  |  Q=quit", width=1280, height=720)
    added = False

    def save(vis):
        o3d.io.write_point_cloud("pointcloud.ply", cloud)
        print(f"Saved {len(cloud.points)} points → pointcloud.ply")

    vis.register_key_callback(83, save)
    vis.register_key_callback(81, lambda v: v.close())

    try:
        while True:
            new = grab_cloud()
            cloud.points = new.points
            cloud.colors = new.colors
            if not added:
                vis.add_geometry(cloud)
                added = True
            else:
                vis.update_geometry(cloud)
            if not vis.poll_events():
                break
            vis.update_renderer()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        vis.destroy_window()

else:
    # ── Single frame mode ─────────────────────────────────────────
    print("Capturing single frame...")
    cloud = grab_cloud()
    pipeline.stop()
    o3d.io.write_point_cloud("pointcloud.ply", cloud)
    print(f"Saved {len(cloud.points)} points → pointcloud.ply")
    print("Close the window to exit.")
    o3d.visualization.draw_geometries([cloud], window_name="Point Cloud")
