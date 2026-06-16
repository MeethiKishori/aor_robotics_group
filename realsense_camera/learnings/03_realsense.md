# RealSense — Hardware & Python Setup

---

## How the RealSense D435i Works

### Hardware Components
- 2 IR cameras (grayscale sensors)
- 1 IR projector
- 1 RGB camera
- 1 IMU (inertial measurement unit, for motion tracking)

### Depth Sensing Process
1. The IR projector casts a structured light pattern onto the scene.
2. Both IR cameras capture the scene — the slight offset between them creates **disparity**.
3. **Stereo matching** computes the disparity between the two IR images.
4. **Triangulation** converts disparity into a depth value for each pixel.

### IR Camera Characteristics
- Captures grayscale images in a single wavelength band (~850 nm, near-infrared).
- Each pixel in the depth frame stores the distance from the camera:
  `depth[x, y]` → distance in metres (e.g. `depth[100, 200]` → `0.83 m`).

realsense camera also gives apart fro rgb , a Point Cloud 

**Note:** Even with the IR projector disabled, the IR cameras still function — they passively capture ambient infrared light from the environment.

---

## Python Wrapper Setup

The RealSense SDK is originally written in C++. To use it from Python, set up a virtual environment using the Python bindings from [librealsense](https://github.com/IntelRealSense/librealsense).

### First-time Setup

```bash
sudo apt install python3.12-venv
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera

# Create and activate virtual environment
python3 -m venv realsense_env
source realsense_env/bin/activate
```

### Every Session (activate the env)

The env lives inside the `realsense_camera` folder:

```bash
source realsense_env/bin/activate
echo $VIRTUAL_ENV   # confirm it's active
```

### Useful CLI Commands

- `realsense-viewer`       — open the GUI camera viewer
- `rs-enumerate-devices`  — list all connected RealSense devices
- `rs-save-to-disk`       — save frames to disk

Camera confirmed working: ![Camera output](image.png)
