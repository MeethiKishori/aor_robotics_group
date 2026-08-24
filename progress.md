# AOR Robotics Group - Learning Notes

**Adaptive Autonomy and Off-Road Robotics Group**

This document contains notes on learning the Finroc framework and Scout safety demo project.

---

## Table of Contents
1. [Scout Project Overview](#scout-project-overview)
2. [Project Structure](#project-structure)
3. [Safety Demo Pipeline](#safety-demo-pipeline)
4. [Understanding mRiskVisualization](#understanding-mriskvsisualization)
5. [Key Concepts](#key-concepts)

---

## Scout Project Overview

**Scout** is a mobile robot application built on the **Finroc** framework that demonstrates autonomous safety features. The main application is called **ScoutControl**.

### Purpose
Scout's primary demo is a **safety demonstration system** that:
- Detects nearby persons using AI (YOLOv4/v3 neural networks)
- Assesses risk based on distance and detection
- Visualizes the risk state on camera feeds
- Controls LED indicators and buzzer to alert humans

### Key Capabilities
- **Dual cameras** (front + rear stereo cameras)
- **Real-time AI perception** (person detection)
- **Risk assessment** based on proximity
- **Visual feedback** (annotated camera streams)
- **Audio-visual alerts** (LED + buzzer)

---

## Project Structure

```
finroc_projects_scout/
│
├── pScoutControl.cpp           # Main process entry point
├── gScoutControl.cpp/h         # Application control group (sensor/controller coordination)
├── gScoutControl.h
│
├── hardware/                   # Hardware interface layer
│   ├── gHardwareInterface.cpp/h
│   └── [Motor control, sensor interfaces]
│
├── safety_demo/                # Safety demonstration modules
│   ├── gSafetyDemo.cpp/h              # Orchestrator - connects all modules
│   ├── mPercept.cpp/h                 # AI Perception Module (YOLOv4/v3)
│   ├── mRiskAssessment.cpp/h          # Risk Evaluation Module
│   ├── mRiskToLampIndicator.cpp/h     # LED/Buzzer Control Module
│   ├── mRiskVisualization.cpp/h       # Visualization Overlay Module (OUR FOCUS)
│   │
│   └── third_party/darknet/           # Neural network models
│       ├── cfg/yolov4-tiny.cfg        # Front camera config
│       ├── yolov4-tiny.weights        # Front camera weights
│       ├── cfg/yolov3-tiny.cfg        # Rear camera config
│       └── yolov3-tiny.weights        # Rear camera weights
│
└── [Other components...]
```

### Naming Convention
- **g*** = Group/container (orchestrates multiple modules)
- **m*** = Module (single processing unit)
- **p*** = Process (executable entry point)

---

## Safety Demo Pipeline

### Data Flow Diagram

```
┌─────────────────────────────────────────────────────┐
│         FRONT & REAR STEREO CAMERAS                │
│  - Camera Images                                   │
│  - 3D Point Clouds (depth information)            │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
        ┌──────────────────────┐
        │  mPercept (x2)       │ AI Detection
        │  - YOLOv4/v3         │ - Bounding boxes
        │  - Person detection  │ - Class IDs
        │  - Distance calc     │ - Confidence scores
        └──────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
        ▼                     ▼
  ┌──────────────┐      ┌──────────────────────┐
  │mRiskAssess   │      │mRiskVisualization    │
  │- Risk level  │      │(OUR FOCUS)           │
  │- Risk label  │      │- Overlay risk info   │
  └──────┬───────┘      │  on image            │
         │              └──────┬───────────────┘
         │                     │
         ▼                     ▼
  ┌──────────────┐      ┌────────────────┐
  │mRiskToLamp   │      │GUI (finstruct) │
  │- LED control │      │Display live    │
  │- Buzzer mode │      │camera feeds    │
  └──────────────┘      └────────────────┘
```

### Processing Steps

**1. Perception (mPercept)**
- Receives camera images and point clouds
- Runs YOLOv4/v3 neural network
- Detects persons in frame
- Calculates distance to nearest person
- Outputs detection visualization with bounding boxes

**2. Risk Assessment (mRiskAssessment)**
- Takes detection results from mPercept
- Evaluates risk level based on:
  - Distance to nearest person
  - Number of persons detected
  - Detection confidence
- Outputs: risk_level (int), risk_label (string)

**3. Risk Visualization (mRiskVisualization) ⭐**
- Takes the detection visualization image from mPercept
- Takes risk_level and risk_label from mRiskAssessment
- Draws a black rectangle at bottom of image
- Overlays text: "Risk: [level] ([label])" in yellow-cyan color
- Outputs annotated image for display

**4. Risk to Lamp Indicator (mRiskToLampIndicator)**
- Takes risk level and label from mRiskAssessment
- Controls LED light mode and brightness
- Sets buzzer frequency/mode
- Provides audio-visual alerts to nearby humans

**5. Output**
- Visual streams sent to GUI (finstruct)
- LED/Buzzer signals sent to hardware

---

## Understanding mRiskVisualization

### Class Definition (Header)

```cpp
class mRiskVisualization : public structure::tModule {
  // INPUTS
  tInput<rrlib::coviroa::tImage> in_image;          // Detection image from mPercept
  tInput<int> in_risk_level;                        // Risk level (0, 1, 2, ...)
  tInput<std::string> in_risk_label;                // Risk label ("Safe", "Warning", "Danger")
  
  // PARAMETER
  tParameter<bool> par_enable_stream;               // Enable/disable visualization
  
  // OUTPUT
  tVisualizationOutput<rrlib::coviroa::tImage> out_image;  // Annotated image
};
```

### What It Does (Update Method)

```
INPUT: Detection image + Risk level + Risk label
  │
  ├─ Check if enabled (par_enable_stream)
  ├─ Check if data changed (optimization)
  ├─ Check if image is valid (width > 0, height > 0)
  │
  ├─ Convert image to OpenCV BGR24 format
  ├─ Copy input to output
  │
  ├─ Draw black rectangle at bottom
  │   Position: (8, rows-52) to (420, rows-8)
  │
  ├─ Overlay text in yellow-cyan
  │   Text: "Risk: {risk_level} ({risk_label})"
  │   Position: (16, rows-20)
  │   Font: SIMPLEX, size: 0.75, thickness: 2
  │
  └─ Publish annotated image with original timestamp

OUTPUT: Annotated image to out_image port
```

### Key Implementation Details

```cpp
void mRiskVisualization::Update() {
  // 1. Check if enabled and data changed
  if (!par_enable_stream.Get()) return;
  if (!(in_image.HasChanged() || in_risk_level.HasChanged() || in_risk_label.HasChanged()))
    return;
  
  // 2. Validate image
  auto image = in_image.GetPointer();
  if (image->GetWidth() == 0 || image->GetHeight() == 0) return;
  
  // 3. Preserve timestamp
  const auto ts = image.GetTimestamp();
  
  // 4. Convert to OpenCV format (BGR24)
  cv::Mat input = rrlib::coviroa::ConvertFormat<cv::Vec3b>(*image, ...);
  
  // 5. Prepare output buffer
  auto output = out_image.GetUnusedBuffer();
  output->Resize(image->GetWidth(), image->GetHeight(), ...);
  cv::Mat vis = rrlib::coviroa::AccessImageAsMat(*output);
  input.copyTo(vis);
  
  // 6. Get risk data
  int risk_level = 0;
  in_risk_level.Get(risk_level);
  std::string risk_label;
  in_risk_label.Get(risk_label);
  
  // 7. Create info string
  const std::string line = "Risk: " + std::to_string(risk_level) + 
                          " (" + risk_label + ")";
  
  // 8. Draw black background rectangle
  cv::rectangle(vis, cv::Point(8, vis.rows - 52), 
               cv::Point(std::min(vis.cols - 8, 420), vis.rows - 8), 
               cv::Scalar(0, 0, 0), cv::FILLED);
  
  // 9. Draw text (Yellow-Cyan in BGR)
  cv::putText(vis, line, cv::Point(16, vis.rows - 20), 
             cv::FONT_HERSHEY_SIMPLEX, 0.75, 
             cv::Scalar(0, 255, 255), 2);
  
  // 10. Publish with original timestamp
  output.SetTimestamp(ts);
  out_image.Publish(output);
}
```

### Why This Design?

1. **Modular**: Visualization is separate from detection and risk assessment
2. **Efficient**: Only updates when data changes
3. **Timestamp preservation**: Maintains timing for real-time systems
4. **Dual streams**: Front and rear cameras each have their own instance
5. **Non-blocking**: Can enable/disable visualization without stopping perception

---

## How gSafetyDemo Orchestrates Everything

The `gSafetyDemo` constructor creates and connects all modules:

```cpp
// Front Side Setup
auto front_percept = new mPercept(this, "Front Percept");
front_percept->par_target_class_name.Set("person");
// ... configuration ...
front_percept->in_images.ConnectTo(this->si_front_stereo_camera_images);

auto front_risk_assessment = new mRiskAssessment(this, "Front Risk Assessment");
front_risk_assessment->in_nearest_distance_m.ConnectTo(
  front_percept->out_nearest_distance_m);
// ... more connections ...

auto front_risk_visualization = new mRiskVisualization(this, "Front Risk Visualization");
front_risk_visualization->in_image.ConnectTo(
  front_percept->out_visualization_classes);  // Input: detection image
front_risk_visualization->in_risk_level.ConnectTo(
  front_risk_assessment->out_risk_level);     // Input: risk level
front_risk_visualization->in_risk_label.ConnectTo(
  front_risk_assessment->out_risk_label);     // Input: risk label
this->vo_front_perception_risk.ConnectTo(
  front_risk_visualization->out_image);       // Output: annotated image

// Rear Side Setup (same as front)
auto rear_risk_visualization = new mRiskVisualization(this, "Rear Risk Visualization");
// ... similar connections ...
```

Key Points:
- **Two instances**: Front and Rear visualizations
- **Port connections**: Data flows through connected ports (like pipes)
- **Automatic scheduling**: Finroc calls Update() when data arrives
- **Decoupled**: Each module doesn't know about others, only port connections

---

## Key Concepts

### Finroc Framework Basics

**What is Finroc?**
- Middleware framework for real-time robot control
- Based on publish-subscribe model (port connections)
- Modular architecture (modules = reusable components)
- Automatic scheduling (no manual event handling)

**Key Components:**
- **tModule**: Base class for processing modules
- **tInput<T>**: Input port (receives data)
- **tParameter<T>**: Configuration parameter
- **tOutput<T>**: Output port (sends data)
- **tFrameworkElement**: Base for hierarchical structure

### OpenCV Usage in mRiskVisualization

- `cv::Mat`: Image matrix (in-memory 2D array)
- `cv::rectangle()`: Draw filled/unfilled rectangles
- `cv::putText()`: Render text on image
- `cv::Scalar(B, G, R)`: Color in BGR format (not RGB!)

### Image Format Conversion

- **rrlib::coviroa::tImage**: Generic Finroc image type
- **cv::Mat**: OpenCV's native image type
- Conversion ensures compatibility between components

### Real-time Considerations

- **Timestamp preservation**: Critical for sensor fusion
- **Conditional updates**: Only process when data changes (efficiency)
- **Buffer management**: Reuse buffers to reduce allocations
- **No blocking**: All operations must be non-blocking

---

## Building & Running Scout

```bash
# Navigate to Finroc root
cd ~/Finroc/finroc

# Load Scout environment
source scripts/setenv -p scout

# Build ScoutControl
make ScoutControl-bin

# Run ScoutControl
./export/linux_x86_64_debug/bin/ScoutControl

# In another terminal, build and run finstruct (GUI)
source scripts/setenv -l java
make finstruct
finstruct
```

---

## Learning Path

1. **Understand modules**: Read mPercept, mRiskAssessment, mRiskVisualization
2. **Understand groups**: Read gSafetyDemo (orchestration)
3. **Finroc basics**: Learn port connections and data flow
4. **OpenCV basics**: Image manipulation, drawing functions
5. **Extend modules**: Add features to existing modules
6. **Create new modules**: Build custom processing components

---

## References

- **Finroc Documentation**: `/home/ssingh/Finroc/finroc/README.md`
- **Scout README**: `/home/ssingh/Finroc/finroc_projects_scout/README.md`
- **OpenCV Documentation**: https://docs.opencv.org
- **mRiskVisualization Source**: `finroc_projects_scout/safety_demo/mRiskVisualization.cpp`

---

## Investigation Report: ScoutControl Runtime Flow

Trace of a real `ScoutControl` startup log against the source, to understand what each stage does and whether failures are fatal.

### A) Overall Architecture of ScoutControl

**Entry point:**
- `pScoutControl.cpp` — Program wrapper that instantiates the main executable
  - Loads `gScoutControl` (line 88) as the root module
  - Main thread runs at 30 Hz (line 86: `SetCycleTime(std::chrono::milliseconds(33))`)
  - Uses Finroc's default_main_wrapper plugin for threading and initialization

**Main module:**
- `gScoutControl.cpp` — Top-level control group
  - Contains two sub-groups: `gHardwareInterface` and `gSafetyDemo` (lines 81–83)

**Hardware vs. Safety pipeline:**
- `gHardwareInterface` — manages robot sensors/actuators (cameras, lidar, LED indicators)
- `gSafetyDemo` — perception + risk assessment + indicator control (connects to hardware outputs)

### B) gHardwareInterface: Indicator Mapping and RealSense Fallback

**File:** `hardware/gHardwareInterface.cpp:260–377`

**Indicator mapping (lines 276–314):**
- Hardcoded paths for CH341 USB signal lights:
  - **Front:** `/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0` (line 77) → fallback `/dev/ttyUSB0` (lines 80–101)
  - **Rear:** `/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0` (line 78) → fallback `/dev/ttyUSB1` (lines 103–116)
- Device resolution logic (lines 276–289): attempts to resolve different serial paths; if rear device matches front, tries alternate USB device (lines 280–284)
- Both indicator lights are instantiated as modules (lines 302–313) with their serial device paths

**RealSense mapping (lines 322–369):**
- Queries connected RealSense cameras by physical port hint (lines 201–228)
  - **Front camera:** queries port containing `"-1/"` (line 326)
  - **Rear camera:** queries port containing `"-4/"` (line 327)
- If rear serial is empty (line 333), falls back to second available camera; if still empty (line 356–357):
  - **Line 366 WARNING:** "Rear RealSense not uniquely available. Falling back to front stream for rear ports."
  - Both rear ports (`so_rear_stereo_camera_images` and `so_rear_stereo_point_cloud`) connect to front camera output (lines 367–368)
- **Why empty rear_sn?** Either no second RealSense physically present, or the rear camera isn't at physical port `-4/` (misconfigured USB hub or not plugged in)

### C) CH341 USB Signal Light Driver & Serial Device Error Handling

**CH341 Driver:**
- `sources/cpp/rrlib/hid/tCH341UsbSignalLightDriver.h:129–141` (in the `finroc` core repo)
- Constructor attempts to open serial device at 9600 baud
- If `isOpen()` succeeds, logs "Successfully started..." (line 134)
- If fails, logs "Failed to start CH341 serial device interface" (line 139)

**Serial Device Opening:**
- `sources/cpp/rrlib/serial_port/tSerialDevice.cpp:118–153`
- **Non-Boost implementation** (lines 137–145): uses `open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)`
  - On line 142, if `device_handle <= 0`, logs: `"Could not open serial device '", device_name, "'"`
  - Matches the observed log: "No such file or directory" (errno 2) — device path doesn't exist at that moment

**Error is NOT fatal:**
- `gHardwareInterface::ForceSetIndicator()` (lines 153–175) wraps CH341 driver construction in try-catch blocks
- Catches exceptions and logs a WARNING rather than aborting
- Hardware interface construction continues even if indicator lights fail to initialize
- LED modules (`mCH341UsbSignalLight`) are created anyway; they simply won't transmit commands if the serial device never opened
- Program continues to initialize cameras, lidar, and other components

### D) Robosense RS-Lidar-16 & Circular Buffer Sizing

**File:** `sources/cpp/libraries/laser_scanner/mRobosense.cpp:256–261`

**Auto-determination logic:**
```cpp
size_t cycles = std::chrono::duration_cast<std::chrono::milliseconds>(
  scheduling::tThreadContainerThread::CurrentThread()->GetCycleTime()).count();
this->rs_driver_->SetBufferCapacity(cycles * 50);
```

- Retrieves the **thread cycle time in milliseconds** (main thread: 33 ms → 33 * 50 = **1650 packets**)
- Buffer capacity is sized to hold ~50 scan cycles worth of data
- At the main thread's 30 Hz (33 ms), the circular buffer holds ~1.65 seconds of RS-Lidar-16 frames
- This prevents drops if the consumer thread lags behind the network reception thread

### E) RealSense D435 Camera: Device Query, Selection, and Enable

**File:** `sources/cpp/libraries/camera/intel_realsense/mD400.cpp:76–281`

**Flow:**
1. **Construction** (line 76): receives shared `rs2::context` for device management
2. **Parameter change on serial number** (lines 221–235):
   - Calls `QueryDevices(*par_sn.GetPointer())` to verify device is present before enabling
   - If found, calls `config.enable_device(serial_number)` (line 227)
   - Sets `valid_device = true` (line 228)
3. **Pipeline start** (lines 237–281):
   - Stops old pipeline if running (lines 239–244)
   - Calls `pipeline.start(config)` (line 248)
   - Catches 10+ specific exception types (`rs2::camera_disconnected_error`, `rs2::backend_error`, etc.)
   - Each logs an ERROR with device serial and reason
4. **Update loop** (line 302): `pipeline.poll_for_frames()` retrieves color, depth, and point cloud frames per output port connections

**Why two cameras in gSafetyDemo?**
- Line 347–355: Front camera created with configurable serial (line 351)
- Line 356–363: Rear camera created only if `rear_sn` is non-empty and different from front
- If that condition fails (line 333), rear ports reuse the front output (lines 367–368)

### F) Darknet/YOLO Integration: Two Networks for Front & Rear

**Architecture:**
- `safety_demo/gSafetyDemo.cpp:69–156`
- Creates **two separate `mPercept` instances** — one per camera stream

**Front Percept** (lines 73–83):
- **Config:** `yolov4-tiny.cfg` + `yolov4-tiny.weights` (lines 75–76)
- **Purpose:** detects persons in front stereo images
- **Input:** `si_front_stereo_camera_images` (from `gHardwareInterface`)

**Rear Percept** (lines 115–125):
- **Config:** `yolov3-tiny.cfg` + `yolov3-tiny.weights` (lines 117–118)
- **Purpose:** detects persons in rear stereo images (or front if rear unavailable)
- **Input:** `si_rear_stereo_camera_images` (falls back to front per section B above)

**Why different YOLO versions?** Possibly intentional experimentation or field iteration (yolov3-tiny is older/lighter than yolov4-tiny). Both trained on COCO dataset for person detection.

**Module loading** (`safety_demo/mPercept.cpp:248–272`):
- Constructor sets default config path (lines 217–218)
- `Update()` method dynamically reloads the model if a parameter changes (line 253)
- On first call or parameter change, instantiates a `tDarknetYOLO` object (line 260)
- Logs: "YOLO model loaded: [config_path]" (line 263)
- Both modules initialize independently in parallel; any load failure only affects that percept branch

### G) TCP Peer Connectivity: Remote Runtime Discovery & Benign Churn

**TCP Server:**
- `sources/cpp/plugins/tcp/internal/tServer.cpp:200`
- Listens on the configured port (here 4444): "TCP server is listening on port 4444"
- Finroc framework automatically starts a TCP server for peer-to-peer distributed runtime discovery

**Remote Runtime & Peer Connection:**
- `sources/cpp/plugins/network_transport/generic_protocol/tRemoteRuntime.cpp:171` logs "Connected to [peer_name]" on successful peer connection
- `sources/cpp/plugins/tcp/internal/tPeerImplementation.cpp:94` logs "Disconnected from [peer_name]" when a peer drops

**What is "hiwi-z890eaglewifi7-1<18630>"?**
- Hostname of another Finroc instance on the network
- The port number in angle brackets identifies the connection
- **Expected behavior:** auto-discovery (if `par_auto_connect_to_all_peers` is enabled) attempts to connect to advertised peers
- **Rapid connect/disconnect** indicates that peer had brief availability (e.g. a monitoring tool or finstruct instance attaching momentarily) or a handshake mismatch, followed by a periodic retry
- **Not fatal** — ScoutControl continues running independently regardless

### Summary Table

| Component | File:Line | Purpose | Failure Mode |
|-----------|-----------|---------|--------------|
| Entry point | pScoutControl.cpp:88 | Loads gScoutControl in 30 Hz main loop | N/A |
| Indicator mapping | gHardwareInterface.cpp:276–314 | Resolves front/rear CH341 devices | Logs WARNING; continues without indicators |
| RealSense fallback | gHardwareInterface.cpp:356–369 | Falls back to front camera if rear unavailable | Logs WARNING; rear streams duplicate front |
| CH341 driver | tCH341UsbSignalLightDriver.h:129–141 | Opens serial device at 9600 baud | Wrapped in try-catch; logs ERROR; non-fatal |
| Serial open failure | tSerialDevice.cpp:142 | Attempts `open(device_path, O_RDWR...)` | Returns false; module continues |
| RS-Lidar buffer | mRobosense.cpp:256–261 | Auto-sizes circular buffer to 50 × cycle_time | Non-fatal; defaults to fixed size |
| RealSense D400 | mD400.cpp:221–281 | Queries by serial, enables device, starts pipeline | Catches exceptions; logs ERROR; non-fatal if device missing |
| YOLO loading | mPercept.cpp:248–272 | Dynamically loads yolov4-tiny & yolov3-tiny | Wrapped in try-catch; logs ERROR; publishes raw images if failed |
| TCP peer discovery | tRemoteRuntime.cpp:171 | Logs peer connections | Non-fatal; retries periodically |

### Overall design takeaway
ScoutControl follows a **graceful degradation** pattern: missing or failed hardware components log errors but don't halt the program. The robot operates with whatever sensors/actuators are available (here: front-camera-only sensing, duplicated to the rear ports, no LED indicators).

---

## Porting the Python YOLO Hazard Pipeline into Scout (C++)

**Scope decision (2026-07-08):** only the YOLO hazard pipeline
(`realsense_camera/python/launch/run_yolo_hazard.py` and the perception/risk modules it uses)
gets ported into the safety demo. The red/black ROI pipeline (`run_risk_roi_red_black.py`,
`red_black_roi.py`, TTC/slider logic) stays a standalone Python prototype and is **NOT to be ported**.

### Hard constraint for all future work on this port

**Keep the original safety_demo structure intact — no structural changes.**
- No new modules and no changes to the `gSafetyDemo` wiring.
- No new ports or parameters on existing modules unless truly unavoidable; reuse the existing ones.
- All implementation goes inside the existing module internals:
  `mPercept` (detection + depth), `mRiskAssessment` (risk model), `mRiskVisualization` (overlay).

### What the Python YOLO hazard pipeline does

1. **Detection** (`perception/yolo_detector.py`): YOLO per frame → labeled detections (HUMAN / ROBOT) with confidence.
2. **Depth attach** (`perception/depth_estimation.py::median_depth_in_box()`): per detection, median depth (meters) over a small patch (1/5 of bbox, min 3 px) at the bbox center — noise-robust; 0.0 means "no valid depth".
3. **Hazard representation** (`perception/hazard.py`): per-frame list of (category c_i, distance d_i, box, confidence).
4. **Risk model** (`risk/runtime_risk_model.py` + `category_weights.py` + `distance_risk.py`):
   `R_i = w(c_i) * g(d_i) * (1 + alpha * v_tilde)` with weights w: LIVING 3.0 / DYNAMIC 2.0 / STATIC 1.0,
   `g(d) = 1/d` clamped to RMAX=30 for d ≤ 0.1 m, alpha = 0.5. Robot velocity `v_tilde` is 0 for now (not tracked yet).
   Environmental risk `R_env = max_i R_i`.
5. **Visualization**: boxes + per-object distance and R_i, header with R_env and nearest object.

### Mapping to safety_demo (all inside existing modules)

| Python piece | C++ home | Status |
|---|---|---|
| `yolo_detector.py` | `mPercept` — darknet YOLO (yolov4/v3-tiny, person detection) already there | already existed |
| `depth_estimation.py` median depth | `mPercept::Update()` internal — per-detection median point-cloud depth replaces the bbox-height distance heuristic (heuristic kept only as fallback when depth is unavailable) | **DONE 2026-07-08** |
| `hazard.py` | implicit — existing ports `out_nearest_distance_m`, `out_target_class_distance_m`, `out_detections` already carry category + distance | no work needed |
| `category_weights.py` + `distance_risk.py` + `runtime_risk_model.py` | `mRiskAssessment::Update()` internal — computes `R = w(c) * g(d)` and maps it onto the existing `out_risk_level`/`out_risk_label` outputs; the existing `par_*_threshold_scale` parameters scale the class weights | **DONE 2026-07-08** |
| velocity term `(1 + alpha * v)` | deferred — robot velocity not tracked in the Python prototype either (runs with v=0) | deferred |
| visualization | `mRiskVisualization` already overlays risk level + label | verified, no change |
| LED/buzzer actuation | `mRiskToLampIndicator` already maps risk level 0-4 to light mode / flash / buzzer, incl. camera-timeout failsafe | verified, no change |
| `gSafetyDemo` wiring | unchanged, as required | verified, no change |

### Implementation notes — perception step (done 2026-07-08)

- `mPercept` previously ignored `in_point_cloud` and approximated distance as
  `image_rows / bbox_height_px`. Now each detection gets the median Euclidean distance over a
  center patch of the organized RealSense cloud, converted to meters via `cloud.Unit()`
  (`mD400` publishes meters already; the Z16 depth-scale gotcha only applies to the raw depth
  image port, which we don't use).
- The cloud arrives as a flat point vector; its grid size is not transported. It is inferred:
  point count == color image size (depth-to-color alignment case) or one of the known RealSense
  depth resolutions. If neither matches, the old height heuristic is used, so behavior degrades
  gracefully instead of publishing garbage distances.
- Zero port/parameter changes. `out_visualization_classes` labels now include the measured
  distance per detection.

### Implementation notes — risk step (done 2026-07-08)

- `mRiskAssessment::Update()` now computes the ported risk model internally:
  `w(c)`: person/animal → LIVING 3.0, vehicle → DYNAMIC 2.0, everything else → STATIC 1.0,
  each multiplied by the existing `par_*_threshold_scale` parameter for its class (defaults 1.0).
  `g(d) = 1/d`, clamped to 30 for d ≤ 0.1 m; `d ≤ 0` (no object / no valid depth) → 0 risk.
- `R_env = max(R_nearest_object, R_nearest_person)` — the percept stage publishes both the
  nearest object of any class and the nearest person (`in_target_class_distance_m`), so a
  farther person can outrank a nearer low-weight object, approximating the Python `max_i R_i`.
- The existing `par_distance_*_m` parameters keep their meaning "distance at which a **person**
  reaches this level": internally converted to risk thresholds via the LIVING weight
  (`T = 3.0 / distance`). Person behavior is band-for-band identical to before; lower-weighted
  classes must come proportionally closer to trigger the same level.
- Two pre-existing issues fixed along the way: the class scale parameters were declared but the
  code always applied the person scale regardless of class; and an empty scene
  (nearest distance published as 0.0 = "no detection") was classified as EXTREME — it now
  yields risk level 0 / NONE.
- No new ports or parameters anywhere; `gSafetyDemo`, `mRiskVisualization`,
  `mRiskToLampIndicator` untouched. Full `make ScoutControl-bin` builds cleanly.

(Note: between this step and the next, `mPercept`/`mRiskAssessment` picked up further
independent work not tracked here in detail — a second robot Darknet model, an optional
Python-TCP detector bridge, a rise-instantly/fall-slowly + CONTACT-latch temporal state
machine in `mRiskAssessment`, and single-object approach-velocity/TTC telemetry. That work
is the baseline the multi-object step below builds on.)

### Implementation notes — multi-object tracking + time-to-collision (done 2026-08-20)

**Problem this solves:** risk was computed from a single per-frame "nearest object" scalar.
A far-but-fast-approaching object could never outrank a near-but-static one, because only
whichever object happened to be geometrically closest was ever looked at — and if a
different object became "nearest" the next frame, its velocity history reset (comparing two
different objects' distances gives a meaningless velocity).

**Fix — per-object tracking:**
- New port pair `mPercept::out_ranged_detections` (`vector<rrlib::machine_learning_appliance::
  tMLStringDetection2D<float>>` — bbox + free-form label + probability) and
  `out_ranged_distances_m` (`vector<float>`, index-aligned), wired to matching new inputs on
  `mRiskAssessment` in `gSafetyDemo.cpp` (front + rear). Reuses rrlib's existing
  string-labelled detection type instead of a new custom struct, so no hand-written
  serialization was needed. `mPercept`'s old `out_detections`/`in_detections` pair is left
  as-is (still dead/unused, per its existing comment — the COCO-enum type can't hold a
  free-form "robot" label).
- `mRiskAssessment::Update()` now matches each frame's detections to a `tTrack` (label +
  last bbox centre) map by nearest-centre-within-radius (`par_track_match_radius_px`,
  default 120 px), same label required. Unmatched tracks age out after
  `par_track_max_misses` (default 5) consecutive missed frames — survives brief occlusion,
  prunes objects that actually left.
- Each track keeps its own EMA-smoothed velocity (alpha 0.2, 0.05 m/s dead-band — same
  constants as the Python prototype's `runtime.py`), computed only between two real
  (>0) depth readings on the *same* track.

**Fix — TTC can matter even when raw distance risk is ~0:** per track,
`R_i = w(c) * g(d) * (1 + alpha_v * v_tilde)` (alpha_v=0.5, v_tilde = velocity / 2 m/s,
clamped [-1, 2]) — same shape as the Python prototype. Separately, if a track's time-to-
collision (`distance / velocity`, only for genuinely closing tracks) drops below
`par_ttc_extreme_s` (2.0s) or `par_ttc_high_s` (4.0s), `R_i` is floored to at least the
EXTREME/HIGH threshold value, regardless of the `g(d)` term. This matters because `g(d)`
hard-zeroes past `cRISK_D_MAX` (3.0 m): without the floor, an object closing fast from 5 m
away would contribute zero risk right up until it crossed 3 m. `R_env = max_i R_i` over
tracks seen in the current frame (true per-object max, replacing the earlier
`max(nearest_object, nearest_person)` approximation). The existing 4-level mapping,
rise-instantly/fall-slowly/CONTACT-latch state machine, and all downstream ports/modules
are unchanged — `out_distance_estimate_m`/`out_nearest_class`/`out_approach_velocity_mps`/
`out_time_to_collision_s` now report whichever *track* produced `R_env`, not whichever
object was geometrically nearest.
- One wiring change in `gSafetyDemo.cpp` (the two new port connections, front + rear) —
  unavoidable to move a per-object list across the module boundary; no new modules, no
  other wiring changes. Full `make ScoutControl-bin` builds cleanly.



delte below

(146) Cannot build rrlib_osm_test_graph_tests (sources/cpp/rrlib/osm/tests/make.xml:4) due to dependency rrlib_osm_graph (38) (sources/cpp/rrlib/osm/make.xml:15) which cannot be built
(147) Cannot build optional rrlib_path_control_example_trajectory (sources/cpp/rrlib/path_control/examples/make.xml:3) due to dependency rrlib_geometry_extended_shapes (22) (sources/cpp/rrlib/geometry/make.xml:34) which cannot be built
(148) Cannot build rrlib_probabilistic_methods_test_probabilistic_methods_tests (sources/cpp/rrlib/probabilistic_methods/tests/make.xml:4) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(149) Cannot build rrlib_rtti_test (sources/cpp/rrlib/rtti/make.xml:21) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(150) Cannot build rrlib_serialization_utils (sources/cpp/rrlib/serialization/make.xml:11) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(151) Cannot build rrlib_serialization_test_file_sink_source (sources/cpp/rrlib/serialization/tests/make.xml:5) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(152) Cannot build rrlib_serialization_test (sources/cpp/rrlib/serialization/tests/make.xml:11) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(153) Cannot build rrlib_si_units_test (sources/cpp/rrlib/si_units/tests/make.xml:5) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(154) Cannot build rrlib_signal_filters_test_boolean_hysterese (sources/cpp/rrlib/signal_filters/tests/make.xml:4) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(155) Cannot build rrlib_signal_filters_test_exponential_filter (sources/cpp/rrlib/signal_filters/tests/make.xml:5) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(156) Cannot build rrlib_signal_filters_test_moving_average_smoothing (sources/cpp/rrlib/signal_filters/tests/make.xml:6) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(157) Cannot build rrlib_signal_processing_test_integration (sources/cpp/rrlib/signal_processing/tests/make.xml:5) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(158) Cannot build rrlib_signal_processing_test_derivation (sources/cpp/rrlib/signal_processing/tests/make.xml:6) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(159) Cannot build optional rrlib_slam_example_incremental_isam2_slam (sources/cpp/rrlib/slam/examples/make.xml:4) due to dependency rrlib_slam_slam (81) (sources/cpp/rrlib/slam/make.xml:3) which cannot be built
(160) Cannot build optional rrlib_slam_data_association (sources/cpp/rrlib/slam/make.xml:19) due to dependency rrlib_scene (43) (sources/cpp/rrlib/scene/make.xml:4) which cannot be built
(161) Cannot build optional rrlib_slam_example_triangle_features (sources/cpp/rrlib/slam/examples/make.xml:5) due to dependency rrlib_slam_data_association (160) (sources/cpp/rrlib/slam/make.xml:19) which cannot be built
(162) Cannot build optional rrlib_slam_data_association_eigen (sources/cpp/rrlib/slam/make.xml:25) due to dependency rrlib_slam_data_association (160) (sources/cpp/rrlib/slam/make.xml:19) which cannot be built
(163) Cannot build optional rrlib_slam_data_association_pcl (sources/cpp/rrlib/slam/make.xml:30) due to dependency rrlib_slam_data_association (160) (sources/cpp/rrlib/slam/make.xml:19) which cannot be built
(164) Cannot build rrlib_tentacles_test_calculations (sources/cpp/rrlib/tentacles/tests/make.xml:4) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(165) Cannot build rrlib_time_test_time (sources/cpp/rrlib/time/make.xml:15) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(166) Cannot build rrlib_time_test_remote_timestamp_conversion (sources/cpp/rrlib/time/make.xml:20) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(167) Cannot build rrlib_uri_test_collection (sources/cpp/rrlib/uri/make.xml:11) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(168) Cannot build rrlib_util_test_type_list (sources/cpp/rrlib/util/tests/make.xml:5) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(169) Cannot build rrlib_util_test_tagged_pointer (sources/cpp/rrlib/util/tests/make.xml:6) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(170) Cannot build rrlib_util_test_string (sources/cpp/rrlib/util/tests/make.xml:7) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(171) Cannot build rrlib_util_test_fileio (sources/cpp/rrlib/util/tests/make.xml:8) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(172) Cannot build rrlib_util_test_enumerate (sources/cpp/rrlib/util/tests/make.xml:9) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(173) Cannot build rrlib_vehicle_kinematics_test_kinematics (sources/cpp/rrlib/vehicle_kinematics/tests/make.xml:5) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
(174) Cannot build rrlib_xml_test (sources/cpp/rrlib/xml/tests/make.xml:4) due to dependency finroc_plugins_unit_test (17) (sources/cpp/plugins/unit_test/make.xml:4) which cannot be built
Creating Makefile successful.
version of package pcl_surface: 1.14.0
version of package realsense2: 2.57.7
version of package pcl_common: 1.14.0
version of package jsoncpp: 1.9.5
version of package pcl_filters: 1.14.0
version of package pcl_io: 1.14.0
version of package flann: 1.9.2
version of package pcl_segmentation: 1.14.0
version of package pcl_registration: 1.14.0
version of package opencv4: 4.6.0
version of package Qt5Gui: 5.15.13
version of package darknet: local
version of package libxml-2.0: 2.9.14
version of package pcl_kdtree: 1.14.0
version of package libcurl: 8.5.0
version of package pcl_features: 1.14.0
version of package eigen3: 3.4.0
done
Thread 1 'Garbage Deleter'::Launcher [sources/cpp/rrlib/thread/tThread.cpp:355] >> Thread started
Indicator mapping: front=/dev/ttyUSB0, rear=/dev/ttyUSB0
ScoutControl::setBaudRate [sources/cpp/rrlib/serial_port/tSerialDevice.cpp:367] >> Updated serial device baud rate to 9600
ScoutControl::tCH341UsbSignalLightDriver [sources/cpp/rrlib/hid/tCH341UsbSignalLightDriver.h:134] >> Successfully started CH341 serial device interface: /dev/ttyUSB0
ScoutControl::setBaudRate [sources/cpp/rrlib/serial_port/tSerialDevice.cpp:367] >> Updated serial device baud rate to 9600
ScoutControl::tCH341UsbSignalLightDriver [sources/cpp/rrlib/hid/tCH341UsbSignalLightDriver.h:134] >> Successfully started CH341 serial device interface: /dev/ttyUSB0
ScoutControl::setBaudRate [sources/cpp/rrlib/serial_port/tSerialDevice.cpp:367] >> Updated serial device baud rate to 9600
ScoutControl::tCH341UsbSignalLightDriver [sources/cpp/rrlib/hid/tCH341UsbSignalLightDriver.h:134] >> Successfully started CH341 serial device interface: /dev/ttyUSB0
RealSense mapping: front_sn=909512070528, rear_sn=
Main Thread/Scout Control/Hardware Interface::gHardwareInterface [sources/cpp/projects/scout/hardware/gHardwareInterface.cpp:366] >> WARNING: Rear RealSense not uniquely available. Falling back to front stream for rear ports.
Thread 3 'TCP Thread'::Launcher [sources/cpp/rrlib/thread/tThread.cpp:355] >> Thread started
Thread 4 'Watchdog'::Launcher [sources/cpp/rrlib/thread/tThread.cpp:355] >> Thread started
TCP server is listening on port 4444
Finroc program 'ScoutControl' is now running.
Thread 5 'ThreadContainer Main Thread'::Launcher [sources/cpp/rrlib/thread/tThread.cpp:355] >> Thread started
Main Thread/Scout Control/Hardware Interface/RS-Lidar-16::OnParameterChange [sources/cpp/libraries/laser_scanner/mRobosense.cpp:260] >> Auto determination of buffer size. Current thread cycle time is 33ms. Setting circular buffer capacity to 1650 packages.
Main Thread/Scout Control/Hardware Interface/RealSense D435 Front::QueryDevices [sources/cpp/libraries/camera/intel_realsense/mD400.cpp:486] >> Querying devices... 
Found device: 
SN: 909512070528
Name: Intel RealSense D435I
Main Thread/Scout Control/Hardware Interface/RealSense D435 Front::OnParameterChange [sources/cpp/libraries/camera/intel_realsense/mD400.cpp:226] >> Enabling device with S/N 909512070528
Main Thread/Scout Control/Hardware Interface/RealSense D435 Front::OnParameterChange [sources/cpp/libraries/camera/intel_realsense/mD400.cpp:250] >> Starting device ...
mini_batch = 1, batch = 1, time_steps = 1, train = 0 
   layer   filters  size/strd(dil)      input                output
   0 conv     32       3 x 3/ 2    416 x 416 x   3 ->  208 x 208 x  32 0.075 BF
   1 conv     64       3 x 3/ 2    208 x 208 x  32 ->  104 x 104 x  64 0.399 BF
   2 conv     64       3 x 3/ 1    104 x 104 x  64 ->  104 x 104 x  64 0.797 BF
   3 route  2                                  1/2 ->  104 x 104 x  32 
   4 conv     32       3 x 3/ 1    104 x 104 x  32 ->  104 x 104 x  32 0.199 BF
   5 conv     32       3 x 3/ 1    104 x 104 x  32 ->  104 x 104 x  32 0.199 BF
   6 route  5 4                                    ->  104 x 104 x  64 
   7 conv     64       1 x 1/ 1    104 x 104 x  64 ->  104 x 104 x  64 0.089 BF
   8 route  2 7                                    ->  104 x 104 x 128 
   9 max                2x 2/ 2    104 x 104 x 128 ->   52 x  52 x 128 0.001 BF
  10 conv    128       3 x 3/ 1     52 x  52 x 128 ->   52 x  52 x 128 0.797 BF
  11 route  10                                 1/2 ->   52 x  52 x  64 
  12 conv     64       3 x 3/ 1     52 x  52 x  64 ->   52 x  52 x  64 0.199 BF
  13 conv     64       3 x 3/ 1     52 x  52 x  64 ->   52 x  52 x  64 0.199 BF
  14 route  13 12                                  ->   52 x  52 x 128 
  15 conv    128       1 x 1/ 1     52 x  52 x 128 ->   52 x  52 x 128 0.089 BF
  16 route  10 15                                  ->   52 x  52 x 256 
  17 max                2x 2/ 2     52 x  52 x 256 ->   26 x  26 x 256 0.001 BF
  18 conv    256       3 x 3/ 1     26 x  26 x 256 ->   26 x  26 x 256 0.797 BF
  19 route  18                                 1/2 ->   26 x  26 x 128 
  20 conv    128       3 x 3/ 1     26 x  26 x 128 ->   26 x  26 x 128 0.199 BF
  21 conv    128       3 x 3/ 1     26 x  26 x 128 ->   26 x  26 x 128 0.199 BF
  22 route  21 20                                  ->   26 x  26 x 256 
  23 conv    256       1 x 1/ 1     26 x  26 x 256 ->   26 x  26 x 256 0.089 BF
  24 route  18 23                                  ->   26 x  26 x 512 
  25 max                2x 2/ 2     26 x  26 x 512 ->   13 x  13 x 512 0.000 BF
  26 conv    512       3 x 3/ 1     13 x  13 x 512 ->   13 x  13 x 512 0.797 BF
  27 conv    256       1 x 1/ 1     13 x  13 x 512 ->   13 x  13 x 256 0.044 BF
  28 conv    512       3 x 3/ 1     13 x  13 x 256 ->   13 x  13 x 512 0.399 BF
  29 conv    255       1 x 1/ 1     13 x  13 x 512 ->   13 x  13 x 255 0.044 BF
  30 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
  31 route  27                                     ->   13 x  13 x 256 
  32 conv    128       1 x 1/ 1     13 x  13 x 256 ->   13 x  13 x 128 0.011 BF
  33 upsample                 2x    13 x  13 x 128 ->   26 x  26 x 128
  34 route  33 23                                  ->   26 x  26 x 384 
  35 conv    256       3 x 3/ 1     26 x  26 x 384 ->   26 x  26 x 256 1.196 BF
  36 conv    255       1 x 1/ 1     26 x  26 x 256 ->   26 x  26 x 255 0.088 BF
  37 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
Total BFLOPS 6.910 
avg_outputs = 310203 
Loading weights from ../finroc_projects_scout/third_party/darknet/yolov4-tiny.weights...
 seen 64, trained: 0 K-images (0 Kilo-batches_64) 
Done! Loaded 38 layers from weights-file 
Person YOLO model loaded: ../finroc_projects_scout/third_party/darknet/cfg/yolov4-tiny.cfg
mini_batch = 1, batch = 16, time_steps = 1, train = 0 
   layer   filters  size/strd(dil)      input                output
   0 conv     32       3 x 3/ 2    640 x 640 x   3 ->  320 x 320 x  32 0.177 BF
   1 conv     64       3 x 3/ 2    320 x 320 x  32 ->  160 x 160 x  64 0.944 BF
   2 conv     64       3 x 3/ 1    160 x 160 x  64 ->  160 x 160 x  64 1.887 BF
   3 route  2                                  1/2 ->  160 x 160 x  32 
   4 conv     32       3 x 3/ 1    160 x 160 x  32 ->  160 x 160 x  32 0.472 BF
   5 conv     32       3 x 3/ 1    160 x 160 x  32 ->  160 x 160 x  32 0.472 BF
   6 route  5 4                                    ->  160 x 160 x  64 
   7 conv     64       1 x 1/ 1    160 x 160 x  64 ->  160 x 160 x  64 0.210 BF
   8 route  2 7                                    ->  160 x 160 x 128 
   9 max                2x 2/ 2    160 x 160 x 128 ->   80 x  80 x 128 0.003 BF
  10 conv    128       3 x 3/ 1     80 x  80 x 128 ->   80 x  80 x 128 1.887 BF
  11 route  10                                 1/2 ->   80 x  80 x  64 
  12 conv     64       3 x 3/ 1     80 x  80 x  64 ->   80 x  80 x  64 0.472 BF
  13 conv     64       3 x 3/ 1     80 x  80 x  64 ->   80 x  80 x  64 0.472 BF
  14 route  13 12                                  ->   80 x  80 x 128 
  15 conv    128       1 x 1/ 1     80 x  80 x 128 ->   80 x  80 x 128 0.210 BF
  16 route  10 15                                  ->   80 x  80 x 256 
  17 max                2x 2/ 2     80 x  80 x 256 ->   40 x  40 x 256 0.002 BF
  18 conv    256       3 x 3/ 1     40 x  40 x 256 ->   40 x  40 x 256 1.887 BF
  19 route  18                                 1/2 ->   40 x  40 x 128 
  20 conv    128       3 x 3/ 1     40 x  40 x 128 ->   40 x  40 x 128 0.472 BF
  21 conv    128       3 x 3/ 1     40 x  40 x 128 ->   40 x  40 x 128 0.472 BF
  22 route  21 20                                  ->   40 x  40 x 256 
  23 conv    256       1 x 1/ 1     40 x  40 x 256 ->   40 x  40 x 256 0.210 BF
  24 route  18 23                                  ->   40 x  40 x 512 
  25 max                2x 2/ 2     40 x  40 x 512 ->   20 x  20 x 512 0.001 BF
  26 conv    512       3 x 3/ 1     20 x  20 x 512 ->   20 x  20 x 512 1.887 BF
  27 conv    256       1 x 1/ 1     20 x  20 x 512 ->   20 x  20 x 256 0.105 BF
  28 conv    512       3 x 3/ 1     20 x  20 x 256 ->   20 x  20 x 512 0.944 BF
  29 conv     18       1 x 1/ 1     20 x  20 x 512 ->   20 x  20 x  18 0.007 BF
  30 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
  31 route  27                                     ->   20 x  20 x 256 
  32 conv    128       1 x 1/ 1     20 x  20 x 256 ->   20 x  20 x 128 0.026 BF
  33 upsample                 2x    20 x  20 x 128 ->   40 x  40 x 128
  34 route  33 23                                  ->   40 x  40 x 384 
  35 conv    256       3 x 3/ 1     40 x  40 x 384 ->   40 x  40 x 256 2.831 BF
  36 conv     18       1 x 1/ 1     40 x  40 x 256 ->   40 x  40 x  18 0.015 BF
  37 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
Total BFLOPS 16.065 
avg_outputs = 709263 
Loading weights from ../finroc_projects_scout/third_party/darknet/unitree-go1-tiny_best.weights...
 seen 64, trained: 83 K-images (1 Kilo-batches_64) 
Done! Loaded 38 layers from weights-file 
Robot YOLO model loaded: ../finroc_projects_scout/third_party/darknet/unitree-go1-tiny.cfg
mini_batch = 1, batch = 1, time_steps = 1, train = 0 
   layer   filters  size/strd(dil)      input                output
   0 conv     16       3 x 3/ 1    416 x 416 x   3 ->  416 x 416 x  16 0.150 BF
   1 max                2x 2/ 2    416 x 416 x  16 ->  208 x 208 x  16 0.003 BF
   2 conv     32       3 x 3/ 1    208 x 208 x  16 ->  208 x 208 x  32 0.399 BF
   3 max                2x 2/ 2    208 x 208 x  32 ->  104 x 104 x  32 0.001 BF
   4 conv     64       3 x 3/ 1    104 x 104 x  32 ->  104 x 104 x  64 0.399 BF
   5 max                2x 2/ 2    104 x 104 x  64 ->   52 x  52 x  64 0.001 BF
   6 conv    128       3 x 3/ 1     52 x  52 x  64 ->   52 x  52 x 128 0.399 BF
   7 max                2x 2/ 2     52 x  52 x 128 ->   26 x  26 x 128 0.000 BF
   8 conv    256       3 x 3/ 1     26 x  26 x 128 ->   26 x  26 x 256 0.399 BF
   9 max                2x 2/ 2     26 x  26 x 256 ->   13 x  13 x 256 0.000 BF
  10 conv    512       3 x 3/ 1     13 x  13 x 256 ->   13 x  13 x 512 0.399 BF
  11 max                2x 2/ 1     13 x  13 x 512 ->   13 x  13 x 512 0.000 BF
  12 conv   1024       3 x 3/ 1     13 x  13 x 512 ->   13 x  13 x1024 1.595 BF
  13 conv    256       1 x 1/ 1     13 x  13 x1024 ->   13 x  13 x 256 0.089 BF
  14 conv    512       3 x 3/ 1     13 x  13 x 256 ->   13 x  13 x 512 0.399 BF
  15 conv    255       1 x 1/ 1     13 x  13 x 512 ->   13 x  13 x 255 0.044 BF
  16 yolo
[yolo] params: iou loss: mse (2), iou_norm: 0.75, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.00
  17 route  13                                     ->   13 x  13 x 256 
  18 conv    128       1 x 1/ 1     13 x  13 x 256 ->   13 x  13 x 128 0.011 BF
  19 upsample                 2x    13 x  13 x 128 ->   26 x  26 x 128
  20 route  19 8                                   ->   26 x  26 x 384 
  21 conv    256       3 x 3/ 1     26 x  26 x 384 ->   26 x  26 x 256 1.196 BF
  22 conv    255       1 x 1/ 1     26 x  26 x 256 ->   26 x  26 x 255 0.088 BF
  23 yolo
[yolo] params: iou loss: mse (2), iou_norm: 0.75, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.00
Total BFLOPS 5.571 
avg_outputs = 341534 
Loading weights from ../finroc_projects_scout/third_party/darknet/yolov3-tiny.weights...
 seen 64, trained: 32013 K-images (500 Kilo-batches_64) 
Done! Loaded 24 layers from weights-file 
Person YOLO model loaded: ../finroc_projects_scout/third_party/darknet/cfg/yolov3-tiny.cfg
mini_batch = 1, batch = 16, time_steps = 1, train = 0 
   layer   filters  size/strd(dil)      input                output
   0 conv     32       3 x 3/ 2    640 x 640 x   3 ->  320 x 320 x  32 0.177 BF
   1 conv     64       3 x 3/ 2    320 x 320 x  32 ->  160 x 160 x  64 0.944 BF
   2 conv     64       3 x 3/ 1    160 x 160 x  64 ->  160 x 160 x  64 1.887 BF
   3 route  2                                  1/2 ->  160 x 160 x  32 
   4 conv     32       3 x 3/ 1    160 x 160 x  32 ->  160 x 160 x  32 0.472 BF
   5 conv     32       3 x 3/ 1    160 x 160 x  32 ->  160 x 160 x  32 0.472 BF
   6 route  5 4                                    ->  160 x 160 x  64 
   7 conv     64       1 x 1/ 1    160 x 160 x  64 ->  160 x 160 x  64 0.210 BF
   8 route  2 7                                    ->  160 x 160 x 128 
   9 max                2x 2/ 2    160 x 160 x 128 ->   80 x  80 x 128 0.003 BF
  10 conv    128       3 x 3/ 1     80 x  80 x 128 ->   80 x  80 x 128 1.887 BF
  11 route  10                                 1/2 ->   80 x  80 x  64 
  12 conv     64       3 x 3/ 1     80 x  80 x  64 ->   80 x  80 x  64 0.472 BF
  13 conv     64       3 x 3/ 1     80 x  80 x  64 ->   80 x  80 x  64 0.472 BF
  14 route  13 12                                  ->   80 x  80 x 128 
  15 conv    128       1 x 1/ 1     80 x  80 x 128 ->   80 x  80 x 128 0.210 BF
  16 route  10 15                                  ->   80 x  80 x 256 
  17 max                2x 2/ 2     80 x  80 x 256 ->   40 x  40 x 256 0.002 BF
  18 conv    256       3 x 3/ 1     40 x  40 x 256 ->   40 x  40 x 256 1.887 BF
  19 route  18                                 1/2 ->   40 x  40 x 128 
  20 conv    128       3 x 3/ 1     40 x  40 x 128 ->   40 x  40 x 128 0.472 BF
  21 conv    128       3 x 3/ 1     40 x  40 x 128 ->   40 x  40 x 128 0.472 BF
  22 route  21 20                                  ->   40 x  40 x 256 
  23 conv    256       1 x 1/ 1     40 x  40 x 256 ->   40 x  40 x 256 0.210 BF
  24 route  18 23                                  ->   40 x  40 x 512 
  25 max                2x 2/ 2     40 x  40 x 512 ->   20 x  20 x 512 0.001 BF
  26 conv    512       3 x 3/ 1     20 x  20 x 512 ->   20 x  20 x 512 1.887 BF
  27 conv    256       1 x 1/ 1     20 x  20 x 512 ->   20 x  20 x 256 0.105 BF
  28 conv    512       3 x 3/ 1     20 x  20 x 256 ->   20 x  20 x 512 0.944 BF
  29 conv     18       1 x 1/ 1     20 x  20 x 512 ->   20 x  20 x  18 0.007 BF
  30 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
  31 route  27                                     ->   20 x  20 x 256 
  32 conv    128       1 x 1/ 1     20 x  20 x 256 ->   20 x  20 x 128 0.026 BF
  33 upsample                 2x    20 x  20 x 128 ->   40 x  40 x 128
  34 route  33 23                                  ->   40 x  40 x 384 
  35 conv    256       3 x 3/ 1     40 x  40 x 384 ->   40 x  40 x 256 2.831 BF
  36 conv     18       1 x 1/ 1     40 x  40 x 256 ->   40 x  40 x  18 0.015 BF
  37 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
Total BFLOPS 16.065 
avg_outputs = 709263 
Loading weights from ../finroc_projects_scout/third_party/darknet/unitree-go1-tiny_best.weights...
 seen 64, trained: 83 K-images (1 Kilo-batches_64) 
Done! Loaded 38 layers from weights-file 
Robot YOLO model loaded: ../finroc_projects_scout/third_party/darknet/unitree-go1-tiny.cfg
Runtime/tcp/hiwi-z890eaglewifi7-1<635647>::tRemoteRuntime [sources/cpp/plugins/network_transport/generic_protocol/tRemoteRuntime.cpp:171] >> Connected to hiwi-z890eaglewifi7-1<635647>
