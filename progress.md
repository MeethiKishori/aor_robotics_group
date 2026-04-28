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





## Personal Notes

### Safety Demo Design Observations

- The pipeline converts **continuous risk** (distance values) to **discrete risk levels** that humans can perceive via LED or buzzer.
- This is essentially **runtime risk modelling**: object detection feeds directly into risk assessment.
- Risk is based on **spatial proximity** to hazardous objects (currently: persons; could extend to dogs, moving objects, etc.).

---

### RealSense Python Wrapper Setup

The RealSense SDK is originally written in C++. To use it from Python, I set up a virtual environment using the Python bindings from [librealsense](https://github.com/IntelRealSense/librealsense).

```bash
sudo apt install python3.12-venv
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera

# Create and activate virtual environment
python3 -m venv realsense_env
source realsense_env/bin/activate
```

Useful commands:
- `realsense-viewer` — open the GUI camera viewer
- `rs-enumerate-devices` — list all connected RealSense devices

Camera confirmed working: ![Camera output](image.png)

---

### Standard C++ Build Workflow (CMake)

This is the pattern to follow for every C++ project:

```bash
mkdir build      # Create build directory
cd build         # Enter build directory
cmake ..         # Configure (looks for CMakeLists.txt in parent folder)
make             # Compile
```

---

### How the RealSense D435i Works

**Hardware components:**
- 2 IR cameras (grayscale sensors)
- 1 IR projector
- 1 RGB camera
- 1 IMU (inertial measurement unit, for motion tracking)

**Depth sensing process:**
1. The IR projector casts a structured light pattern onto the scene.
2. Both IR cameras capture the scene — the slight offset between them creates **disparity**.
3. **Stereo matching** computes the disparity between the two IR images.
4. **Triangulation** converts disparity into a depth value for each pixel.

**IR camera characteristics:**
- Captures grayscale images in a single wavelength band (~850 nm, near-infrared).
- Each pixel in the depth frame stores the distance from the camera: `depth[x, y]` → distance in metres (e.g. `depth[100, 200]` → `0.83 m`).

**Note:** Even with the IR projector disabled, the IR cameras still function — they passively capture ambient infrared light from the environment.