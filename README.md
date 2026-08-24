# Scout Project

`scout` is a Finroc project that builds the `ScoutControl` application and the supporting control, hardware interface, and safety demo libraries.

## Read this first

This README does not replace the main Finroc README.

Before following the steps below, first read the main repository README and run the required Ubuntu setup step there:

```bash
cd ~/Finroc/finroc
make run-setup
```

All commands in this README must also be run from the Finroc repository root:

```bash
cd ~/Finroc/finroc
```

Do not run the commands from inside `sources/cpp/projects/scout`.

The main Finroc README covers:

- general Ubuntu setup
- `make run-setup`
- general Finroc rebuilds
- `finstruct`

This README only covers the `scout` project itself.

## Build `ScoutControl`

Run exactly this:

```bash
cd ~/Finroc/finroc
source scripts/setenv -p scout
make ScoutControl-bin
```

If the build succeeds, the executable is:

```bash
./export/linux_x86_64_debug/bin/ScoutControl
```

## Run `ScoutControl`

Run exactly this:

```bash
cd ~/Finroc/finroc
source scripts/setenv -p scout
./export/linux_x86_64_debug/bin/ScoutControl
```

If the environment is already loaded in your current shell, this shorthand usually works too:

```bash
ScoutControl
```

## Build and run `finstruct`

`ScoutControl` is typically used together with `finstruct`.

Build `finstruct` with:

```bash
cd ~/Finroc/finroc
source scripts/setenv -l java
make finstruct
```

Run `finstruct` with:

```bash
cd ~/Finroc/finroc
source scripts/setenv -l java
finstruct
```

If the command is not found, use the explicit path:

```bash
./export/linux_x86_64_debug/bin/finstruct
```

## Recommended startup order

If you want to use `ScoutControl` together with `finstruct`, use two terminals.

Terminal 1:

```bash
cd ~/Finroc/finroc
source scripts/setenv -p scout
make ScoutControl-bin
./export/linux_x86_64_debug/bin/ScoutControl
```

Terminal 2:

```bash
cd ~/Finroc/finroc
source scripts/setenv -l java
make finstruct
finstruct
```

## Darknet / YOLO note

The `scout` safety demo contains perception-related code that can use Darknet / YOLO.

Inference is implemented: `mPercept` runs two Darknet models per camera — a
person model (`yolov4-tiny`) and the custom Unitree robot model
(`unitree-go1-tiny`). If a model fails to load, that percept falls back to
publishing the raw camera feed with a message overlay, so the app still runs.

`libdarknet.so` is built locally (confirmed by `version of package darknet: local`
in the build output).

To rebuild Darknet if the shared library is missing:

```bash
cd ~/Finroc/finroc_projects_scout/third_party/darknet
make LIBSO=1 libdarknet.so
```

## Python YOLO detector (optional)

`mPercept` can run detection two ways:

- **Local Darknet** (default) — Finroc runs the person (`yolov4-tiny`) and custom
  robot (`unitree-go1-tiny`) models itself.
- **Python bridge** — Finroc sends each camera frame over TCP to a Python server
  that runs the Ultralytics models (`yolov8n.pt` = person, `best.pt` = Unitree
  robot) and returns the boxes. Finroc runs no YOLO in this mode. Use this to run
  your PyTorch `.pt` models directly (OpenCV 4.6 cannot load YOLOv8 ONNX).

Data flow: `Finroc (camera) → JPEG frame → Python (YOLO) → boxes → Finroc (risk)`.

**Run the bridge:**

1. Recreate the venv if needed and start the server (terminal A):

   ```bash
   cd ~/Finroc/finroc_projects_scout/realsense_camera
   python3 -m venv realsense_env && realsense_env/bin/pip install -r requirements.txt
   source realsense_env/bin/activate
   python3 python/finroc_bridge/yolo_server.py     # listens on 127.0.0.1:5555
   ```

2. Start `ScoutControl` (terminal B) and, in `finstruct`, set the percept
   parameter **`Use Python Detector = true`** (per percept: Front / Rear).
   Related parameters: `Python Host` (default `127.0.0.1`), `Python Port` (5555).

Notes:

- Default is `false`, so without the server ScoutControl behaves exactly as
  before (local Darknet). If the server is not reachable it logs
  `"Python detector unavailable"`, keeps retrying, and never blocks.
- Only one process can hold the camera; the bridge avoids conflict because
  Finroc owns the camera and Python only receives frames.
- The Python server uses the **GPU automatically** when CUDA is available
  (it prints `Device: cuda:0` on start) and warms up the models at startup so
  the first real frame isn't slow.

**Test the server without Finroc/camera:**

Send a single image and see the boxes it returns (writes an annotated
`test_client_result.jpg`):

```bash
cd ~/Finroc/finroc_projects_scout/realsense_camera
source realsense_env/bin/activate
python3 python/finroc_bridge/test_client.py unitree-go1-1/test/images/<some_image>.jpg
```

## Pausing a percept / reducing lag

Each percept (Front / Rear) is a separate `mPercept`. Both run on the same
thread, so disabling one shortens the whole loop and reduces lag — useful when
only one physical camera is present (the rear just reprocesses the front feed).

In `finstruct`, on `Rear Percept` (or Front):

- **`Enabled = false`** — fully pauses the percept: no detection, no TCP call,
  no camera processing at all. (Its outputs stop updating, so its lamp will
  show the camera-timeout state.)
- **`Max Inference FPS`** — throttle instead of fully stopping. A small value
  like `0.5` runs detection ~once every 2 s (near-paused) while the camera view
  stays live. Do **not** set it to `0` — that means "no throttle" (runs every
  frame, the opposite).

Detection is the expensive part (YOLO inference / TCP round-trip); drawing the
camera view is cheap, so throttling detection removes almost all of the lag.

## If something fails

- If the general Finroc build fails, go back to the main repository README and run `make run-setup`.
- If `finstruct` reports that `dot` is missing, install `graphviz`.
- If `ScoutControl` is not found after the build, make sure you ran `source scripts/setenv -p scout` from the Finroc repository root before building.
- If you are unsure about the general Finroc environment, always start with the main [README.md](../finroc/README.md).
- If Darknet is missing, rebuild it with `make LIBSO=1 libdarknet.so` inside `finroc_projects_scout/third_party/darknet`.


# HOW TO DELETE ENV & BRING IT BACK WHEN NOT NEEDED FOR PYTHON PROTOTYPING
rm -rf realsense_camera/realsense_env    # reclaim 7.1 GB
# recreate anytime:
python3 -m venv realsense_env && realsense_env/bin/pip install -r requirements.txt


#install librealsense in system when using python

# if want to delete earlier instance and delete

pgrep -a ScoutControl        # see what's running
pkill -INT ScoutControl      # clean stop (Ctrl+C equivalent, turns lamps off)
# if still stuck after a few seconds:
pkill -KILL ScoutControl     # force kill


How to record screen :  ffmpeg -f x11grab -framerate 30 -i :0.0 -c:v libx264 -preset ultrafast output1.mp4


challenges faced

1-Both mCH341UsbSignalLight modules get created unconditionally, each opening its own independent serial connection, each running on its own update cycle, each continuously writing whatever its own upstream risk pipeline (front camera's risk vs. rear camera's risk — different YOLO models, different timing) says. When they land on the same physical device, they fight over it non-stop. That's almost certainly your real flicker sourc

solved by putting 2 lamps and one glows red and other works as planned.

2-YOLO detections flicker frame-to-frame (person/robot detected then not, object still there) -> risk_level bounced 0/1 every frame since a track only counted while matched THIS exact frame.
solved: mRiskAssessment now keeps scoring a track for a few frames after it's briefly missed (using its last known distance/velocity), so one dropped detection frame doesn't read as "object gone". Also split the Python YOLO server's single CONF into HUMAN_CONF/ROBOT_CONF so each model can be tuned separately.

3-CONTACT (max risk) falsely latched when an object was just far-but-fast (TTC-boosted) or calmly withdrawn, since the trigger only checked the abstract risk_state_ level, not real distance.
solved: CONTACT now also requires the last tracked object to have genuinely been within par_contact_distance_m right before it disappeared, not just a high risk score.

4-C++ safety demo (Darknet) couldn't detect the custom Unitree robot at all: our trained model was Ultralytics (best.pt), and Darknet can't load .pt files -- only .cfg+.weights.
solved: added a Python TCP bridge (python/finroc_bridge/yolo_server.py) that runs the real Ultralytics models and sends detections to Finroc over a socket; also separately trained a Darknet-format robot cfg/weights as a second option.

5-RealSense point cloud arrives as one flat list of 3D points with no width/height attached, so there's no direct way to look up "the depth under this detection's bounding box".
solved: mPercept infers the grid size from the point count (matches color image size when depth is aligned to color, or a known RealSense resolution) and samples a small patch at each box's centre.

6-Only one physical RealSense camera and one physical signal lamp are connected, but the code assumes front+rear pairs -- rear silently falls back to reusing front's camera stream and front's serial device, which caused the lamp collision in #1 and means rear risk assessment isn't seeing an independent view.

7-git: `git add .` staged the vendored realsense_camera/librealsense checkout as an embedded repo (warns instead of committing its files), and a `.gitignore` edit that narrowed from ignoring all of realsense_camera/ down to just the venv briefly caused 1000+ build artifacts to be stageable.
solved: added realsense_camera/librealsense and realsense_camera/build to .gitignore explicitly; unstaged before committing.

8-realsense_camera Python prototype had accumulated cruft from iterating fast: three duplicate copies of yolov8n.pt, a stale hsv_config.yaml no longer read by any script, and a `//` C-style comment left at the top of depth_estimation.py that was a silent Python SyntaxError (import would fail).
solved: de-duplicated model files, removed the dead config, moved one-off test scripts into trials/, fixed the syntax error.


risk level 0-4 reference (mRiskAssessment)

Base formula: thr_X = cWEIGHT_LIVING(3.0) / par_distance_X_m. This converts a
distance parameter into a "risk score" number, always using PERSON's weight,
regardless of what's actually detected. A class's own weight (person=3.0,
robot/vehicle=2.0, other=1.0) is what actually differentiates it -- it scales
the live score (r = weight * 1/distance * velocity-term), not the threshold.
So a robot needs to be proportionally closer than a person for the same score.

Base thresholds, as currently tuned (par_distance_low/medium/high/extreme_m = 4/3/2/1):

| Level | par_distance_*_m | thr_* (score units) |
|---|---|---|
| LOW | 4m | 0.75 |
| MEDIUM | 3m | 1.00 |
| HIGH | 2m | 1.50 |
| EXTREME | 1m | 3.00 |

Class distance cutoffs (score threshold above converted to metres per class, distance = class_weight / thr_*):

| Level | PERSON (w=3.0) | ROBOT/vehicle (w=2.0) | OTHER (w=1.0) |
|---|---|---|---|
| EXTREME | <= 1.00m | <= 0.67m | <= 0.33m |
| HIGH | <= 2.00m | <= 1.33m | <= 0.67m |
| MEDIUM | <= 3.00m | <= 2.00m | <= 1.00m |
| LOW | <= 4.00m | <= 2.67m | <= 1.33m |

These numbers move if par_distance_*_m is retuned -- recompute via class_weight / (3.0 / par_distance_X_m).

Hysteresis (par_level_hysteresis_pct, default 15%): the level doesn't reset from
distance alone each frame -- it starts from wherever it currently is, and only
moves once the score clears a wider "rise" bar (thr * 1.15) or drops below a
narrower "fall" bar (thr * 0.85). So a level, once reached, resists dropping
back out from small jitter right at the boundary. In score units, at the
thresholds above:

| Level | Rise bar (enter from below) | Fall bar (drop out) |
|---|---|---|
| LOW | >= 0.86 | < 0.64 |
| MEDIUM | >= 1.15 | < 0.85 |
| HIGH | >= 1.73 | < 1.28 |
| EXTREME | >= 3.45 | < 2.55 |

The live score is published as "Risk Score" (out_risk_score) on the
Front/Rear Risk Assessment module in finstruct -- watch it against this
table alongside "Risk Level"/"Risk Label" to see exactly why a level changed.

Approach Velocity / Time To Collision are not on the distance table -- they can override it. If the object is closing (Approach Velocity > 0) and Time To Collision < 2.0s (par_ttc_extreme_s) -> forced to at least the EXTREME threshold regardless of distance. If TTC is between 2.0s and 4.0s (par_ttc_high_s) -> forced to at least HIGH. So a person at 3m (LOW/NONE by distance alone) showing TTC = 1.5s will jump to EXTREME.

Nearest Class itself is not thresholded to a level -- it just picks which of the three weight columns above the live score uses.

CONTACT is not part of this 0-4 table. It is a separate state that only fires when the object disappears while the smoothed level was already >= 3 (par_contact_from_level) AND its last known distance was <= 0.15m (par_contact_distance_m). 


wright lving alswas in the formula/distance not object detectin class-- its not part of class specific threshold only in R fomrula. ("everything is measured in person-equivalent danger units"). 

always keep fps 30 or its flickers