# Roboflow → Darknet: end-to-end guide

Build a dataset in Roboflow, export it as **YOLO Darknet**, train YOLOv4 on it.

No format conversion anywhere in this pipeline: a Roboflow "YOLO Darknet" export
is already darknet's native layout and label format. Export it right and there
is nothing to convert.

---

## Part 0 — Is darknet the right target?

Decide first; it saves a day if the answer is no.

Pick darknet if you need a self-contained C/C++ binary with no Python runtime —
e.g. inference inside a Finroc module. Pick Ultralytics if you want the best mAP
for the least effort; `../models/best.pt` is an existing baseline, and
`yolo export format=onnx` gives C++ inference via OpenCV DNN without retraining.

On a small single-class set, YOLOv4-tiny will likely score *below* that
baseline. Proceed anyway if the constraint is real.

---

## Part 1 — Build the dataset in Roboflow

### 1.1 Create the project

app.roboflow.com → **Create New Project**:

- **Project Type: Object Detection** — the only type that yields boxes.
- **Annotation Group**: what one box means, e.g. `robots`.

### 1.2 Upload images

Drag in images or video (Roboflow frame-splits video at a chosen FPS — sample at
1–2 FPS, not 30, or you get near-duplicate frames).

What actually drives accuracy, roughly in order:

- **Shoot from the deployment camera.** Train on RealSense frames if you deploy
  on RealSense. A model trained on phone photos degrades on a different sensor,
  lens, and exposure curve.
- **Vary distance, angle, lighting, background.** Include the robot partially
  occluded, at the frame edge, and truncated.
- **Include negatives** — images of the lab with no robot. These carry an
  *empty* `.txt` and measurably cut false positives. Aim for ~10% of the set;
  you must explicitly opt in to keeping null images when generating a version.
- **Quantity**: AlexeyAB's guidance is ~2000 images per class.

### 1.3 Annotate

One tight box per object instance.

- Box the object's full extent, including occluded parts within its silhouette.
- Label **every** instance. A missed robot teaches the model that robots are
  background — the most common cause of bad mAP.
- Be consistent on edge cases (is a robot 90% out of frame labelled?). Pick a
  rule, write it down, stick to it.

### 1.4 Split

**Train ~70% / Valid ~20% / Test ~10%.**

Split *before* augmentation, and keep near-duplicate frames from the same video
clip in the same split — otherwise validation frames nearly match training
frames and your mAP is fiction.

### 1.5 Preprocessing

- **Auto-Orient**: on. Strips EXIF rotation, which darknet ignores.
- **Resize**: to the network size you'll train at — 640×640. Prefer
  **"Fit (letterbox)"** over **"Stretch"**; darknet letterboxes at inference, so
  stretch is a train/serve mismatch.

Skip greyscale/contrast/tiling without a specific reason.

### 1.6 Augmentation

Darknet **already augments during training** from the cfg — `saturation=1.5`,
`exposure=1.5`, `hue=.1`, `jitter=.3`, `flip=1`, plus mosaic on YOLOv4.
Duplicating that in Roboflow only bloats the download.

Add only what darknet doesn't do: blur / noise (matches motion blur from a
walking robot). **Never** vertical flip or 90° rotation for a ground robot.

### 1.7 Export

**Generate** the version → **Download Dataset** → **Format: `YOLO Darknet`**.

Zip to computer, or use the snippet:

```bash
curl -L "https://universe.roboflow.com/ds/XXXX?key=YYYY" -o roboflow.zip
unzip roboflow.zip -d unitree-go1-darknet && rm roboflow.zip
```

### 1.8 What you get

```
unitree-go1-darknet/
├── train/  img.jpg + img.txt side by side, _darknet.labels
├── valid/
└── test/
```

Each `.txt` is one line per object: `class_id cx cy w h`, normalized 0–1, with
`cx`/`cy` the box **centre**. Darknet resolves a label by swapping the image's
extension for `.txt` — which is exactly this layout. Nothing to convert.

---

## Part 2 — Build darknet

Use **[hank-ai/darknet](https://github.com/hank-ai/darknet)** (Darknet V3), not
AlexeyAB's. That repo is archived and its hand-maintained `Makefile` `ARCH` list
has no `sm_120`, so it will not emit working kernels for this machine's
**RTX 5070 Ti** (Blackwell). The hank-ai fork is CMake-based and autodetects.

### Getting CUDA without sudo

This account is not in sudoers, so `apt install cuda-toolkit` is not an option —
and PyTorch's bundled CUDA doesn't help either. PyTorch ships the CUDA
*runtime* (pre-compiled kernels); darknet is C++ and needs the *toolkit*
(`nvcc`) to compile its own `.cu` kernels at build time.

The whole toolkit is pip-installable into the venv:

```bash
realsense_env/bin/pip install nvidia-cuda-nvcc nvidia-cuda-cccl
```

- `nvidia-cuda-nvcc` — the compiler (45 MB). Note: **not** `nvidia-cuda-nvcc-cu13`,
  which is deprecated and fails to build.
- `nvidia-cuda-cccl` — libcu++ headers. Without it the build dies on
  `fatal error: nv/target: No such file or directory`.
- `nvidia-cuda-runtime` was already present as a torch dependency.

CMake's `FindCUDAToolkit` expects a system-style layout, which the pip wheels
don't provide. Two symlink fixes inside `site-packages/nvidia/cu13/`:

```bash
CU=realsense_env/lib/python3.12/site-packages/nvidia/cu13
cd $CU/lib && for f in *.so.*; do ln -sf "$f" "${f%%.so.*}.so"; done   # unversioned .so names
cd $CU && ln -s lib lib64                                             # CMake looks in lib64/
```

Without these you get `Could NOT find CUDAToolkit (missing: CUDA_CUDART)`.

### Build

```bash
git clone https://github.com/hank-ai/darknet ~/darknet
CU=$PWD/realsense_env/lib/python3.12/site-packages/nvidia/cu13
mkdir -p ~/darknet/build-gpu && cd ~/darknet/build-gpu
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CUDA_COMPILER=$CU/bin/nvcc \
      -DCUDAToolkit_ROOT=$CU \
      -DCMAKE_CUDA_ARCHITECTURES=120 \
      ..
make -j$(nproc)
```

`sudo make install` is **not** needed — run the binary from the build tree.

Two gotchas:
- The build dir must be **inside the git clone**. Darknet's CMake runs `git
  describe` for its version string; building in `/tmp` fails with
  `VERSION ".." format invalid`.
- `-DCMAKE_CUDA_ARCHITECTURES=120` targets sm_120 (Blackwell, RTX 5070 Ti).

Verify:

```bash
./darknet.sh --version    # must list your GPU, not "compiled to use the CPU"
```

`darknet.sh` is a thin wrapper that sets `LD_LIBRARY_PATH` to the pip CUDA libs —
without it the binary won't find `libcudart.so.13` at runtime.

Measured on an RTX 5070 Ti: **0.54 s/iteration** at 640×640 batch 64, so 2000
iterations ≈ 18 minutes. The CPU build was 23 s/iteration — about 40 hours.

---

## Part 3 — Index files

```bash
python3 prepare_dataset.py ../unitree-go1-darknet
```

A Roboflow export has no `train.txt`, `obj.names`, or `obj.data` — darknet
requires all three, so this generates them (it does not touch your labels):

| file | contents |
|---|---|
| `train.txt` | one absolute image path per line |
| `valid.txt` | same, validation split |
| `obj.names` | copied from `_darknet.labels` — **order defines class ids** |
| `obj.data` | points darknet at the above + `backup/` |

Images with no matching `.txt` are skipped and reported. Paths are absolute, so
re-run if the repo moves.

The **test** split is deliberately excluded — keep it untouched for a final
honest `map` run.

## Part 4 — Config

```bash
./prepare_cfg.sh 1              # classes; add `tiny|full` and size to override
```

Downloads `yolov4-tiny.cfg` + the `yolov4-tiny.conv.29` pretrained backbone,
writes `unitree-go1-tiny.cfg` with the custom-class edits:

| setting | value | rule |
|---|---|---|
| `classes` | 1 | every `[yolo]` block |
| `filters` | 18 | `(classes + 5) * 3`, conv **immediately before** each `[yolo]` |
| `max_batches` | 6000 | `classes * 2000`, floored at 6000 |
| `steps` | 4800,5400 | 80% / 90% of `max_batches` |
| `width`/`height` | 640 | match the export; multiple of 32 |
| `batch`/`subdivisions` | 64 / 16 | 16 GB VRAM handles this at 640 |

`filters` is the one everyone gets wrong: **not** `classes+5`, and only in the
conv *before* each `[yolo]`. The script verifies the patched count equals the
number of `[yolo]` blocks and fails loudly otherwise.

### Why `max_batches` is so much larger than an Ultralytics epoch count

`yolov4-tiny.conv.29` holds **layers 0–28 only** — the COCO-trained backbone,
extracted from `yolov4-tiny.weights` with `darknet partial`. Layers 29–37,
including both `[yolo]` detection heads, are randomly initialized: the COCO head
can't be reused because its output shape (`filters=255` for 80 classes) doesn't
match a 1-class model (`filters=18`).

Ultralytics' `yolov8n.pt` instead ships the **whole** COCO detector and swaps
only the final class-score conv, so 50 epochs of fine-tuning suffices. Darknet
has to learn detection from scratch on top of borrowed features, which needs far
more iterations.

Conversion for this dataset: `iterations ≈ epochs × 306 / 64 ≈ epochs × 4.8`.
So 6000 iterations ≈ 1255 epochs.

`max_batches` is also not a standalone knob — `burn_in=1000` ramps the learning
rate over the first 1000 iterations, and `steps` drops it at 80%/90%. Setting
`max_batches=239` ("50 epochs") would end training mid-burn-in, before the LR
ever reached its target or either step fired. If you shorten the schedule, scale
`burn_in` and `steps` with it.

In practice you don't need to: with `-map`, darknet saves `_best.weights`
whenever validation mAP improves, so just stop when mAP plateaus.

## Part 5 — Train

The build here is **Darknet V5 "Moonlit"**, whose CLI differs from V3/AlexeyAB:
options are single-word (`-dontshow`, not `-dont_show`) and there is no
`-mjpeg_port`. The binary lives at `~/darknet/build/src-cli/darknet` — `sudo
make install` is not required and this account is not in sudoers anyway.

```bash
~/darknet/build/src-cli/darknet detector train \
    obj.data unitree-go1-tiny.cfg yolov4-tiny.conv.29 -map -dontshow
```

- `-map` evaluates on `valid.txt` every 100 iterations and saves
  `backup/unitree-go1-tiny_best.weights` at the peak. **Deploy that**, not
  `_final.weights` — the last iteration is usually overfit.
- `-dontshow` suppresses the X window, so the run survives ssh. Use `-chart` to
  write the loss chart to a file instead.
- On a CPU-only build, V5 interactively demands you type `yes` before training.
  Pipe it in (`echo yes | darknet ...`) for a non-interactive smoke test.
- CUDA OOM → raise `subdivisions` to 32 or 64 (slower, same result).
- Resume: swap the pretrained weights for `backup/unitree-go1-tiny_last.weights`.

Avg loss should settle around 0.5–1.5. If mAP flattens well before 6000
iterations, stop and take the best checkpoint.

## Part 6 — Evaluate

```bash
darknet detector map obj.data unitree-go1-tiny.cfg backup/unitree-go1-tiny_best.weights
```

Read **mAP@0.50**. Then point `valid=` in `obj.data` at a `test.txt` for the
untouched split — if that's far below validation mAP, the splits leaked
near-duplicate frames (see 1.4).

```bash
darknet detector test obj.data unitree-go1-tiny.cfg \
    backup/unitree-go1-tiny_best.weights ../../recordings/dog3.jpeg -thresh 0.4
```

## Part 7 — Deploy

OpenCV 4.6 is installed and its DNN module loads darknet weights directly — no
linking against darknet:

```python
net = cv2.dnn.readNetFromDarknet("unitree-go1-tiny.cfg",
                                 "backup/unitree-go1-tiny_best.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
```

The CUDA backend needs an OpenCV built with CUDA; stock Ubuntu `libopencv-dev`
is CPU-only, so fall back to `DNN_BACKEND_OPENCV` / `DNN_TARGET_CPU` otherwise.
The same two-line API exists in C++ for the Finroc side.

## Troubleshooting

| symptom | cause |
|---|---|
| loss = `nan` from iteration 1 | `filters` ≠ `(classes+5)*3` |
| `Couldn't open file: .../x.txt` | stale `train.txt` — re-run `prepare_dataset.py` |
| `No _darknet.labels found` | exported as YOLOv8/COCO — re-export as **YOLO Darknet** |
| mAP stays 0 | `obj.names` order ≠ class ids in the `.txt` files |
| trains but detects nothing | forgot the pretrained `.conv` weights, or `max_batches` too low |
| CUDA OOM | raise `subdivisions` |
| slow, GPU idle | darknet built without CUDA — recheck the `cmake` summary |
