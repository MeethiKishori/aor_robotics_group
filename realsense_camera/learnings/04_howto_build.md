# C++ Build & Run (CMake + RealSense)

---

## Standard CMake Build Workflow

This is the pattern to follow for every C++ project:

```bash
mkdir build      # Create build directory
cd build         # Enter build directory
cmake ..         # Configure (looks for CMakeLists.txt in parent folder)
make             # Compile
```

---

## Build and Run a RealSense C++ Project (e.g. frame_check)

Each C++ project has its own folder under `cpp/` with a `main.cpp` and `CMakeLists.txt`.
There is one shared `build/` folder at the `realsense_camera` level.

### Build All Projects (do once)

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera/build
cmake ..   # reads all CMakeLists.txt files and generates Makefiles
make       # compiles all C++ projects
```

### Run a Project

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera/build/frame_check
./frame_check
```

### Rebuild Only One Project (faster)

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera/build/frame_check
make
./frame_check
```

**Note:** If you add a new `cpp/` subfolder with a `CMakeLists.txt`, always re-run `cmake ..` and `make` from the `build/` folder.

---

## Run Python Scripts

Always activate the virtual environment first, then run the script.

### test_realsense.py

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera
source realsense_env/bin/activate
cd python
python3 test_realsense.py
```

### risk.py

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera
source realsense_env/bin/activate
cd python
python3 risk.py
```
