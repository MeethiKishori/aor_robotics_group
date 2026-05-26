here i will have all my learnings, no big file
git clone https://gitlab.rhrk.uni-kl.de/aor-students/finroc/finroc_projects_scout.git
always open bash in konsole,like terminal

how to use github

  git status, checks if its git folder
  git pull origin main --> from gitlab updated
  git checkout -b learnfinroc --> created new branch learnfinroc
  git commit -m "done some change here"
  git push origin learnfinroc --> pushed changes to my branch learnfinroc
  git config --list --> here you can update email and name, as it gets labeled while you commit
  git branch -a  --> to see all branches
  git branch --> check which branch you are on
  git checkout import --> will change branch to import from current branch
  git add . for staging all my changes
  git commit will commit all changes
  git switch import
  git pull origin import
  git switch -c chua
  git push -u origin chua
  git rebase import
  git branch -vv    -tracks where it goes to



in finroc
in finstruct, at autoupdate part, go to view change to port data and then auto update in right side of the toolbar


any time do this
cd ~/Finroc/finroc
make

alwaysregister new project in finroc
ok , scout is running, in finstruct


after i pulled latest changes, when camera was workig on dog, it gives below error
      |                           ^~~~~~~~~~~~~~~
make[1]: *** [Makefile.generated:33290: build/linux_x86_64_debug/libraries/camera/behaviors/mbbDisparityDegradation.o] Error 1
make: *** [Makefile:50: build] Error 2
ssingh@hiwi-z890eaglewifi7-1:~/Finroc/finroc$


now solved after make
 -lfinroc_plugins_runtime_construction -lfinroc_plugins_network_transport -lrrlib_mapping -lrrlib_machine_learning_appliance -lrrlib_geometry_basic_shapes -lrrlib_math_quaternion_legacy -lrrlib_localization_quaternion -lrrlib_canvas -lrrlib_coviroa_base -lrrlib_util_legacy -lrrlib_distance_data -lrrlib_distance_data_units -lfinroc_libraries_tree_stem_mapping_utils -lrrlib_distance_data_utils -lrrlib_mapping_transformations -lrrlib_tentacles -lfinroc_libraries_mapping_behaviors -lfinroc_libraries_localization_behaviors_utils -lrrlib_aspect_maps -lrrlib_mapping_transformations_clothoid -lrrlib_point_clouds -lrrlib_tentacles_i2bc -lrrlib_vehicle_kinematics
done





### RealSense Python Wrapper Setup

The RealSense SDK is originally written in C++. To use it from Python, I set up a virtual environment using the Python bindings from [librealsense](https://github.com/IntelRealSense/librealsense).

```bash
sudo apt install python3.12-venv
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera


when workign with python in realsense : ac env is in realsense camera folder
ssingh@hiwi-z890eaglewifi7-1:~/Finroc/singh_files/aor_robotics_group/realsense_camera$ source realsense_env/bin/activate
# Create and activate virtual environment
python3 -m venv realsense_env
source realsense_env/bin/activate
```

echo $VIRTUAL_ENV


Useful commands:
- `realsense-viewer` — open the GUI camera viewer
- `rs-enumerate-devices` — list all connected RealSense devices

Camera confirmed working: ![Camera output](image.png)

rs-save-to-disk
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


how to run test_realsense.py

how to run point cloud viewer

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera
source realsense_env/bin/activate
pip install open3d
cd python
python3 test_pointcloud.py
```

This opens an Open3D window and shows the live colored RealSense point cloud.

---

### How to build and run a RealSense C++ project (e.g. frame_check)

Each C++ project has its own folder under cpp/ with a main.cpp and CMakeLists.txt.
There is one shared build/ folder at the realsense_camera level.

Steps (do this once to build):

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera/build
cmake ..          # reads all CMakeLists.txt files and generates Makefiles
make              # compiles all C++ projects
```

To run frame_check after building:

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera/build/frame_check
./frame_check
```

If you add a new cpp/ subfolder with a CMakeLists.txt, always re-run cmake .. and make from the build/ folder.

To rebuild only one project (faster):

```bash
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera/build/frame_check
make
./frame_check
```



how to run launch - python3 python/launch/run_risk_roi_red_black.py

sudo chmod 666 /dev/ttyUSB0

hex_usb uses hex command to sent to usb and run the led and buzzer together

run_tower.py and it will check if tower works fine


how to run pyrealsense : realsense-viewer