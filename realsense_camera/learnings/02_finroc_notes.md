# Finroc Notes

In finstruct, at autoupdate part: go to **view** → change to **port data** → then **auto update** in right side of the toolbar.

Always register new project in finroc.
Scout is running in finstruct.

After any pull, rebuild:
```bash
cd ~/Finroc/finroc
make
```

## Build Error + Fix

After pulling latest changes (when camera was working on dog):

**Error:**
```
      |                           ^~~~~~~~~~~~~~~
make[1]: *** [Makefile.generated:33290: build/linux_x86_64_debug/libraries/camera/behaviors/mbbDisparityDegradation.o] Error 1
make: *** [Makefile:50: build] Error 2
```

**Fix:** just run `make` again. The linker flags that resolved it:
```
-lfinroc_plugins_runtime_construction -lfinroc_plugins_network_transport -lrrlib_mapping
-lrrlib_machine_learning_appliance -lrrlib_geometry_basic_shapes -lrrlib_math_quaternion_legacy
-lrrlib_localization_quaternion -lrrlib_canvas -lrrlib_coviroa_base -lrrlib_util_legacy
-lrrlib_distance_data -lrrlib_distance_data_units -lfinroc_libraries_tree_stem_mapping_utils
-lrrlib_distance_data_utils -lrrlib_mapping_transformations -lrrlib_tentacles
-lfinroc_libraries_mapping_behaviors -lfinroc_libraries_localization_behaviors_utils
-lrrlib_aspect_maps -lrrlib_mapping_transformations_clothoid -lrrlib_point_clouds
-lrrlib_tentacles_i2bc -lrrlib_vehicle_kinematics
```
