cmake_minimum_required(VERSION 3.10)
project(realsense_cpp_projects)

# Ensure modern C++ support
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the RealSense library once for the whole project
find_package(realsense2 REQUIRED)

# Add your subdirectories
add_subdirectory(hello_realsense)
add_subdirectory(frame_check)