# Find required packages for OpenGL window rendering
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

set(LIBREALSENSE_DIR /home/ssingh/Finroc/singh_files/aor_robotics_group/realsense_camera/librealsense)

# ImGui sources needed by example.hpp
set(IMGUI_SOURCES
    ${LIBREALSENSE_DIR}/third-party/imgui/imgui.cpp
    ${LIBREALSENSE_DIR}/third-party/imgui/imgui_draw.cpp
    ${LIBREALSENSE_DIR}/third-party/imgui/imgui_widgets.cpp
    ${LIBREALSENSE_DIR}/third-party/imgui/imgui_tables.cpp
    ${LIBREALSENSE_DIR}/third-party/imgui/imgui_impl_glfw.cpp
    ${LIBREALSENSE_DIR}/third-party/imgui/imgui_impl_opengl3.cpp
)

# Create the executable from your source file + ImGui
add_executable(frame_check main.cpp ${IMGUI_SOURCES})

# Include example.hpp and ImGui headers
target_include_directories(frame_check PRIVATE
    ${LIBREALSENSE_DIR}/examples
    ${LIBREALSENSE_DIR}/third-party/imgui
    ${LIBREALSENSE_DIR}/third-party/glfw/include
)

# Link realsense2, GLFW, OpenGL, and GLU
target_link_libraries(frame_check PRIVATE
    realsense2::realsense2
    glfw
    OpenGL::GL
    GLU
)