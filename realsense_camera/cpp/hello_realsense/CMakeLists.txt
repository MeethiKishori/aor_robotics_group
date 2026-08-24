# Create the executable from your source file
add_executable(hello_realsense main.cpp)

# Link the library using the target found in the root
target_link_libraries(hello_realsense PRIVATE realsense2::realsense2)