cmake_minimum_required(VERSION 3.16)
project(Vision2023)

set(CMAKE_CXX_STANDARD 17)

set(BUILD_PYTHON_WRAPPER OFF)
add_subdirectory(third_party/AprilTag)
add_subdirectory(third_party/argparse)

find_package(OpenCV REQUIRED)

add_executable(vision src/main.cpp src/VisualizationSystem.cpp src/VisualizationSystem.h)
target_link_libraries(vision PUBLIC ${OpenCV_LIBS} apriltag argparse)
target_include_directories(vision PUBLIC ${OpenCV_INCLUDE_DIRS})

add_subdirectory(tools/calibration)
add_subdirectory(tools/camera)
add_subdirectory(third_party/glfw)
add_subdirectory(third_party/glad)

# Imgui Configuration
add_library(imgui
        third_party/imgui/imgui.cpp
        third_party/imgui/imgui_demo.cpp
        third_party/imgui/imgui_draw.cpp
        third_party/imgui/imgui_tables.cpp
        third_party/imgui/imgui_widgets.cpp
        third_party/imgui/backends/imgui_impl_glfw.cpp
        third_party/imgui/backends/imgui_impl_opengl3.cpp
        )
target_include_directories(imgui PUBLIC third_party/imgui)

# OpenGL
find_package(OpenGL)

target_link_libraries(vision PUBLIC imgui glfw glad OpenGL::GL)