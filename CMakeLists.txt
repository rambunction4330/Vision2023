cmake_minimum_required(VERSION 3.16)
project(Vision2023)

set(BUILD_PYTHON_WRAPPER OFF)
set(CMAKE_CXX_STANDARD 17)


find_package(OpenCV REQUIRED)

add_executable(vision src/main.cpp src/VisualizationSystem.cpp src/VisualizationSystem.h src/FieldData.cpp src/FieldData.h src/ThreadedCaptureSystem.cpp src/ThreadedCaptureSystem.h src/Detection.cpp src/Detection.h src/PositionEstimation.cpp src/PositionEstimation.h)
target_link_libraries(vision PUBLIC ${OpenCV_LIBS} apriltag argparse)
target_include_directories(vision PUBLIC ${OpenCV_INCLUDE_DIRS} src)

if(IS_RPI)
    message("here")
    target_compile_definitions(vision PRIVATE VISION_GUI_OPENGL_ES2 HI)
endif()


add_subdirectory(tools/calibration)
add_subdirectory(tools/camera)
add_subdirectory(third_party/glfw)
add_subdirectory(third_party/glad)
add_subdirectory(third_party/AprilTag)
add_subdirectory(third_party/argparse)

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
target_link_libraries(imgui PUBLIC dl glfw)

# OpenGL
find_package(OpenGL)

# INCBIN
add_library(incbin STATIC third_party/incbin/incbin.c)
target_include_directories(incbin PUBLIC third_party/incbin)

if(UNIX)
    target_link_libraries(vision PUBLIC dl)
endif()

target_link_libraries(vision PUBLIC imgui glfw glad OpenGL::GL incbin)

add_executable(example-opencv src/opencv_demo.cc)
target_include_directories(example-opencv PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(example-opencv PUBLIC apriltag ${OpenCV_LIBS})
