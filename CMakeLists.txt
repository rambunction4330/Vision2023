cmake_minimum_required(VERSION 3.16)
project(Vision2023)

set(CMAKE_CXX_STANDARD 17)

set(BUILD_PYTHON_WRAPPER OFF)
add_subdirectory(third_party/AprilTag)
add_subdirectory(third_party/argparse)

find_package(OpenCV REQUIRED)

add_executable(vision src/main.cpp)
target_link_libraries(vision PUBLIC ${OpenCV_LIBS} apriltag argparse)
target_include_directories(vision PUBLIC ${OpenCV_INCLUDE_DIRS})

add_subdirectory(tools/calibration)
add_subdirectory(tools/camera)