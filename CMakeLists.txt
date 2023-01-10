cmake_minimum_required(VERSION 3.24)
project(AprilTagTest)

set(CMAKE_CXX_STANDARD 17)

set(BUILD_PYTHON_WRAPPER OFF)
add_subdirectory(third_party/AprilTag)
add_subdirectory(third_party/argparse)

find_package(OpenCV REQUIRED)

add_executable(AprilTagTest src/main.cpp)
target_link_libraries(AprilTagTest PUBLIC ${OpenCV_LIBS} apriltag)
target_include_directories(AprilTagTest PUBLIC ${OpenCV_INCLUDE_DIRS})

add_subdirectory(tools/calibration)
add_subdirectory(tools/camera)