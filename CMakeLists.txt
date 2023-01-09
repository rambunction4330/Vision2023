cmake_minimum_required(VERSION 3.24)
project(AprilTagTest)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party/AprilTag)

find_package(OpenCV REQUIRED)

add_executable(AprilTagTest main.cpp)
message("LIBS: ${OpenCV_LIBS}")
target_link_libraries(AprilTagTest)
