add_executable(calibrator main.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(calibrator PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibrator PUBLIC ${OpenCV_LIBS})
