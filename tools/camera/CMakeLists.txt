add_executable(camera main.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(camera PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera PUBLIC ${OpenCV_LIBS})