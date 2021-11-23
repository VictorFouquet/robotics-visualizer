cmake_minimum_required(VERSION 3.15)

project(RoboticsVisualizer VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(RoboticsVisualizer
    src/main.cpp
)

add_subdirectory(src)

target_link_libraries(RoboticsVisualizer
    app
)