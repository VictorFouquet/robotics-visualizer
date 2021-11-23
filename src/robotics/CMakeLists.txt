add_library(robotics
    robotArmExample.cpp
)

target_include_directories(robotics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robotics
    linear_algebra
)
