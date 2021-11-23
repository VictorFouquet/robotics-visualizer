add_library(linear_algebra
    matrix.cpp
    vector3d.cpp
)

target_include_directories(linear_algebra PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)