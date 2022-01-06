add_library(geometry
    circle.cpp
    segment.cpp
)

target_include_directories(geometry PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(geometry
    linear_algebra
)