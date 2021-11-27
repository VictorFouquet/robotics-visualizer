add_library(renderer
    arc.cpp
    circle.cpp
    circleBorder.cpp
    frame.cpp
    line.cpp
    polygon.cpp
    primitive.cpp
    rectangle.cpp
    triangle.cpp
    window.cpp
)

target_include_directories(renderer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(renderer
    SDL2main
    SDL2
)