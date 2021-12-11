add_library(renderer
    arc.cpp
    circle.cpp
    circleBorder.cpp
    frame.cpp
    line.cpp
    messageBox.cpp
    polygon.cpp
    primitive.cpp
    rectangle.cpp
    triangle.cpp
    window.cpp
)

set(FILES_TO_COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/roboto.ttf
)

file(COPY ${FILES_TO_COPY}
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../..
)

target_include_directories(renderer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(renderer
    SDL2main
    SDL2
    SDL2_ttf
)