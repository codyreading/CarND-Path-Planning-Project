set (NAME types)

add_library(${NAME}
    src/state.hpp
    src/path.hpp
    src/frenet_point.hpp
    src/waypoints.hpp
    src/state.cpp
    src/path.cpp
    src/frenet_point.cpp
    src/waypoints.cpp
)

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)