set (NAME types)

add_library(${NAME}
    src/path.hpp
    src/frenet_point.hpp
    src/waypoints.hpp
    src/path.cpp
    src/frenet_point.cpp
    src/waypoints.cpp
    src/vehicle.hpp
    src/vehicle.cpp
)

# Link libraries
set(LIBRARIES
    spline
)

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)