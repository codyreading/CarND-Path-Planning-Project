set (NAME collision_detector)

add_library(${NAME}
    src/collision_detector.hpp
    src/collision_detector.cpp
)

# Link libraries
set(LIBRARIES
    types
)

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)