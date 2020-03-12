set (NAME cost_functions)

add_library(${NAME}
    src/cost_functions.hpp
    src/cost_functions.cpp
)

# Link libraries
set(LIBRARIES
    collision_detector
)

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)