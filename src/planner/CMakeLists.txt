set (NAME planner)

add_library(${NAME}
    src/planner.hpp
    src/planner.cpp
)

# Link libraries
set(LIBRARIES
    types
    spline
    state_machine
    path
    cost_functions
)

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)