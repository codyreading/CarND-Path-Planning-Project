set (NAME path)

add_library(${NAME}
    src/path_generator.hpp
    src/path_generator.cpp
    src/path_validator.hpp
    src/path_validator.cpp
)

# Link libraries
set(LIBRARIES
    types
    state_machine
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../Eigen-3.3")

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)