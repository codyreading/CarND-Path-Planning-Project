set (NAME utils)

add_library(${NAME}
    src/utils.hpp
    src/utils.cpp
)

# Link libraries
set(LIBRARIES
    types
)

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(utils PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)