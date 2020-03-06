set (NAME json)

add_library(${NAME}
    src/json.hpp
    src/json.cpp
)

# Link libraries
set(LIBRARIES
)

target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(utils PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)