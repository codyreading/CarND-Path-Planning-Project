add_library(spline
    src/spline.hpp
    src/spline.cpp
)

# Link libraries
set(LIBRARIES
    utils
)

target_link_libraries(spline ${LIBRARIES})

target_include_directories(spline PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)