project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(SOURCES
    src/main.cpp
    src/utils.h
)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


# Add library
add_library(spline_fit
    src/spline-fit/spline_fit.cpp
    src/spline-fit/spline_fit.hpp
)

# Add excutable
add_executable(path_planning ${SOURCES})

# Link libraries
set(LIBRARIES
    z
    ssl
    uv
    uWS
    spline_fit
)

target_link_libraries(path_planning ${LIBRARIES})

# Add include directories
target_include_directories(path_planning PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/spline-fit"
                          "${PROJECT_SOURCE_DIR}/third-party/spline/src")
