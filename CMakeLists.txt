project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(SOURCES
    src/main.cpp
)

# Add libraries
add_subdirectory(src/utils)
add_subdirectory(src/spline)
add_subdirectory(src/types)
add_subdirectory(src/planner)
add_subdirectory(src/json)

# Add excutable
add_executable(path_planning ${SOURCES})

# Link libraries
set(LIBRARIES
    z
    ssl
    uv
    uWS
    utils
    spline
    types
    planner
    json
)

target_link_libraries(path_planning ${LIBRARIES})

