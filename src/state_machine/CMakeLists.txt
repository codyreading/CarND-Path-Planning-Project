set (NAME state_machine)

add_library(${NAME}
    src/state_machine.hpp
    src/state_machine.cpp
)

# Link libraries
#set(LIBRARIES
#)

#target_link_libraries(${NAME} ${LIBRARIES})

target_include_directories(${NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)