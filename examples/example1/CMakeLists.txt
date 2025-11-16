cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(build_test1 C CXX ASM)
pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../.. mcp251x_pico)

add_executable(build_test1
    main.c
)

# Link your library into the executable
target_link_libraries(build_test1
    mcp251x_pico
    pico_stdlib
)

pico_add_extra_outputs(build_test1)
