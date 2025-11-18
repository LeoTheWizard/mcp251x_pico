cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(read_frames C CXX ASM)
pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../.. mcp251x_pico)

add_executable(read_frames
    main.c
)

# Enable USB serial output
pico_enable_stdio_usb(read_frames 1)
pico_enable_stdio_uart(read_frames 0)

# Link your library into the executable
target_link_libraries(read_frames
    mcp251x_pico
    pico_stdlib
)

pico_add_extra_outputs(read_frames)
