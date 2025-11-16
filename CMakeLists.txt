cmake_minimum_required(VERSION 3.13)

set(LIBRARY_NAME mcp251x_pico)

# Set standards.
project(${LIBRARY_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# Check for SDK.
#if (NOT DEFINED PICO_SDK_INITIALIZED)
#    message(FATAL_ERROR "MISSING PICO SDK, please include library *after* initializing the pico_sdk.")
#endif()

# Add sources.
add_library(${LIBRARY_NAME}
    src/mcp251x.c
    src/platform/sleep_pico.c
    src/platform/spi.c
)

# Include directories
target_include_directories(${LIBRARY_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link pico libraries
target_link_libraries(${LIBRARY_NAME}
    pico_stdlib
    hardware_spi
    hardware_gpio
)