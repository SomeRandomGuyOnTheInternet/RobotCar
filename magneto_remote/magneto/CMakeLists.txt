add_library(gy511_lib
    gy511.c
    gy511.h
)

# Link the necessary libraries to gy511_lib
target_link_libraries(gy511_lib
    pico_stdlib
    hardware_i2c
)

# Create the executable (gy511)
add_executable(gy511
    gy511.c  # Use main.c here, where your main logic resides
)

target_include_directories(gy511 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    include_directories("${PICO_SDK_PATH}/sdk/2.0.0/src/rp2_common/hardware_i2c/include")
)

# Link gy511 executable to libraries
target_link_libraries(gy511
    pico_stdlib
    hardware_i2c
    gy511_lib  # Link to your gy511_lib here
)

# Enable USB output (if using USB for communication)
pico_enable_stdio_usb(gy511 1)
pico_enable_stdio_uart(gy511 0)

# Create the output (UF2 binary for flashing onto the Pico)
pico_add_extra_outputs(gy511)
pico_enable_stdio_usb(gy511_lib 1)
