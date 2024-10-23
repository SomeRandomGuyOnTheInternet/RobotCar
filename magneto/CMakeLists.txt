add_executable(gy511
    gy511.c
)

target_include_directories(gy511 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../motor
)

target_link_libraries(gy511
    pico_stdlib
    hardware_i2c
    motor_lib
)

add_library(gy511_lib
    gy511.c
    gy511.h
)

# Link the necessary libraries
target_link_libraries(gy511_lib
    pico_stdlib
    hardware_i2c
)

# Enable USB output (if using USB for communication)
pico_enable_stdio_usb(gy511 1)
pico_enable_stdio_uart(gy511 0)

# Create the output (UF2 binary for flashing onto the Pico)
pico_add_extra_outputs(gy511)
pico_enable_stdio_usb(gy511_lib 1)

