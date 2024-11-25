add_executable(encoder encoder.c)

target_include_directories(encoder PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(encoder
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_timer
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(encoder 1)

pico_add_extra_outputs(encoder)

add_library(encoder_lib
    encoder.c
    encoder.h
)

target_include_directories(encoder_lib PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(encoder_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_timer
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(encoder_lib 1)
