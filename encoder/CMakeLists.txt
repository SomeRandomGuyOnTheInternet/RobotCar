add_executable(encoder encoder.c)

target_include_directories(encoder PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../motor
)

target_link_libraries(encoder
    pico_stdlib
    hardware_pwm
    motor_lib
)

pico_enable_stdio_usb(encoder 1)

pico_add_extra_outputs(encoder)

add_library(encoder_lib
    encoder.c
    encoder.h
)

target_link_libraries(encoder_lib
    pico_stdlib
    hardware_pwm
    motor_lib
)

pico_enable_stdio_usb(encoder_lib 1)
