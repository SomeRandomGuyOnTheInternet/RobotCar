add_executable(motor
        motor.c
)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(motor PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../encoder
)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(motor
        pico_stdlib
        hardware_adc
        hardware_pwm
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        encoder_lib
)

# enable usb output, disable uart output
pico_enable_stdio_usb(motor 1)
pico_enable_stdio_uart(motor 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(motor)

add_library(motor_lib
        motor.c
        motor.h
)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(motor_lib PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../encoder
)

# pull in common dependencies
target_link_libraries(motor_lib
        pico_stdlib
        hardware_adc
        hardware_pwm
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        encoder_lib
)

# create map/bin/hex file etc.
pico_enable_stdio_usb(motor_lib 1)
