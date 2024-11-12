add_executable(motor motor.c
../encoder/encoder.c)

include(../FreeRTOS_Kernel_import.cmake)

# Add FreeRTOS include directories
target_include_directories(motor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(motor PRIVATE ${FREERTOS_SOURCE_DIR}/include)
target_include_directories(motor PRIVATE ${FREERTOS_SOURCE_DIR}/portable/ThirdParty/GCC/RP2040/include)

# Link against FreeRTOS Kernel
target_link_libraries(motor FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(motor 
        pico_stdlib 
        hardware_pwm
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
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

# pull in common dependencies
target_link_libraries(motor_lib
    pico_stdlib 
    hardware_pwm
    )

# create map/bin/hex file etc.
pico_enable_stdio_usb(motor_lib 1)
