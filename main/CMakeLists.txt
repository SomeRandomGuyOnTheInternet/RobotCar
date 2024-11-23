# Enable the PICO SDK
include(../pico_sdk_import.cmake)

#include FreeRTOS
include(../FreeRTOS_Kernel_import.cmake)

# Initialize the SDK
pico_sdk_init()

add_compile_definitions(CYW43_LWIP=1
    PICO_MAX_SHARED_IRQ_HANDLERS=8
)

add_executable(main
        main.c
        ../wifi/tcp_server/picow_tcp_server.c  # Correct path to your TCP client source file
        ../ir/barcode/barcode.c
)

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_CURRENT_LIST_DIR}/../motor
        ${CMAKE_CURRENT_LIST_DIR}/../encoder
        ${CMAKE_CURRENT_LIST_DIR}/../ultrasonic
        ${CMAKE_CURRENT_LIST_DIR}/../wifi
        ${CMAKE_CURRENT_LIST_DIR}/../wifi/tcp_server # Include tcp_client folder
        ${CMAKE_CURRENT_LIST_DIR}/../ir/barcode
)

target_compile_definitions(main PRIVATE
        PICO_MAX_SHARED_IRQ_HANDLERS=5
)

# pull in common dependencies
target_link_libraries(main
        pico_stdlib
        pico_cyw43_arch_none
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_adc
        hardware_pwm
        hardware_gpio
        hardware_timer
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        motor_lib
        encoder_lib
        ultrasonic_lib
)

# link_directories(${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
