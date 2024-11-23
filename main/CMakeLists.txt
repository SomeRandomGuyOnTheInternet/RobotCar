cmake_minimum_required(VERSION 3.13)

# Set the project name
project(RoboticCar)

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
    ${CMAKE_CURRENT_LIST_DIR}/../wifi
    ${CMAKE_CURRENT_LIST_DIR}/../wifi/tcp_server # Include tcp_client folder
    ${CMAKE_CURRENT_LIST_DIR}/../ir/barcode
)

target_compile_definitions(main PRIVATE
PICO_MAX_SHARED_IRQ_HANDLERS=8
)

# Pull in common dependencies
target_link_libraries(main 
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
    hardware_adc
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

link_directories(${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch)

# Create map/bin/hex file etc.
pico_add_extra_outputs(main)

# Enable USB output, disable UART output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
