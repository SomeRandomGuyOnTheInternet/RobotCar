cmake_minimum_required(VERSION 3.13)

# Set the project name
project(RoboticCar)

# Enable the PICO SDK
include(../../pico_sdk_import.cmake)

# Initialize the SDK
pico_sdk_init()

add_compile_definitions(CYW43_LWIP=1)

add_executable(remote_main
    main.c
    ../wifi/tcp_client/picow_tcp_client.c  # Correct path to your TCP client source file
    ../magneto/gy511.c                               # Add your GY-511 source file here
)

target_include_directories(remote_main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../wifi
    ${CMAKE_CURRENT_LIST_DIR}/../wifi/tcp_client # Include tcp_client folder
    ${CMAKE_CURRENT_LIST_DIR}/../magneto
)

target_compile_definitions(remote_main PRIVATE
    PICO_MAX_SHARED_IRQ_HANDLERS=5
)

# Pull in common dependencies
target_link_libraries(remote_main 
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
    hardware_adc
    gy511_lib
)

link_directories(${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch)

# Create map/bin/hex file etc.
pico_add_extra_outputs(remote_main)

# Enable USB output, disable UART output
pico_enable_stdio_usb(remote_main 1)
pico_enable_stdio_uart(remote_main 1)
