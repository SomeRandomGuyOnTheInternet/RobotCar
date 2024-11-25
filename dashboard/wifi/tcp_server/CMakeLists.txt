# Add the TCP server executable for background mode
add_executable(dashboard_main
    picow_tcp_server.c
)
target_compile_definitions(dashboard_main PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)
target_include_directories(dashboard_main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..  # for our common lwipopts
)
target_link_libraries(dashboard_main
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
)
pico_add_extra_outputs(dashboard_main)

# Enable USB output for the background server
pico_enable_stdio_usb(dashboard_main 1)
pico_enable_stdio_uart(dashboard_main 0)  # Set to 1 if you want UART output instead
