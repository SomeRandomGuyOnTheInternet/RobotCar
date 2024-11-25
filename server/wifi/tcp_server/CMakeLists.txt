# Add the TCP server executable for background mode
add_executable(server_dashboard
    picow_tcp_server.c
)
target_compile_definitions(server_dashboard PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)
target_include_directories(server_dashboard PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..  # for our common lwipopts
)
target_link_libraries(server_dashboard
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
)
pico_add_extra_outputs(server_dashboard)

# Enable USB output for the background server
pico_enable_stdio_usb(server_dashboard 1)
pico_enable_stdio_uart(server_dashboard 0)  # Set to 1 if you want UART output instead
