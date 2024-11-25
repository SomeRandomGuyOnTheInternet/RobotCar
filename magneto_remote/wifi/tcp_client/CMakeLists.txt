# Add your executable for background TCP client
add_executable(picow_tcpip_client_background
        picow_tcp_client.c
)

target_compile_definitions(picow_tcpip_client_background PRIVATE
        WIFI_SSID= "${WIFI_SSID}"
        WIFI_PASSWORD= "${WIFI_PASSWORD}"
)

target_include_directories(picow_tcpip_client_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # For common lwipopts
        ${CMAKE_CURRENT_LIST_DIR}/../../magneto # Path to gy511.h
)

target_link_libraries(picow_tcpip_client_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        gy511_lib

)

# Enable USB output for the background server
pico_enable_stdio_usb(picow_tcpip_client_background 1)
pico_enable_stdio_uart(picow_tcpip_client_background 0)

# Create extra outputs for the background TCP client
pico_add_extra_outputs(picow_tcpip_client_background)

# Add your executable for polling TCP client
add_executable(picow_tcpip_client_poll
        picow_tcp_client.c
)

target_compile_definitions(picow_tcpip_client_poll PRIVATE
        WIFI_SSID= "${WIFI_SSID}"
        WIFI_PASSWORD= "${WIFI_PASSWORD}"
)

target_include_directories(picow_tcpip_client_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # For common lwipopts
)

target_link_libraries(picow_tcpip_client_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        gy511_lib # Link the gy511 library
        cyw43_arch
)

# Enable USB output for the polling server
pico_enable_stdio_usb(picow_tcpip_client_poll 1)
pico_enable_stdio_uart(picow_tcpip_client_poll 0)

# Create extra outputs for the polling TCP client
pico_add_extra_outputs(picow_tcpip_client_poll)
