set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")


if ("${WIFI_SSID}" STREQUAL "")
    message("Skipping some Pico W examples as WIFI_SSID is not defined")
elseif ("${WIFI_PASSWORD}" STREQUAL "")
    message("Skipping some Pico W examples as WIFI_PASSWORD is not defined")
else()
    include(../FreeRTOS_Kernel_import.cmake)
    add_subdirectory(tcp_server)
endif()
