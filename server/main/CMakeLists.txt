add_compile_definitions(CYW43_LWIP=1
    PICO_MAX_SHARED_IRQ_HANDLERS=8
)

add_executable(server_main
        main.c
)

target_include_directories(server_main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..
)

target_compile_definitions(server_main PRIVATE
        PICO_MAX_SHARED_IRQ_HANDLERS=5
)

# pull in common dependencies
target_link_libraries(server_main
        pico_stdlib
)

# link_directories(${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch)

# create map/bin/hex file etc.
pico_add_extra_outputs(server_main)

# enable usb output, disable uart output
pico_enable_stdio_usb(server_main 1)
pico_enable_stdio_uart(server_main 1)