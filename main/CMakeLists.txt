add_executable(main
        main.c
        )

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_CURRENT_LIST_DIR}/../ultrasonic
        ${CMAKE_CURRENT_LIST_DIR}/../motor
        ${CMAKE_CURRENT_LIST_DIR}/../encoder
        )

target_compile_definitions(main PRIVATE
        PICO_MAX_SHARED_IRQ_HANDLERS=5
        )

# pull in common dependencies
target_link_libraries(main 
        pico_stdlib
        pico_cyw43_arch_none
        hardware_adc
        ultrasonic_lib
        motor_lib
        encoder_lib
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)