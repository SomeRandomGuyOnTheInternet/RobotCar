add_executable(barcode
    barcode.c  # Replace with your C file's name
)

# Enable USB output, disable UART (for serial monitor via USB)
pico_enable_stdio_usb(barcode 1)
pico_enable_stdio_uart(barcode 1)

# Link with required Pico libraries
target_link_libraries(barcode pico_stdlib hardware_adc hardware_gpio hardware_timer) 

# Create build output
pico_add_extra_outputs(barcode)
