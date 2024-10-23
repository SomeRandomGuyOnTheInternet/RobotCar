add_executable(barcodeADC
    barcodeADC.c  # Replace with your C file's name
)

# Enable USB output, disable UART (for serial monitor via USB)
pico_enable_stdio_usb(barcodeADC 1)
pico_enable_stdio_uart(barcodeADC 1)

# Link with required Pico libraries
target_link_libraries(barcodeADC pico_stdlib hardware_adc hardware_gpio hardware_timer) 

# Create build output
pico_add_extra_outputs(barcodeADC)
