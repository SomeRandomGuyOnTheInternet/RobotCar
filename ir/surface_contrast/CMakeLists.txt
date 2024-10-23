add_executable(surface_contrast
    surface_contrast.c  # Replace with your C file's name
)

# Enable USB output, disable UART (for serial monitor via USB)
pico_enable_stdio_usb(surface_contrast 1)
pico_enable_stdio_uart(surface_contrast 1)

# Link with required Pico libraries
target_link_libraries(surface_contrast pico_stdlib hardware_adc hardware_gpio hardware_timer) 

# Create build output
pico_add_extra_outputs(surface_contrast)
