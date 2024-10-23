#include "pico/stdlib.h"  // Provides standard functions for GPIO and timing
#include "hardware/adc.h"  // Provides functions for ADC handling
#include <stdio.h>         // Enables printf for debugging
#include <stdbool.h>  // Defines "true", "false", and "bool" in C

#define IR_SENSOR_ADC_PIN 26  // ADC pin to read analog signals from the IR sensor (GPIO26 = ADC0)
#define THRESHOLD 1000        // Define a threshold to differentiate between black and white

volatile uint32_t event_id = 0;  // Event ID for tracking

void init_adc() {
    // Initialize the ADC and select input pin
    adc_init();
    adc_gpio_init(IR_SENSOR_ADC_PIN);  // Initialize GPIO26 for ADC
    adc_select_input(0);  // ADC0 corresponds to GPIO26
}

uint16_t read_adc_value() {
    // Read the ADC value (12-bit resolution, values range from 0 to 4095)
    uint16_t result = adc_read();
    return result;
}

int main() {
    stdio_init_all();
    printf("IR Sensor Surface Contrast Detection using ADC initialized.\n");

    // Initialize ADC for IR sensor readings
    init_adc();

    // Main loop to continuously check the ADC value and determine the surface contrast
    while (true) {
        // Read the analog value from the IR sensor
        uint16_t adc_value = read_adc_value();

        // Print the ADC value for debugging
        printf("[Event ID: %u] ADC Value: %u\n", event_id, adc_value);

        // Detect black or white based on ADC value and the defined threshold
        if (adc_value < THRESHOLD) {
            // Surface is black (less reflection)
            printf("[Event ID: %u] Surface detected: Black\n", event_id);
        } else {
            // Surface is white (more reflection)
            printf("[Event ID: %u] Surface detected: White\n", event_id);
        }

        // Increment the event ID
        event_id++;

        // Small delay to avoid flooding the console
        sleep_ms(500);
    }

    return 0;
}
