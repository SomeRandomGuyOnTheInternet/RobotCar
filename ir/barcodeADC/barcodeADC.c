#include "pico/stdlib.h"    // Provides standard functions like gpio_init() and gpio_get() to work with the Raspberry Pi Pico's hardware, including GPIOs.
#include "hardware/adc.h"   // Includes functions for handling the ADC on the Pico.
#include "hardware/timer.h" // Provides functions for managing hardware timers like add_repeating_timer_ms().
#include <stdio.h>          // Enables functions like printf() to print messages to the console.

#define IR_SENSOR_PIN 15       // IR sensor GPIO pin (not used as ADC is now used)
#define IR_SENSOR_ADC_PIN 0    // Analog input pin for the IR sensor (ADC0)
#define ADC_CHANNEL 0          // ADC channel corresponding to ADC0 (pin 26)
#define DEBOUNCE_DELAY_MS 100  // Debounce delay in milliseconds
#define THRESHOLD 1000         // Threshold value to differentiate "black" and "white" states (adjust as needed)

volatile uint32_t black_stopwatch_counter = 0; // Stopwatch counter for "black" state (object detected)
volatile uint32_t white_stopwatch_counter = 0; // Stopwatch counter for "white" state (no object detected)
volatile bool black_timer_active = false;      // Track if "black" timer is running
volatile bool white_timer_active = false;      // Track if "white" timer is running
volatile bool object_detected = false;         // Track the current state of the sensor (black or white)
volatile uint32_t event_id = 0;                // Event ID to track each event
struct repeating_timer black_stopwatch_timer;  // Timer for "black" state
struct repeating_timer white_stopwatch_timer;  // Timer for "white" state

// "Black" state (low signal) timer callback
bool black_stopwatch_timer_callback(struct repeating_timer *t) {
    black_stopwatch_counter++;
    printf("[Event ID: %u] Black Stopwatch: %d seconds\n", event_id, black_stopwatch_counter);
    return true;
}

// "White" state (high signal) timer callback
bool white_stopwatch_timer_callback(struct repeating_timer *t) {
    white_stopwatch_counter++;
    printf("[Event ID: %u] White Stopwatch: %d seconds\n", event_id, white_stopwatch_counter);
    return true;
}

void check_ir_sensor() {
    uint16_t adc_value = adc_read();  // Read the current ADC value from the IR sensor.

    // Check if the ADC value crosses the threshold to detect an object.
    if (adc_value <= THRESHOLD && !object_detected) {  // Object detected ("black" state)
        object_detected = true;
        printf("[Event ID: %u] Object detected (Black state begins).\n", event_id);

        // Stop the white stopwatch timer (high state)
        if (white_timer_active) {
            printf("[Event ID: %u] Stopping White Stopwatch.\n", event_id);
            cancel_repeating_timer(&white_stopwatch_timer); // Stop white stopwatch
            white_timer_active = false;
        }

        // Start the black stopwatch timer (low state)
        if (!black_timer_active) {
            event_id++;
            printf("[Event ID: %u] Starting Black Stopwatch.\n", event_id);
            black_stopwatch_counter = 0;  // Reset the black stopwatch counter
            add_repeating_timer_ms(1000, black_stopwatch_timer_callback, NULL, &black_stopwatch_timer); // Start the black timer (1-second interval)
            black_timer_active = true;
        }
    } else if (adc_value > THRESHOLD && object_detected) {  // Object no longer detected ("white" state)
        object_detected = false;
        printf("[Event ID: %u] Object no longer detected (White state begins).\n", event_id);

        // Stop the black stopwatch timer (low state)
        if (black_timer_active) {
            printf("[Event ID: %u] Stopping Black Stopwatch.\n", event_id);
            cancel_repeating_timer(&black_stopwatch_timer); // Stop black stopwatch
            black_timer_active = false;
        }

        // Start the white stopwatch timer (high state)
        if (!white_timer_active) {
            printf("[Event ID: %u] Starting White Stopwatch.\n", event_id);
            white_stopwatch_counter = 0;  // Reset the white stopwatch counter
            add_repeating_timer_ms(1000, white_stopwatch_timer_callback, NULL, &white_stopwatch_timer); // Start the white timer (1-second interval)
            white_timer_active = true;
        }
    }
}

int main() {
    stdio_init_all();
    printf("IR Sensor-based Stopwatch using Analog Input initialized.\n");

    // Initialize the ADC
    adc_init();
    adc_gpio_init(26 + IR_SENSOR_ADC_PIN);  // Initialize GPIO pin 26 as ADC input (corresponds to ADC0)
    adc_select_input(ADC_CHANNEL);         // Select ADC channel 0 for reading

    // Main loop to periodically check the IR sensor reading
    while (true) {
        check_ir_sensor();
        sleep_ms(DEBOUNCE_DELAY_MS);  // Add a debounce delay to avoid rapid fluctuations.
    }

    return 0;
}
