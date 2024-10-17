#include "pico/stdlib.h" // Provides standard functions like gpio_init() and gpio_get() to work with the Raspberry Pi Pico's hardware, including GPIOs.
#include "hardware/gpio.h" // Contains GPIO-specific functions for input/output handling.
#include "hardware/timer.h" // Provides functions for managing hardware timers like add_repeating_timer_ms().
#include <stdio.h> // Enables functions like printf() to print messages to the console.

#define IR_SENSOR_PIN 15  // IR sensor GPIO pin for object detection
#define DEBOUNCE_DELAY_MS 100  // Debounce delay in milliseconds

volatile uint32_t black_stopwatch_counter = 0; // Stopwatch counter for "black" state (low signal)
volatile uint32_t white_stopwatch_counter = 0; // Stopwatch counter for "white" state (high signal)
volatile bool black_timer_active = false;      // Track if "black" timer is running
volatile bool white_timer_active = false;      // Track if "white" timer is running
volatile bool object_detected = false;         // Track the current state of the sensor (black or white)
volatile uint32_t event_id = 0;                // Event ID to track each event
struct repeating_timer black_stopwatch_timer;  // Timer for "black" state
struct repeating_timer white_stopwatch_timer;  // Timer for "white" state
volatile uint32_t last_event_time = 0;         // To store the time of the last event

// "Black" state (low signal) timer callback
bool black_stopwatch_timer_callback(struct repeating_timer *t) {
    black_stopwatch_counter++; // Increment the black stopwatch counter each time callback is executed
    printf("[Event ID: %u] Black Stopwatch: %d seconds\n", event_id, black_stopwatch_counter); // Debug message
    return true; // To keep the timer running
}

// "White" state (high signal) timer callback
bool white_stopwatch_timer_callback(struct repeating_timer *t) {
    white_stopwatch_counter++; // Increment the white stopwatch counter each time callback is executed
    printf("[Event ID: %u] White Stopwatch: %d seconds\n", event_id, white_stopwatch_counter); // Debug message
    return true; // To keep the timer running
}

// GPIO interrupt handler for IR sensor (falling and rising edge)
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time()); // Get current time in ms
    printf("[Event ID: %u] GPIO callback triggered. GPIO: %d, Events: %d, Current Time: %d ms\n", event_id, gpio, events, current_time);

    last_event_time = current_time; // Update the last event time

    // Check current state of GPIO to distinguish between rising and falling edges
    bool current_state = gpio_get(IR_SENSOR_PIN);

    if (!current_state && !object_detected) {  // Falling edge: Object detected ("black" state begins)
        object_detected = true;
        printf("[Event ID: %u] Object detected (Falling edge - Black state begins).\n", event_id);

        // Stop the white stopwatch timer (high state)
        if (white_timer_active) {
            printf("[Event ID: %u] Stopping White Stopwatch.\n", event_id);
            cancel_repeating_timer(&white_stopwatch_timer); // Stop white stopwatch
            white_timer_active = false;
        }

        // Start the black stopwatch timer (low state)
        if (!black_timer_active) {
            event_id++; // Start a new event cycle
            printf("[Event ID: %u] Starting Black Stopwatch.\n", event_id);
            black_stopwatch_counter = 0;  // Reset the black stopwatch counter
            add_repeating_timer_ms(1000, black_stopwatch_timer_callback, NULL, &black_stopwatch_timer); // Start the black timer (1-second interval)
            black_timer_active = true;
        }
    } else if (current_state && object_detected) {  // Rising edge: Object no longer detected ("white" state begins)
        object_detected = false;
        printf("[Event ID: %u] Object no longer detected (Rising edge - White state begins).\n", event_id);

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
    printf("IR Sensor-based Stopwatch initialized.\n");

    // Initialize the IR sensor GPIO
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);       // Set as input
    gpio_pull_up(IR_SENSOR_PIN);                // Enable pull-up resistor 

    // Debug message for GPIO initialization
    printf("GPIO %d initialized as input with pull-up.\n", IR_SENSOR_PIN);

    // Enable GPIO interrupt for both falling (object detected) and rising (object no longer detected) edges
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Main loop does nothing, just waiting for interrupts
    while (true) {
        tight_loop_contents();  // Low-power sleep while waiting for interrupts
    }

    return 0;
}
