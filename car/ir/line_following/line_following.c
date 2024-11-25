#include "line_following.h"
#include "../../motor/motor.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define LINE_THRESHOLD 280       // Define a custom threshold for line detection
#define DEBOUNCE_DELAY_MS 100     // 100 ms debounce delay
#define TURN_SPEED 2000           // Slower speed for turning

// Function to initialize the line sensor using ADC
void init_line_sensor() {
    adc_init();                     // Initialize ADC
    adc_gpio_init(LINE_SENSOR_PIN); // Initialize the chosen ADC pin 
    adc_select_input(2);            // Select ADC input 2 for GPIO28
}

// Function to perform a slower timed turn
void line_follow_turn_motor(int direction) {
    // Set motor direction for turning
    if (direction == 0) { // Turn left slowly
        gpio_put(L_MOTOR_IN1, 0);
        gpio_put(L_MOTOR_IN2, 1);
        gpio_put(R_MOTOR_IN3, 1);
        gpio_put(R_MOTOR_IN4, 0);
    } else { // Turn right slowly
        gpio_put(L_MOTOR_IN1, 1);
        gpio_put(L_MOTOR_IN2, 0);
        gpio_put(R_MOTOR_IN3, 0);
        gpio_put(R_MOTOR_IN4, 1);
    }

    // Set motors to lower speed for slower turn
    pwm_set_gpio_level(L_MOTOR_ENA, TURN_SPEED);
    pwm_set_gpio_level(R_MOTOR_ENB, TURN_SPEED);

    uint64_t start_time = time_us_64();
    uint64_t max_turn_duration = STEER_DURATION * 2 * 1000; // Convert to microseconds

    // Continue turning while monitoring the line sensor
    while ((time_us_64() - start_time) < max_turn_duration) {
        uint16_t sensor_value = adc_read(); // Read ADC value

        // If the line is detected, exit the turn early
        if (sensor_value > LINE_THRESHOLD) {
            printf("Line detected during turn. Exiting turn early. Sensor value: %d\n", sensor_value);
            stop_motor();  // Stop motors immediately
            return;        // Return to main to resume line-following
        }

        sleep_ms(10); // Small delay to avoid overwhelming the ADC with too many reads
    }

    // Stop the motors after completing the turn if the line was not detected
    stop_motor();
    sleep_ms(50); // Small delay to stabilize
}

int start_line_following() {
    stdio_init_all(); // Initialize standard I/O for printing

    init_line_sensor(); // Initialize the line sensor with ADC

    motor_init();
    motor_pwm_init();
    printf("Motor pins and PWM initialized\n");
    sleep_ms(500);

    uint64_t last_transition_time = 0;
    int turn_direction = 0;

    while (true) {
        uint64_t current_time = time_us_64();
        uint16_t sensor_value = adc_read();

        if ((current_time - last_transition_time) > (DEBOUNCE_DELAY_MS * 1000)) {
            last_transition_time = current_time;

            if (sensor_value > LINE_THRESHOLD) {
                forward_motor(2500, 2500);
                printf("Line detected. Sensor value: %d\n", sensor_value);
            } else {
                printf("Out of course. Initiating slow turn. Sensor value: %d\n", sensor_value);
                stop_motor();
                reverse_motor(2200, 2200);
                sleep_ms(100);

                line_follow_turn_motor(turn_direction);
                turn_direction = 1 - turn_direction;
            }
        }

        sleep_ms(50);
    }

    return 0;
}
