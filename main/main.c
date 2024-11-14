// Main program

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "gy511.h"

void callbacks(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    // Left wheel encoder callback
    case L_ENCODER_OUT:
        read_encoder_pulse(L_ENCODER_OUT, events);
        break;
    // Right wheel encoder callback
    case R_ENCODER_OUT:
        read_encoder_pulse(R_ENCODER_OUT, events);
        break;
    // Ultrasonic callback
    case ECHOPIN:
        read_echo_pulse(ECHOPIN, events);
        break;
    default:
        break;
    }
}

// Function to init all sensors and motors
void init_all()
{
    // Initialise standard I/O
    stdio_init_all();
    sleep_ms(1000);

    // Initialise motor pins and PWM
    motor_init();
    printf("Motor pins and PWM initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    ultrasonic_init();
    printf("Ultrasonic pins initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    gy511_init();
    printf("Magnetometer pins initialised\n");
    sleep_ms(500);

    // Initialise encoder sensor
    encoder_init();
    printf("Encoder pins initialised\n");
    sleep_ms(500);
}

// Function to init all interrupts
void init_interrupts()
{
    printf("Interrupts initialised\n");
    // Initialise interrupts for needed sensors
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &callbacks);
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &callbacks);
}

// Helper function to get average distance traveled by both wheels
float get_average_distance() {
    float average_distance = (get_left_distance() + get_right_distance()) / 2.0f;
    // printf("Average Distance: %f", average_distance);
    return average_distance;
}

void station_1_run()
{
    printf("Starting test\n");

    float target_speed = 10.0f;

    kalman_state *state = kalman_init(5.0, 0.5, 0.1, 100.0);
    bool obstacle_detected = false;
    double cm, prev_cm;

    // GO STRAIGHT UNTIL OBSTACLE
    reset_left_encoder();
    reset_right_encoder();
    while (1)
    {
        // Read ultrasonic sensor
        for (int i = 0; i < 20; i++)
        {
            cm = get_cm(state);
        }
        obstacle_detected = cm < MIN_CM;

        if (cm != prev_cm)
        {
            printf("Obstacle distance: %.2lf cm\n", cm);
            printf("----\n");
            prev_cm = cm;
        }

        printf("----\n");
        // Control motor based on obstacle detection
        if (obstacle_detected)
        {
            printf("Obstacle detected\n");
            stop_motor();
            sleep_ms(1000);
            break;
        }
        else
        {
            // move_car(FORWARD, target_speed, 0.0f);
        }
    }

    // // TURN RIGHT
    // turn_motor(RIGHT_WHEEL);
    // sleep_ms(1000);
    // double previous_distance = total_average_distance;

    // MOVE 90CM
    reset_left_encoder();
    reset_right_encoder();
    double start_timestamp = time_us_64() / 1000000.0; // Start time - Converts microseconds to seconds
    move_car(FORWARD, target_speed, 0.0f);  // Set speed (e.g., 20 cm/s)
    while (get_left_distance() < 21.0f || get_right_distance() < 21.0f) { 
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    }
    double end_timestamp = time_us_64() / 1000000.0; // End time - Converts microseconds to seconds
    double time_diff = end_timestamp - start_timestamp;
    float average_speed = 21 / time_diff;
    printf("Average speed: %f \n", average_speed);
    
    // STOP CAR
    move_car(STOP, 0.0f, 0.0f); // Stop after reaching target distance
    printf("Reached 90 cm. Stopping.\n");
    vTaskDelay(pdMS_TO_TICKS(500)); // Small pause
}

int main()
{
    // Init all required
    init_all();
    init_interrupts();

    // Create car movement task
    xTaskCreate(station_1_run, "Robot Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}