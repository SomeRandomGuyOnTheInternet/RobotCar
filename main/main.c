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

    // Initialise magnetometer
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

void station_1_run()
{
    printf("Starting test\n");

    // GO STRAIGHT UNTIL OBSTACLE
    float obstacle_distance = 21.0f;
    reset_encoders();
    disable_pid_control();
    sleep_ms(500);
    printf("==========\n");
    printf("Started ultrasonic obstacle detection.\n");
    while (get_obstacle_distance() > obstacle_distance)
    {
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    }
    stop_motor(); // Stop after reaching target distance
    printf("Obstacle detected at %f cm. Stopping.\n", obstacle_distance);

    // TURN AT TARGET ANGLE
    float target_angle = 90.0f;
    int turn_direction = RIGHT;
    reset_encoders();
    sleep_ms(500);
    printf("==========\n");
    printf("Turning %s %f degrees.\n", (turn_direction == LEFT) ? "left" : "right", target_angle);
    turn_motor(turn_direction, target_angle, PWM_MAX, PWM_MAX);
    printf("Turned %s %f degrees. Stopping.\n", (turn_direction == LEFT) ? "left" : "right", target_angle);

    // GO STRAIGHT AT SPEED UNTIL TARGET DISTANCE
    float target_speed = 10.0f;
    float target_distance = 200.0f;
    reset_encoders();
    enable_pid_control();
    sleep_ms(500);
    printf("==========\n");
    printf("Doing PID move motor at %f cm/s until %f cm.\n", target_speed, target_distance);
    move_motor_pid(target_speed);
    while (get_average_distance() < target_distance)
    {
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    }
    stop_motor_pid(); // Stop after reaching target distance
    printf("Reached %f cm. Stopping.\n", target_distance);

    // TURN CONTINUOUSLY IN ONE DIRECTION
    turn_direction = LEFT;
    reset_encoders();
    disable_pid_control();
    sleep_ms(500);
    printf("==========\n");
    printf("Turning %s continuously.\n", (turn_direction == LEFT) ? "left" : "right");
    turn_motor(turn_direction, CONTINUOUS, PWM_MAX, PWM_MAX);
    sleep_ms(2000);
    stop_motor();
    printf("Stopped turning left.\n");
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
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}



















// double start_timestamp = time_us_64() / 1000000.0; // Start time - Converts microseconds to seconds
// move_car(FORWARD, target_speed, 0.0f);             // Set speed (e.g., 20 cm/s)
// while (get_average_distance() < target_distance)
// {
//     vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
// }
// double end_timestamp = time_us_64() / 1000000.0; // End time - Converts microseconds to seconds
// float average_speed = target_distance / (end_timestamp - start_timestamp);
// printf("Average speed: %f \n", average_speed);