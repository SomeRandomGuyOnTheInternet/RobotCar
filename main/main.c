// Main program

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h"
#include "gy511.h"

// Function that is invoked upon a change in right IR sensor's input
void callbacks(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    // Ultrasonic callback
    case ECHOPIN:
        get_echo_pulse(ECHOPIN, events);
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
    init_motor_setup();
    init_motor_pwm();
    printf("Motor pins and PWM initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    setup_ultrasonic_pins();
    printf("Ultrasonic pins initialised\n");
    sleep_ms(500);

    gy511_init();
    printf("Magnetometer pins initialised\n");
    sleep_ms(500);

}

// Function to init all interrupts
void init_interrupts()
{
    printf("Interrupts initialised\n");
    // Initialise interrupts for needed sensors
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &callbacks);
}

void test()
{
    printf("Starting test\n");
    bool obstacle_detected = false;
    double cm;
    kalman_state *state = kalman_init(1, 100, 0, 0);
    int16_t accel_x, accel_y, accel_z;

    while (1)
    {
        sleep_ms(100); // Reduced sleep for more responsive readings

        // Read ultrasonic sensor
        cm = get_cm(state);
        cm = 9999;
        obstacle_detected = cm < 25;

        // Control motor based on obstacle detection
        if (obstacle_detected)
        {
            printf("Obstacle detected\n");
            stop_motor();
        }
        else 
        {
            // Read accelerometer data
            gy511_read_accel(&accel_x, &accel_y, &accel_z);
            printf("Accel X: %d, Y: %d, Z: %d\n", accel_x, accel_y, accel_z);

            // Adjust speed based on Y-axis tilt
            if (accel_y > 5000) // Forward tilt
            {
                move_motor(2500, 2500);
            }
            else if (accel_y < -5000) // Backward tilt
            {
                reverse_motor(2500, 2500);
            }

            // Adjust turning based on X-axis tilt
            if (accel_x > 5000) // Right tilt
            {
                move_motor(2500, -2500);
            }
            else if (accel_x < -5000) // Left tilt
            {
                move_motor(-2500, 2500);
            }
        }

        // printf("Distance: %.2lf\n", cm);
        sleep_ms(500); // Delay between readings
    }
}


int main()
{
    // Init all required
    init_all();
    init_interrupts();
    
    test();

    return 0;
}