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
    
    while (1)
    {
        sleep_ms(500);
        for (int i = 0; i < 20; i++)
        {
            cm = get_cm(state);
            obstacle_detected = cm < 25;
        }

        if (obstacle_detected == true)
        {
            printf("Obstacle detected\n");
            stop_motor();
        } else {
            move_motor(3125, 3125);
        }

        printf("Distance: %.2lf\n", cm);
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