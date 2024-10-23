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

void callbacks(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    // Ultrasonic callback
    case ECHOPIN:
        ultrasonic_interrupt_callback(ECHOPIN, events);
        break;
    case L_ENCODER_OUT:
        encoder_pulse_callback(L_ENCODER_OUT, events);
        break;
    case R_ENCODER_OUT:
        encoder_pulse_callback(R_ENCODER_OUT, events);
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
}

void test()
{
    // Set up a timer to generate interrupts every second
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, encoder_1s_callback, NULL, &timer);

    while (1) {
        // Run at half duty cycle
        move_motor(1563, 1563);
        sleep_ms(5000);

        // Turn left at full duty cycle
        move_motor(3165, 3165);
        turn_motor(1);
        sleep_ms(250);

        // Turn right at full duty cycle
        move_motor(3165, 3165);
        turn_motor(0);
        sleep_ms(250);

        // Run at 32% duty cycle
        // moveMotor(1000);
        move_motor(1000, 1000);
        sleep_ms(5000);
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