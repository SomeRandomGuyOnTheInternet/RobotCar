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

// External variables declared in motor.c
extern float setpoint_speed;
extern volatile float actual_speed_left;
extern volatile float actual_speed_right;
extern volatile float pwmL;
extern volatile float pwmR;

void callbacks(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    // Left wheel encoder callback
    case L_ENCODER_OUT:
        encoder_pulse_callback(L_ENCODER_OUT, events);
        break;
    // Right wheel encoder callback
    case R_ENCODER_OUT:
        encoder_pulse_callback(R_ENCODER_OUT, events);
        break;
    // Ultrasonic callback
    case ECHOPIN:
        ultrasonic_interrupt_callback(ECHOPIN, events);
        break;
    default:
        break;
    }
}

// Function to init all sensors and motors
void init_all()
{
    stdio_init_all();
    sleep_ms(1000);

    // Initialize motor and other components
    initMotorSetup();
    initMotorPWM();
    printf("Motor pins and PWM initialised\n");

    ultrasonic_init();
    printf("Ultrasonic pins initialised\n");

    gy511_init();
    printf("Magnetometer pins initialised\n");

    encoder_init();
    printf("Encoder pins initialised\n");
}

void init_interrupts()
{
    printf("Interrupts initialised\n");
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &callbacks);
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &callbacks);
}

void test_straight_movement()
{
    printf("Starting straight movement test with encoder feedback.\n");

    struct repeating_timer timer;
    add_repeating_timer_ms(1000, encoder_1s_callback, NULL, &timer); // 1-second callback for speed updates

    while (1) {
        update_motor_speed(); // Adjust motor speed based on encoder feedback
        moveMotor(pwmL, pwmR); // Apply adjusted PWM values

        // Monitor and print actual speeds and PWM values
        printf("Target Speed: %.2f | Left Speed: %.2f, Right Speed: %.2f | PWM Left: %.2f, PWM Right: %.2f\n",
               setpoint_speed, actual_speed_left, actual_speed_right, pwmL, pwmR);

        sleep_ms(100); // Delay for periodic adjustment
    }
}

int main() {
    init_all();
    init_interrupts();

    // Run the straight movement test
    test_straight_movement();

    return 0;
}
