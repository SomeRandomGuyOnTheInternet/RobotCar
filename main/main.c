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
    // Initialise standard I/O
    stdio_init_all();
    sleep_ms(1000);

    // Initialise motor pins and PWM
    motor_init();
    motor_pwm_init();
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

double normalise(double value, double min, double max) {
   // Ensure value is within bounds
    if (value < min) value = min;
    if (value > max) value = max;

    return (value - min) / (max - min);
}

void test_straight_movement()
{
    printf("Starting straight movement test with encoder feedback.\n");

    struct repeating_timer timer;
    add_repeating_timer_ms(1000, encoder_1s_callback, NULL, &timer); // 1-second callback for speed updates

    while (1) {
        move_motor(pwmL, pwmR); // Apply adjusted PWM values
        update_motor_speed(); // Adjust motor speed based on encoder feedback

        // Monitor and print actual speeds and PWM values
        printf("Target Speed: %.2f | Left Speed: %.2f, Right Speed: %.2f | PWM Left: %.2f, PWM Right: %.2f\n",
               setpoint_speed, actual_speed_left, actual_speed_right, pwmL, pwmR);

        sleep_ms(1000); // Delay for periodic adjustment
    }
}

void test()
{
    printf("Starting test\n");

    struct repeating_timer timer;
    add_repeating_timer_ms(1000, encoder_1s_callback, NULL, &timer);

    kalman_state *state = kalman_init(1.0, 0.5, 0.1, 100.0);

    bool obstacle_detected = false;
    double cm;
    double prev_cm;

    while (1)
    {
        sleep_ms(250); // Reduced sleep for more responsive readings

        // Read ultrasonic sensor
        cm = get_cm(state);
        // cm = 99;
        obstacle_detected = cm < MIN_CM;

        printf("----\n");
        // Control motor based on obstacle detection
        if (obstacle_detected)
        {
            printf("Obstacle detected\n");
            stop_motor();
        }
        else
        {
            // double normalised = normalise(cm, MIN_CM, MAX_CM);
            // int normalised_duty_cycle = (int)(PWM_MIN + ((PWM_MAX - PWM_MIN) * normalised));
            move_motor(PWM_MIN, PWM_MIN);
        }

        if (cm != prev_cm) {
            printf("Obstacle distance: %.2lf cm\n", cm);
            printf("----\n");
            prev_cm = cm;
        }
    }
}

int main()
{
    // Init all required
    init_all();
    init_interrupts();

    test_straight_movement();

    return 0;
}