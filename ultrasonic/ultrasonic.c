// Get readings from ultrasonic sensor

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "ultrasonic.h"
// #include "motor.h"

volatile absolute_time_t start_time;
volatile uint64_t pulse_length = 0;
volatile bool obstacle_detected = false;

typedef struct kalman_state_
{
    double q; // process noise covariance
    double r; // measurement noise covariance
    double x; // estimated value
    double p; // estimation error covariance
    double k; // kalman gain
} kalman_state;

kalman_state *kalman_init(double q, double r, double p, double initial_value)
{
    kalman_state *state = calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;

    return state;
}

void kalman_update(kalman_state *state, double measurement)
{
    // Prediction update
    state->p = state->p + state->q;

    // Measurement update
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;
}

void ultrasonic_init()
{
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_set_dir(ECHOPIN, GPIO_IN);
}

void ultrasonic_interrupt_callback(uint gpio, uint32_t events)
{
    printf("Ultrasonic interrupt recieved\n");
    if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_RISE)
    {
        set_start_time(gpio, events);
    }
    else if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_FALL)
    {
        set_pulse_length(gpio, events);
    }
}

void set_start_time(uint gpio, uint32_t events)
{
    if (gpio == ECHOPIN)
    {
        // Rising edge detected, start the timer
        start_time = get_absolute_time();
    }
}

void set_pulse_length(uint gpio, uint32_t events)
{
    if (gpio == ECHOPIN)
    {
        // Falling edge detected, calculate the pulse width
        pulse_length = absolute_time_diff_us(start_time, get_absolute_time());
    }
}

void send_pulse()
{
    gpio_put(TRIGPIN, 1);
    sleep_us(10);
    gpio_put(TRIGPIN, 0);
    sleep_ms(1);
}

uint64_t get_pulse_length()
{
    return pulse_length;
}

double get_cm(kalman_state *state)
{
    send_pulse();
    uint64_t sent_pulse_length = get_pulse_length();
    double measured = sent_pulse_length / 29.0 / 2.0;
    kalman_update(state, measured);

    if (state->x < 10)
    {
        obstacle_detected = true;
    }

    return state->x;
}

// int main()
// {
//     // Initialise standard I/O
//     stdio_init_all();
//     sleep_ms(1000);

//     // Initialise motor GPIO pins and PWM
//     motor_init();
//     motor_pwm_init();

//     // Initialise ultrasonic sensor
//     ultrasonic_init();
//     printf("Ultrasonic pins initialised\n");
//     sleep_ms(500);

//     // Driver code to run ultrasonic sensor
//     printf("Starting test\n");
//     kalman_state *state = kalman_init(1, 100, 0, 0);
//     bool obstacle_detected = false;
//     double cm;

//     while (1)
//     {
//         printf("ECHOPIN: %i\n", ECHOPIN);
//         printf("TRIGPIN: %i\n", TRIGPIN);
//         sleep_ms(500); // Reduced sleep for more responsive readings

//         // Read ultrasonic sensor
//         cm = get_cm(state);
//         obstacle_detected = cm < 25;

//         // Control motor based on obstacle detection
//         if (obstacle_detected)
//         {
//             printf("Obstacle detected\n");
//             stop_motor();
//         }
//         else 
//         {
//             move_motor(2500, 2500);
//         }

//         printf("Distance: %.2lf\n", cm);
//     }
// }