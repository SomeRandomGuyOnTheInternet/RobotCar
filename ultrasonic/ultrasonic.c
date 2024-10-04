// Get readings from ultrasonic sensor

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "ultrasonic.h"

volatile absolute_time_t start_time;
volatile uint64_t pulse_width = 0;
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

void get_echo_pulse(uint gpio, uint32_t events)
{
    if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_RISE)
    {
        // Rising edge detected, start the timer
        start_time = get_absolute_time();
    }
    else if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_FALL)
    {
        // Falling edge detected, calculate the pulse width
        pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
    }
}

void setup_ultrasonic_pins()
{
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &get_echo_pulse);
}

uint64_t get_pulse()
{
    gpio_put(TRIGPIN, 1);
    sleep_us(10);
    gpio_put(TRIGPIN, 0);
    sleep_ms(1);

    return pulse_width;
}

double get_cm(kalman_state *state)
{
    uint64_t pulse_length = get_pulse(TRIGPIN, ECHOPIN);
    double measured = pulse_length / 29.0 / 2.0;
    kalman_update(state, measured);

    if (state->x < 10)
    {
        obstacle_detected = true;
    }

    return state->x;
}

/*
int main()
{
    // Driver code to run ultrasonic sensor
    double cm;
    stdio_init_all();
    printf("Setting up ultrasonic pins\n");
    setupUltrasonicPins();
    kalman_state *state = kalman_init(1, 100, 0, 0);
    sleep_ms(1000);
    while (true)
    {
        for (int i = 0; i < 20; i++)
        {
            cm = getCm(state);
        }
        printf("Distance: %.2lf\n", cm);
        sleep_ms(500);
    }
}
*/