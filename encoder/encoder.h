#ifndef encoder_h
#define encoder_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define encoder pins
#define L_ENCODER_POW 10 // GPIO pin for L encoder power
#define L_ENCODER_OUT 11 // GPIO pin for L encoder output
#define R_ENCODER_POW 17 // GPIO pin for R encoder power
#define R_ENCODER_OUT 16 // GPIO pin for R encoder output

// Define encoder disk specs
#define ENCODER_NOTCH 20.0
#define ENCODER_CIRCUMFERENCE 8.5
#define WHEEL_CIRCUMFERENCE 22.0

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

// External variables
extern volatile uint32_t oscillation;
extern volatile float actual_speed_left;
extern volatile float actual_speed_right;

// Functions for encoders
void encoder_init();
bool encoder_1s_callback();
void encoder_pulse_callback(uint gpio, uint32_t events);
void read_encoder_pulse(uint gpio, uint32_t events);
void start_tracking();
void get_speed_and_distance(int encoder, uint32_t pulse_count);

#endif
