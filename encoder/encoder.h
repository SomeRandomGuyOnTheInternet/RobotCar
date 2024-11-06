#ifndef encoder_h
#define encoder_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define encoder pins
#define L_ENCODER_POW 17 // GPIO pin for L encoder power
#define L_ENCODER_OUT 16 // GPIO pin for L encoder output
#define R_ENCODER_POW 14 // GPIO pin for R encoder power
#define R_ENCODER_OUT 15 // GPIO pin for R encoder output

// Define encoder disk specs
#define ENCODER_NOTCH 20.0
#define ENCODER_RADIUS 1.25
#define WHEEL_CIRCUMFERENCE 22.0
#define WHEEL_TO_WHEEL_DISTANCE 11.0
#define PI 3.14159265

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

typedef void (*encoder_read_callback)(uint gpio, uint32_t events);

// External variables
extern volatile uint32_t oscillation;
extern volatile double total_average_distance;
extern volatile float actual_speed_left;
extern volatile float actual_speed_right;
extern volatile float actual_distance_left;
extern volatile float actual_distance_right;


// Functions for encoders
void encoder_init();
bool encoder_set_distance_speed_callback(struct repeating_timer *t);
void set_distance_speed(int encoder, int interval);
void read_encoder_pulse(uint gpio, uint32_t events);
void start_tracking();

#endif
