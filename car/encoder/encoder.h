#ifndef encoder_h
#define encoder_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Define encoder pins
#define L_ENCODER_POW 17 // GPIO pin for L encoder power
#define L_ENCODER_OUT 16 // GPIO pin for L encoder output
#define R_ENCODER_POW 14 // GPIO pin for R encoder power
#define R_ENCODER_OUT 15 // GPIO pin for R encoder output

// Define encoder disk specs
#define ENCODER_NOTCHES 20.0
#define WHEEL_CIRCUMFERENCE 20.0
#define WHEEL_TO_WHEEL_DISTANCE 10.8
#define PULSES_PER_REVOLUTION 20.0  // Number of pulses per wheel revolution

#define INVALID_SPEED -1.0

#define NEUTRAL 0
#define FORWARDS 1
#define BACKWARDS -1
#define RIGHT 2
#define LEFT 3

typedef struct {
    volatile uint32_t pulse_count;
    volatile uint64_t timestamp;
} EncoderData;

typedef struct {
    volatile EncoderData data;
    volatile EncoderData last_data;
    SemaphoreHandle_t mutex;
} Encoder;

// Functions for encoders
void encoder_init();
void read_encoder_pulse(uint gpio, uint32_t events);

float get_left_distance();
float get_right_distance();
float get_average_distance();

// Speed measurement functions for each encoder
float get_left_speed();
float get_right_speed();
float get_average_speed();

// Encoder reset functions for each encoder
void reset_left_encoder();
void reset_right_encoder();
void reset_encoders();

#endif
