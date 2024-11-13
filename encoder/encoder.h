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
#define ENCODER_NOTCH 20.0
#define ENCODER_RADIUS 1.25
#define WHEEL_CIRCUMFERENCE 20.0
#define WHEEL_TO_WHEEL_DISTANCE 10.0
#define PULSES_PER_REVOLUTION 40  // Number of pulses per wheel revolution
#define PI 3.14159265

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

typedef struct {
    uint32_t pulse_count;
    uint64_t timestamp;
} EncoderData;

typedef void (*encoder_read_callback)(uint gpio, uint32_t events);

extern volatile EncoderData left_data;
extern volatile EncoderData right_data;

// Mutexes for each encoder
extern SemaphoreHandle_t left_data_mutex;
extern SemaphoreHandle_t right_data_mutex;

// Functions for encoders
void encoder_init();
bool encoder_set_distance_speed_callback(struct repeating_timer *t);
void read_encoder_pulse(uint gpio, uint32_t events);

float get_left_distance();
float get_right_distance();

// Speed measurement functions for each encoder
float get_left_speed();
float get_right_speed();

// Encoder reset functions for each encoder
void reset_left_encoder();
void reset_right_encoder();

#endif
