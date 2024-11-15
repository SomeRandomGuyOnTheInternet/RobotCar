#ifndef ultrasonic_h
#define ultrasonic_h

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define TRIGPIN 7
#define ECHOPIN 6

#define MIN_CM 10
#define MAX_CM 400

typedef struct kalman_state_ kalman_state;

kalman_state *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(kalman_state *state, double measurement);

void ultrasonic_init();
void read_echo_pulse();
void set_start_time(uint gpio, uint32_t events);
void set_pulse_length(uint gpio, uint32_t events);
void send_pulse();
uint64_t get_pulse_length();
double get_cm(kalman_state *state);

#endif