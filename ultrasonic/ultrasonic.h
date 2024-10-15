#ifndef ultrasonic_h
#define ultrasonic_h

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define TRIGPIN 16
#define ECHOPIN 17

typedef struct kalman_state_ kalman_state;
extern volatile bool obstacle_detected;

kalman_state *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(kalman_state *state, double measurement);

void setup_ultrasonic_pins();
void get_echo_pulse(uint gpio, uint32_t events);
uint64_t get_pulse();
double get_cm(kalman_state *state);

#endif