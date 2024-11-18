#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Define ultrasonic sensor pins
#define TRIGPIN 5
#define ECHOPIN 4

// Kalman filter state structure
typedef struct kalman_state_ {
    double q; // process noise covariance
    double r; // measurement noise covariance
    double x; // estimated value
    double p; // estimation error covariance
    double k; // kalman gain
} kalman_state;

// Function prototypes
kalman_state *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(kalman_state *state, double measurement);

void ultrasonic_init();
void send_echo_pulse();
void read_echo_pulse(uint gpio, uint32_t events);
double get_obstacle_distance();

#endif
