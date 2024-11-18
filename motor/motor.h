#ifndef motor_h
#define motor_h

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "encoder.h"

// Define motor pins
#define L_MOTOR_IN1 0 // GPIO pin for L motor input 1
#define L_MOTOR_IN2 1 // GPIO pin for L motor input 2
#define L_MOTOR_ENA 2 // GPIO pin for L motor enable

#define R_MOTOR_IN3 26 // GPIO pin for R motor input 1
#define R_MOTOR_IN4 27 // GPIO pin for R motor input 2
#define R_MOTOR_ENB 22 // GPIO pin for R motor enable

#define PWM_MIN 1600
#define PWM_MAX 3125

// Function prototypes
void motor_init();
void motor_pwm_init();
void reverse_motor(float new_pwm_left, float new_pwm_right);
void stop_motor();
void turn_motor(int direction, float angle);
void move_motor_pid(float new_target_speed);
void move_motor_constant(float new_pwm_left, float new_pwm_right);

// PID control functions
float compute_pid_pwm(float target_speed, float current_value, float *integral, float *prev_error);
void pid_task(void *params);

#endif // MOTOR_H


