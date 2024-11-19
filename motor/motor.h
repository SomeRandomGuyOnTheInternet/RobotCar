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

#define PWM_KICKSTART 2000
#define PWM_MIN_LEFT 1800
#define PWM_MIN_RIGHT 1600
#define PWM_MAX 3500

#define MIN_SPEED 30.0f
#define MAX_SPEED 40.0f

#define PI 3.14159265358979323846
#define FULL_CIRCLE 360.0f

#define CONTINUOUS -1.0f

typedef enum {
    STOP,
    FORWARD,
    REVERSE,
    LEFT_TURN,
    RIGHT_TURN,
    DISABLED
} PIDState;

// Function prototypes
void motor_init();
void motor_pwm_init();

// Movement functions
void move_motor(float new_pwm_left, float new_pwm_right);
void reverse_motor(float new_pwm_left, float new_pwm_right);
void turn_motor(int direction, float angle, float new_pwm_left, float new_pwm_right);
void stop_motor();

// PID control functions
void enable_pid_control();
void disable_pid_control();
void move_motor_pid(float new_target_speed);
void reverse_motor_pid(float new_target_speed);
void turn_motor_pid(int direction, float new_target_speed);
void stop_motor_pid();
float compute_pid_pwm(float target_speed, float current_value, float *integral, float *prev_error);
void pid_task(void *params);

#endif // MOTOR_H


