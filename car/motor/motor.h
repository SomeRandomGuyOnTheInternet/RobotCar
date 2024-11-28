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
#define L_MOTOR_ENA 3 // GPIO pin for L motor enable

#define R_MOTOR_IN3 26 // GPIO pin for R motor input 1
#define R_MOTOR_IN4 27 // GPIO pin for R motor input 2
#define R_MOTOR_ENB 22 // GPIO pin for R motor enable

#define Kp 0.80f
#define Ki 0.00f
#define Kd 0.00f

#define PWM_MIN 2100
#define PWM_MIN_LEFT 2100
#define PWM_MIN_RIGHT 1725
#define PWM_MID 2800
#define PWM_MID_LEFT 2800
#define PWM_MID_RIGHT 2700
#define PWM_MAX 3500
#define PWM_MAX_LEFT 3500
#define PWM_MAX_RIGHT 3400
#define PWM_JUMPSTART 2100
#define PWM_TURN 2100

#define MIN_SPEED 30.0f
#define MAX_SPEED 40.0f
#define TURN_SPEED 35.0f
#define JUMPSTART_SPEED_THRESHOLD 5.0f

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
void forward_motor(float new_pwm_left, float new_pwm_right);
void reverse_motor(float new_pwm_left, float new_pwm_right);
void turn_motor(int direction, float angle, float new_pwm_left, float new_pwm_right);
void stop_motor();

// Manual functions
void disable_pid_control();
void forward_motor_manual(float new_pwm_left, float new_pwm_right);
void reverse_motor_manual(float new_pwm_left, float new_pwm_right);
void turn_motor_manual(int direction, float angle, float new_pwm_left, float new_pwm_right);
void stop_motor_manual();
void offset_move_motor(int direction, int turn, float offset);

// PID control functions
void enable_pid_control();
void forward_motor_pid(float new_target_speed);
void reverse_motor_pid(float new_target_speed);
void turn_motor_pid(int direction, float new_target_speed);
void stop_motor_pid();
float compute_pid_pwm(float target_speed, float current_value, float *integral, float *prev_error);
void pid_task(void *params);

void motor_conditioning();

#endif // MOTOR_H
