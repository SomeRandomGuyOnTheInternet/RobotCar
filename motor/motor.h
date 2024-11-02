#ifndef motor_h
#define motor_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"


//FLIP LEFT AND RIGHT 
// Define motor pins
#define L_MOTOR_IN1 0 // GPIO pin for L motor input 1
#define L_MOTOR_IN2 1 // GPIO pin for L motor input 2
#define L_MOTOR_ENA 2 // GPIO pin for L motor enable
#define R_MOTOR_IN3 26 // GPIO pin for R motor input 1
#define R_MOTOR_IN4 27 // GPIO pin for R motor input 2
#define R_MOTOR_ENB 22 // GPIO pin for R motor enable
#define PWM_MIN 1600
#define PWM_MAX 3125

// External variables
extern volatile float setpoint_speed;
extern volatile float pwmL;
extern volatile float pwmR;

// Functions for motors
void motor_init();
void motor_pwm_init();
void move_motor(float pwm_left, float pwm_right);
void reverse_motor(float pwm_left, float pwm_right);
void stop_motor();
void turn_motor(int direction);
void update_motor_speed();
void move_grids(int number_of_grids);
void reverse_grids(int number_of_grids);

#endif