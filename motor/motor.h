#ifndef motor_h
#define motor_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"


//FLIP LEFT AND RIGHT 
// Define motor pins
#define L_MOTOR_IN1 26 // GPIO pin for what was previously the R motor input 1
#define L_MOTOR_IN2 27 // GPIO pin for what was previously the R motor input 2
#define L_MOTOR_ENA 22 // GPIO pin for what was previously the R motor enable
#define R_MOTOR_IN3 0  // GPIO pin for what was previously the L motor input 1
#define R_MOTOR_IN4 1  // GPIO pin for what was previously the L motor input 2
#define R_MOTOR_ENB 2  // GPIO pin for what was previously the L motor enable
#define PWM_MIN 1600
#define PWM_MAX 3125

// External variables
extern volatile float pwm_left;
extern volatile float pwm_right;

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