#ifndef motor_h
#define motor_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Define motor pins
#define L_MOTOR_IN1 0 // GPIO pin for L motor input 1
#define L_MOTOR_IN2 1 // GPIO pin for L motor input 2
#define L_MOTOR_ENA 2 // GPIO pin for L motor enable

#define R_MOTOR_IN3 26 // GPIO pin for R motor input 1
#define R_MOTOR_IN4 27 // GPIO pin for R motor input 2
#define R_MOTOR_ENB 22 // GPIO pin for R motor enable

// Global values to be called
extern bool turning_active;
extern volatile int stop_running;

// PID Values (Testing)
#define KP 0.005f
#define KI 0.002f
#define KD 0.006f

#define M_PI 3.14159265358979323846

//PWM values for movement
#define MAX_DUTY_CYCLE 1.0f
#define MIN_DUTY_CYCLE 0.0f

/*PWM for line following-hakam look here*/
#define MAX_LINE_DUTY_CYCLE 0.60f
#define MIN_LINE_DUTY_CYCLE 0.50f

//Movement Enum for direction
typedef enum {
    FORWARD,
    BACKWARD,
    STOP,
    TURN_LEFT,
    TURN_RIGHT,
    MOTOR_ON_LINE
} MovementDirection;

// Motor control type
typedef struct {
    uint32_t ena_pin;         // GPIO pin for PWM enable (e.g., L_MOTOR_ENA or R_MOTOR_ENB)
    uint32_t in1_pin;         // GPIO pin for direction control 1 (e.g., L_MOTOR_IN1 or R_MOTOR_IN3)
    uint32_t in2_pin;         // GPIO pin for direction control 2 (e.g., L_MOTOR_IN2 or R_MOTOR_IN4)
    float current_speed;      // Current speed of the motor
    float target_speed;       // Target speed for the motor
    float target_position;    // Target position (if needed for positional control)
    float current_position;   // Current position 
} MotorConfig;

// PID control variables
typedef struct {
    float integral;
    float prev_error;
} PIDState;

// Prototype functions
void motor_init();
void move_car(MovementDirection direction, float speed, float angle);

#endif


