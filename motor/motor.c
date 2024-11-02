// Control L and R motor speed

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "motor.h"
#include "math.h"
#include "../encoder/encoder.h"

extern volatile float actual_speed_left; // Get encoder speed for left motor
extern volatile float actual_speed_right; // Get encoder speed for right motor

float setpoint_speed = 15.0; // Target speed in cm/s (or unit of choice)

// Separate PID gains for each motor
float Kp_L = 1.5;
float Kp_R = 1.5;
float Ki_L = 0.5;
float Ki_R = 0.5;
float Kd_L = 0.1;
float Kd_R = 0.1;

// PID control variables
float integral_L = 0.0;
float integral_R = 0.0;
float prev_error_L = 0.0;
float prev_error_R = 0.0;

float setpoint_speed = 15.0;

volatile float pwmL = 1900;
volatile float pwmR = 1900;

// Function to initialize pins for motors
void initMotorSetup()
{
    // Initialize GPIO pins for L motor control
    gpio_init(L_MOTOR_IN1);
    gpio_init(L_MOTOR_IN2);
    gpio_init(L_MOTOR_ENA);

    // Initialize GPIO pins for R motor control
    gpio_init(R_MOTOR_IN3);
    gpio_init(R_MOTOR_IN4);
    gpio_init(R_MOTOR_ENB);

    // Set GPIO pins as outputs for L motor
    gpio_set_dir(L_MOTOR_IN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_IN2, GPIO_OUT);
    gpio_set_dir(L_MOTOR_ENA, GPIO_OUT);

    // Set GPIO pins as outputs for R motor
    gpio_set_dir(R_MOTOR_IN3, GPIO_OUT);
    gpio_set_dir(R_MOTOR_IN4, GPIO_OUT);
    gpio_set_dir(R_MOTOR_ENB, GPIO_OUT);

    // Enable the EN pins
    gpio_put(L_MOTOR_ENA, 1);
    gpio_put(R_MOTOR_ENB, 1);
}

// Function to initialize PWMs for motors
void initMotorPWM()
{
    // Set GPIO pins for ENA and ENB to PWM mode
    gpio_set_function(L_MOTOR_ENA, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_ENB, GPIO_FUNC_PWM);

    // Get PWM slice and channel for ENA and ENB
    uint sliceLeft = pwm_gpio_to_slice_num(L_MOTOR_ENA);
    uint channelLeft = pwm_gpio_to_channel(L_MOTOR_ENA);
    uint sliceRight = pwm_gpio_to_slice_num(R_MOTOR_ENB);
    uint channelRight = pwm_gpio_to_channel(R_MOTOR_ENB);

    // Set PWM frequency to 40kHz (125MHz / 3125)
    pwm_set_wrap(sliceLeft, 3125);
    pwm_set_wrap(sliceRight, 3125);

    // Set clock divider to 125
    pwm_set_clkdiv(sliceLeft, 125);
    pwm_set_clkdiv(sliceRight, 125);

    // Enable PWM for both motor channels
    pwm_set_enabled(sliceLeft, true);
    pwm_set_enabled(sliceRight, true);
}

// Function to move forward
void moveMotor(float new_pwmL, float new_pwmR)
{
    // printf("UPDATING MOTOR : LEFT - %f, RIGHT - %f\n", new_pwmL, new_pwmR);

    // stopMotor();
    sleep_ms(50);
    // Set both motors to output high for desired PWM
    // Get PWM slice and channel for ENA and ENB
    uint sliceLeft = pwm_gpio_to_slice_num(L_MOTOR_ENA);
    uint channelLeft = pwm_gpio_to_channel(L_MOTOR_ENA);
    uint sliceRight = pwm_gpio_to_slice_num(R_MOTOR_ENB);
    uint channelRight = pwm_gpio_to_channel(R_MOTOR_ENB);

    // Set PWM frequency to 40kHz (125MHz / 3125)
    pwm_set_wrap(sliceLeft, 3125);
    pwm_set_wrap(sliceRight, 3125);

    // Set clock divider to 125
    pwm_set_clkdiv(sliceLeft, 125);
    pwm_set_clkdiv(sliceRight, 125);

    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwmL);
    // sleep_ms(50);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwmR);

    // Turn on both motors
    gpio_put(L_MOTOR_IN1, 0);
    gpio_put(L_MOTOR_IN2, 1);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 1);

    // Enable the enable pins
    gpio_put(L_MOTOR_ENA, 1);
    gpio_put(R_MOTOR_ENB, 1);
}

// Function to move backward
void reverseMotor(float new_pwmL, float new_pwmR)
{
    // stopMotor();
    sleep_ms(50);

    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwmL);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwmR);

    // Turn on both motors
    gpio_put(L_MOTOR_IN1, 1);
    gpio_put(L_MOTOR_IN2, 0);
    gpio_put(R_MOTOR_IN3, 1);
    gpio_put(R_MOTOR_IN4, 0);

    // Enable the enable pins
    gpio_put(L_MOTOR_ENA, 1);
    gpio_put(R_MOTOR_ENB, 1);
}

// Function to stop
void stopMotor()
{
    // Turn off all motors
    gpio_put(L_MOTOR_IN1, 0);
    gpio_put(L_MOTOR_IN2, 0);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 0);

    // Disable the enable pins
    gpio_put(L_MOTOR_ENA, 0);
    gpio_put(R_MOTOR_ENB, 0);
}

// Function to turn
// 0 - left, 1 - right
void turnMotor(int direction)
{
    // pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), pwm);
    // pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), pwm);

    oscillation = 0;

    int targetNotchCount = 190 * ENCODER_NOTCH / 360;
    moveMotor(3125, 3125);

    // Motor to turn left
    if (direction == 0)
    {
        // Reverse left wheel, forward right wheel
        gpio_put(L_MOTOR_IN1, 1);
        gpio_put(L_MOTOR_IN2, 0);
        gpio_put(R_MOTOR_IN3, 0);
        gpio_put(R_MOTOR_IN4, 1);

        // Enable the enable pins
        gpio_put(L_MOTOR_ENA, 1);
        gpio_put(R_MOTOR_ENB, 1);
    }
    // Motor to turn right
    else
    {
        // Reverse right wheel, forward left wheel
        gpio_put(L_MOTOR_IN1, 0);
        gpio_put(L_MOTOR_IN2, 1);
        gpio_put(R_MOTOR_IN3, 1);
        gpio_put(R_MOTOR_IN4, 0);

        // Enable the enable pins
        gpio_put(L_MOTOR_ENA, 1);
        gpio_put(R_MOTOR_ENB, 1);
    }

    while (oscillation < targetNotchCount)
    {
        // wait
    }

    stopMotor();
    sleep_ms(50);
}

/* Function to move forward for a set number of grids
void moveGrids(int number_of_grids)
{
    startTracking(number_of_grids);

    while (!completeMovement)
    {
        moveMotor(pwmL, pwmR);
        sleep_ms(50);
    }
    // Stop once reached target grids
    stopMotor();
}*/

/* Function to move backwards for a set number of grids
void reverseGrids(int number_of_grids)
{
    startTracking(number_of_grids);

    while (!completeMovement)
    {
        reverseMotor(pwmL, pwmR);
        sleep_ms(50);
    }

    // Stop once reached target grids
    stopMotor();
}*/

// Function to compute PID control signal
float compute_pid(float setpoint, float current_value, float Kp, float Ki, float Kd, float *integral, float *prev_error) {
    float error = setpoint - current_value;
    *integral += error;

    float derivative = error - *prev_error;
    float control_signal = Kp * error + Ki * (*integral) + Kd * derivative;

    *prev_error = error;
    return control_signal;
}

// Call this function at a regular interval, e.g., every 100ms to stabilise car
void update_motor_speed() {
    pwmL = compute_pid(setpoint_speed, actual_speed_left, Kp_L, Ki_L, Kd_L, &integral_L, &prev_error_L);
    pwmR = compute_pid(setpoint_speed, actual_speed_right, Kp_R, Ki_R, Kd_R, &integral_R, &prev_error_R);

    // Ensure PWM levels are within the valid range
    if (pwmL < PWM_MIN) pwmL = PWM_MIN;
    if (pwmR < PWM_MIN) pwmR = PWM_MIN;
    if (pwmL > PWM_MAX) pwmL = PWM_MAX;
    if (pwmR > PWM_MAX) pwmR = PWM_MAX;
}




