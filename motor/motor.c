// Control L and R motor speed

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "motor.h"
#include "math.h"
#include "../encoder/encoder.h"

extern volatile float actual_speed_left;
extern volatile float actual_speed_right;

// PID parameters
float Kp = 2.0;
float Ki = 2.0;
float Kd = 0.0;

// PID control variables
float integral_left = 0.0;
float integral_right = 0.0;
float prev_error_left = 0.0;
float prev_error_right = 0.0;

float setpoint_speed = 15.0;

volatile float pwm_left = 1900;
volatile float pwm_right = 1900;

// Function to initialize pins for motors
void init_motor_setup()
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
void init_motor_pwm()
{
    // Set GPIO pins for ENA and ENB to PWM mode
    gpio_set_function(L_MOTOR_ENA, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_ENB, GPIO_FUNC_PWM);

    // Get PWM slice and channel for ENA and ENB
    uint slice_left = pwm_gpio_to_slice_num(L_MOTOR_ENA);
    uint channel_left = pwm_gpio_to_channel(L_MOTOR_ENA);
    uint slice_right = pwm_gpio_to_slice_num(R_MOTOR_ENB);
    uint channel_right = pwm_gpio_to_channel(R_MOTOR_ENB);

    // Set PWM frequency to 40kHz (125MHz / 3125)
    pwm_set_wrap(slice_left, 3125);
    pwm_set_wrap(slice_right, 3125);

    // Set clock divider to 125
    pwm_set_clkdiv(slice_left, 125);
    pwm_set_clkdiv(slice_right, 125);

    // Enable PWM for both motor channels
    pwm_set_enabled(slice_left, true);
    pwm_set_enabled(slice_right, true);
}

// Function to move forward
void move_motor(float new_pwm_left, float new_pwm_right)
{
    printf("UPDATING MOTOR : LEFT - %f, RIGHT - %f\n", new_pwm_left, new_pwm_right);

    // stopMotor();
    sleep_ms(50);
    // Set both motors to output high for desired PWM
    // Get PWM slice and channel for ENA and ENB
    uint slice_left = pwm_gpio_to_slice_num(L_MOTOR_ENA);
    uint channel_left = pwm_gpio_to_channel(L_MOTOR_ENA);
    uint slice_right = pwm_gpio_to_slice_num(R_MOTOR_ENB);
    uint channel_right = pwm_gpio_to_channel(R_MOTOR_ENB);

    // Set PWM frequency to 40kHz (125MHz / 3125)
    pwm_set_wrap(slice_left, 3125);
    pwm_set_wrap(slice_right, 3125);

    // Set clock divider to 125
    pwm_set_clkdiv(slice_left, 125);
    pwm_set_clkdiv(slice_right, 125);

    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwm_left);
    // sleep_ms(50);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwm_right);

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
void reverse_motor(float new_pwm_left, float new_pwm_right)
{
    // stopMotor();
    sleep_ms(50);

    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwm_left);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwm_right);

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
void stop_motor()
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
void turn_motor(int direction)
{
    // pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), pwm);
    // pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), pwm);

    oscillation = 0;

    int target_notch_count = 190 * ENCODER_NOTCH / 360;
    move_motor(3125, 3125);

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

    while (oscillation < target_notch_count)
    {
        // wait
    }

    stop_motor();
    sleep_ms(50);
}

// Function to compute PID control signal
float compute_pid(float setpoint, float current_value, float *integral, float *prev_error)
{
    float error = setpoint - current_value;

    *integral += error;

    float derivative = error - *prev_error;

    float control_signal = Kp * error + Ki * *integral + Kd * derivative;

    *prev_error = error;

    return control_signal;
}

// Call this function at a regular interval, e.g., every 100ms to stabilise car
void update_motor_speed()
{
    // Compute the control signals
    pwm_left = compute_pid(setpoint_speed, actual_speed_left, &integral_left, &prev_error_left);
    pwm_right = compute_pid(setpoint_speed, actual_speed_right, &integral_right, &prev_error_right);
}

/*
int main() {
    stdio_init_all();

    // Initialise motor GPIO pins
    initMotorSetup();

    // Initialise motor PWM
    initMotorPWM();

    while (1) {
        // Run at half duty cycle for 2 seconds
        moveMotor(1563);
        sleep_ms(2000);

        // Turn right for 1 second
        turnMotor(1)
        sleep_ms(1000);

        // Run at full duty cycle for 2 seconds
        moveMotor(3125);
        sleep_ms(2000);

        // Turn left for 1 second
        turnMotor(0)
        sleep_ms(1000);

        // Stop for 5 seconds
        stopMotor()
        sleep_ms(5000);
    }

    return 0;
}
*/