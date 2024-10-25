// Driver for controlling motor speed & direction variance

#include <stdio.h>
#include "pico/stdlib.h"      // Standard library for Raspberry Pi Pico
#include "hardware/gpio.h"    // GPIO library for controlling the GPIO pins
#include "hardware/pwm.h"     // PWM (Pulse Width Modulation) library for controlling motor speed
#include "hardware/timer.h"   // Timer library for managing delays
#include "motor.h"            // Custom motor header file containing function declarations and motor pin mappings
#include "math.h"             // Math library for performing mathematical operations
#include "../encoder/encoder.h" // Encoder library for reading encoder data (not fully shown in this file)

// External variables to hold the actual motor speeds, updated elsewhere (likely in an ISR)
extern volatile float actual_speed_left;
extern volatile float actual_speed_right;

// PID parameters (Proportional, Integral, Derivative) for motor speed control
float Kp = 2.0;  // Proportional gain
float Ki = 2.0;  // Integral gain
float Kd = 0.0;  // Derivative gain

// Variables for PID control, used to compute the necessary adjustments to the motor speed
float integral_left = 0.0;  // Accumulated error for left motor (integral term)
float integral_right = 0.0; // Accumulated error for right motor (integral term)
float prev_error_left = 0.0;  // Previous error for left motor (for derivative term)
float prev_error_right = 0.0; // Previous error for right motor

float setpoint_speed = 15.0;  // Target speed for the motors

// Variables to store the PWM values that control the motor speed (initially set to 1900)
volatile float pwm_left = 1900;
volatile float pwm_right = 1900;

// Function to initialize GPIO pins for motor control
void init_motor_setup()
{
    // Initialize GPIO pins for the left motor control
    gpio_init(L_MOTOR_IN1);  // Initialize left motor input 1
    gpio_init(L_MOTOR_IN2);  // Initialize left motor input 2
    gpio_init(L_MOTOR_ENA);  // Initialize left motor enable pin

    // Initialize GPIO pins for the right motor control
    gpio_init(R_MOTOR_IN3);  // Initialize right motor input 1
    gpio_init(R_MOTOR_IN4);  // Initialize right motor input 2
    gpio_init(R_MOTOR_ENB);  // Initialize right motor enable pin

    // Set GPIO pins as output for controlling the motors
    gpio_set_dir(L_MOTOR_IN1, GPIO_OUT);  // Set direction for left motor input 1 as output
    gpio_set_dir(L_MOTOR_IN2, GPIO_OUT);  // Set direction for left motor input 2 as output
    gpio_set_dir(L_MOTOR_ENA, GPIO_OUT);  // Set direction for left motor enable pin as output

    gpio_set_dir(R_MOTOR_IN3, GPIO_OUT);  // Set direction for right motor input 1 as output
    gpio_set_dir(R_MOTOR_IN4, GPIO_OUT);  // Set direction for right motor input 2 as output
    gpio_set_dir(R_MOTOR_ENB, GPIO_OUT);  // Set direction for right motor enable pin as output

    // Enable the left and right motors by setting their enable pins high
    gpio_put(L_MOTOR_ENA, 1);  // Enable left motor
    gpio_put(R_MOTOR_ENB, 1);  // Enable right motor
}

// Function to set up the PWM (Pulse Width Modulation) for controlling motor speed
void init_motor_pwm()
{
    // Set the motor enable pins (ENA and ENB) to PWM mode to control speed via duty cycle
    gpio_set_function(L_MOTOR_ENA, GPIO_FUNC_PWM);  // Set left motor enable pin to PWM mode
    gpio_set_function(R_MOTOR_ENB, GPIO_FUNC_PWM);  // Set right motor enable pin to PWM mode

    // Get the PWM slices (hardware units for generating PWM signals) and channels for each motor
    uint slice_left = pwm_gpio_to_slice_num(L_MOTOR_ENA);  // Get PWM slice for left motor
    uint channel_left = pwm_gpio_to_channel(L_MOTOR_ENA);  // Get PWM channel for left motor
    uint slice_right = pwm_gpio_to_slice_num(R_MOTOR_ENB); // Get PWM slice for right motor
    uint channel_right = pwm_gpio_to_channel(R_MOTOR_ENB); // Get PWM channel for right motor

    // Set PWM frequency to 40kHz by setting wrap value (125MHz clock / 3125 gives 40kHz)
    pwm_set_wrap(slice_left, 3125);   // Set wrap value for left motor
    pwm_set_wrap(slice_right, 3125);  // Set wrap value for right motor

    // Set clock divider to 125 to control the frequency further
    pwm_set_clkdiv(slice_left, 125);  // Set clock divider for left motor
    pwm_set_clkdiv(slice_right, 125); // Set clock divider for right motor

    // Enable PWM for both motor channels
    pwm_set_enabled(slice_left, true);   // Enable PWM for left motor
    pwm_set_enabled(slice_right, true);  // Enable PWM for right motor
}

// Function to move the motors forward with a specified speed (via PWM values)
void move_motor(float new_pwm_left, float new_pwm_right)
{
    printf("UPDATING MOTOR : LEFT - %f, RIGHT - %f\n", new_pwm_left, new_pwm_right);  // Print the updated PWM values for debugging

    sleep_ms(50);  // Delay to prevent abrupt changes

    // Set the PWM levels (duty cycles) for both motors to control speed
    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwm_left);  // Left motor PWM
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwm_right); // Right motor PWM

    // Set the direction for both motors to move forward
    gpio_put(L_MOTOR_IN1, 0);  // Left motor forward direction
    gpio_put(L_MOTOR_IN2, 1);  // Left motor forward direction
    gpio_put(R_MOTOR_IN3, 0);  // Right motor forward direction
    gpio_put(R_MOTOR_IN4, 1);  // Right motor forward direction

    // Ensure the motors are enabled
    gpio_put(L_MOTOR_ENA, 1);  // Enable left motor
    gpio_put(R_MOTOR_ENB, 1);  // Enable right motor
}

// Function to move the motors backward with specified speed (via PWM values)
void reverse_motor(float new_pwm_left, float new_pwm_right)
{
    printf("UPDATING MOTOR : LEFT - %f, RIGHT - %f\n", new_pwm_left, new_pwm_right);  // Print the updated PWM values for debugging

    sleep_ms(50);  // Delay to prevent abrupt changes

    // Set the PWM levels (duty cycles) for both motors to control speed
    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwm_left);  // Left motor PWM
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwm_right); // Right motor PWM

    // Set the direction for both motors to move backward
    gpio_put(L_MOTOR_IN1, 1);  // Left motor reverse direction
    gpio_put(L_MOTOR_IN2, 0);  // Left motor reverse direction
    gpio_put(R_MOTOR_IN3, 1);  // Right motor reverse direction
    gpio_put(R_MOTOR_IN4, 0);  // Right motor reverse direction

    // Ensure the motors are enabled
    gpio_put(L_MOTOR_ENA, 1);  // Enable left motor
    gpio_put(R_MOTOR_ENB, 1);  // Enable right motor
}

// Function to stop both motors
void stop_motor()
{
    // Stop all motors by setting their control pins low (no power to motors)
    gpio_put(L_MOTOR_IN1, 0);  // Left motor stop
    gpio_put(L_MOTOR_IN2, 0);  // Left motor stop
    gpio_put(R_MOTOR_IN3, 0);  // Right motor stop
    gpio_put(R_MOTOR_IN4, 0);  // Right motor stop

    // Disable the enable pins, effectively cutting off the power to the motors
    gpio_put(L_MOTOR_ENA, 0);  // Disable left motor
    gpio_put(R_MOTOR_ENB, 0);  // Disable right motor
}

// IGNORE FOR NOW Function to compute the PID control signal for speed adjustment
float compute_pid(float setpoint, float current_value, float *integral, float *prev_error)
{
    float error = setpoint - current_value;  // Calculate the error between desired speed and actual speed

    *integral += error;  // Accumulate the error over time for the integral term

    float derivative = error - *prev_error;  // Calculate the rate of change of the error for the derivative term

    // Calculate the control signal using the PID formula
    float control_signal = Kp * error + Ki * *integral + Kd * derivative;

    *prev_error = error;  // Update the previous error for the next iteration

    return control_signal;  // Return the control signal (used to adjust PWM)
}

// Function to update motor speeds based on PID control, called periodically
void update_motor_speed()
{
    // Compute the PID-controlled PWM values for the left and right motors
    pwm_left = compute_pid(setpoint_speed, actual_speed_left, &integral_left, &prev_error_left);  // Left motor PID adjustment
    pwm_right = compute_pid(setpoint_speed, actual_speed_right, &integral_right, &prev_error_right); // Right motor PID adjustment
}

// Main function that demonstrates motor movement in forward and reverse directions, then stops
int main() {
    stdio_init_all();  // Initialize standard I/O for debugging (e.g., printf)

    // Initialize motor control pins and PWM setup
    init_motor_setup();  // Initialize motor GPIO pins
    init_motor_pwm();    // Initialize motor PWM

    // Main loop to control motor movement
    while (1) {
        // Move forward at half speed for 2 seconds
        move_motor(1563, 1563);  // Set both motors to move forward at half speed
        sleep_ms(2000);  // Wait for 2 seconds

        stop_motor();
        sleep_ms(2000);

        // Move forward at full speed for 2 seconds
        move_motor(3125, 3125);  // Set both motors to move forward at full speed
        sleep_ms(2000);  // Wait for 2 seconds

        // Stop for 2 secondsc
        stop_motor();  // Stop both motors
        sleep_ms(2000);  // Wait for 5 seconds

        // Reverse at half speed for 2 seconds
        reverse_motor(1563, 1563);  // Set both motors to move backward at half speed
        sleep_ms(2000);  // Wait for 2 seconds

        stop_motor();
        sleep_ms(2000);

        // Reverse at full speed for 2 seconds
        reverse_motor(3125, 3125);  // Set both motors to move backward at full speed
        sleep_ms(2000);  // Wait for 2 seconds

        // Stop for 5 seconds
        stop_motor();  // Stop both motors
        sleep_ms(5000);  // Wait for 5 seconds
    }

    return 0;  // Program will never reach here due to the infinite loop
}
