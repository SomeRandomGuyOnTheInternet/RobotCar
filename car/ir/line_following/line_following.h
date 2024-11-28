#ifndef line_following_h
#define line_following_h

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"

// Define GPIO pins for line sensors
#define LINE_SENSOR_PIN 28 // Define the GPIO pin for the line sensor
#define STEER_DURATION 213
#define LINE_THRESHOLD 200    // Define a custom threshold for line detection
#define DEBOUNCE_DELAY_MS 100 // 100 ms debounce delay
#define TURNING_SPEED 2000       // Slower speed for turning

// Function Prototypes

/**
 * @brief Initializes the line-following sensor using ADC.
 *
 * Configures the line sensor GPIO pin for ADC input.
 */
void init_line_sensor(void);

/**
 * @brief Turns the motors using PID control to follow the line.
 *
 * @param direction The direction to turn (0 for left, 1 for right).
 * @param steer_duration The duration in milliseconds for which the turn should last.
 *
 * This function controls the motors using PID control to turn the robot in the specified direction
 * for the given duration. It checks the line sensor and exits the turn early if the line is detected.
 */
void perform_turn(int direction, uint64_t steer_duration);

/**
 * @brief Task function for line-following logic.
 *
 * This function is intended to be used as a FreeRTOS task. It continuously reads the line sensor value
 * and adjusts the motor control logic to keep the robot on track. If the line is detected, it moves forward.
 * If the line is not detected, it initiates a slow turn in alternating directions. If consecutive reversals occur,
 * the steering duration is increased by 20 ms after each reversal until the line is detected again, at which point it resets.
 *
 * @param pvParameters Parameters for the task (not used).
 */
void lineFollowTask(void *pvParameters);

#endif // LINE_FOLLOWING_H