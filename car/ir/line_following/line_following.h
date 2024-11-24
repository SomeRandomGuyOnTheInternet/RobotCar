#ifndef line_following_h
#define line_following_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdbool.h>

// Define GPIO pins for line sensors
#define LINE_SENSOR_PIN 28    // Define the GPIO pin for the line sensor
#define STEER_DURATION 160   


// Function Prototypes

/**
 * @brief Initializes the line-following sensors.
 * 
 * Configures the left and right line sensor GPIO pins as input and enables pull-up resistors.
 */
void init_line_sensor(void);
int start_line_following();

/**
 * @brief Follows the line based on input from the line-following sensors.
 * 
 * This function adjusts the motor control logic to keep the robot on track.
 * Prints messages like "Line detected. Car is following." and "Out of course, correcting car."
 */
void follow_line(void);

#endif // LINE_FOLLOWING_H
