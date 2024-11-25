#include "gy511.h" 
#include <stdio.h> 
#include <math.h> // For atan2 and M_PI 
#include <unistd.h> // For sleep_ms 
#include <stdlib.h>

 
// Filtering constants 
#define FILTER_SIZE 3 
#define ALPHA 0.8  // Low-pass filter constant 
 
// Arrays for moving average filter 
int16_t filter_accel_x[FILTER_SIZE] = {0}; 
int16_t filter_accel_y[FILTER_SIZE] = {0}; 
int16_t filter_accel_z[FILTER_SIZE] = {0}; 
 
// Offsets for gravitational compensation 
int16_t accel_x_offset = 0; 
int16_t accel_y_offset = 0; 
int16_t accel_z_offset = 0; 
 
// Previous values for low-pass filter 
int16_t old_accel_x = 0; 
int16_t old_accel_y = 0; 
int16_t old_accel_z = 0; 
 
// Flag to check if calibration is done 
int is_calibrated = 0; 

#define MAX_TILT 10000   // Maximum value for X and Y axes


// Busy wait for ms
void my_busy_wait_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __asm__("nop");
    }
}
 
// Initialize the GY-511 sensor 
void gy511_init() { 
    i2c_init(i2c0, 100 * 1000); 
    gpio_set_function(4, GPIO_FUNC_I2C); 
    gpio_set_function(5, GPIO_FUNC_I2C); 
    gpio_pull_up(4); 
    gpio_pull_up(5); 
 
    // Set accelerometer to 100Hz sampling rate 
    uint8_t data[2]; 
    data[0] = CTRL_REG1_A; 
    data[1] = 0x57; 
    i2c_write_blocking(i2c0, ACCEL_ADDRESS, data, 2, false); 
} 
 
// Read accelerometer data 
void gy511_read_accel(int16_t *x, int16_t *y, int16_t *z) { 
    uint8_t data[6]; 
    uint8_t reg = OUT_X_L_A | 0x80; 
    i2c_write_blocking(i2c0, ACCEL_ADDRESS, &reg, 1, true); 
    i2c_read_blocking(i2c0, ACCEL_ADDRESS, data, 6, false); 
     
    *x = (int16_t)(data[1] << 8 | data[0]); 
    *y = (int16_t)(data[3] << 8 | data[2]); 
    *z = (int16_t)(data[5] << 8 | data[4]); 
} 
 
// Apply moving average filter 
int16_t moving_average(int16_t *filter_array, int16_t new_value) { 
    int sum = 0; 
    for (int i = 1; i < FILTER_SIZE; i++) { 
        filter_array[i - 1] = filter_array[i]; 
        sum += filter_array[i - 1]; 
    } 
    filter_array[FILTER_SIZE - 1] = new_value; 
    sum += new_value; 
 
    return sum / FILTER_SIZE; 
} 
 
// Apply low-pass filter 
int16_t apply_low_pass_filter(int16_t old_value, int16_t new_value) {
    if (abs(new_value - old_value) > 8000) {  // Threshold for large change
        return new_value;  // Bypass filter for instant change
    }
    return (int16_t)(ALPHA * new_value + (1 - ALPHA) * old_value);
}
 
void calibrate_accelerometer() {
    int16_t accel_x, accel_y, accel_z;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int num_samples = 50;

    for (int i = 0; i < num_samples; i++) {
        gy511_read_accel(&accel_x, &accel_y, &accel_z);
        sum_x += accel_x;
        sum_y += accel_y;
        sum_z += accel_z;
        busy_wait_ms(20);  // Use busy wait to avoid sleep_ms
    }

    accel_x_offset = sum_x / num_samples;
    accel_y_offset = sum_y / num_samples;
    accel_z_offset = sum_z / num_samples;
}

void gy511_get_data(int16_t *x, int16_t *y, int16_t *z, int16_t *fixed_x, int16_t *fixed_y) {
    static int initialized = 0;
    int16_t accel_x, accel_y, accel_z;

    // Initialize and calibrate only once
    if (!initialized) {
        gy511_init();
        calibrate_accelerometer();
        initialized = 1;
    }

    // Read accelerometer data
    gy511_read_accel(&accel_x, &accel_y, &accel_z);

    // Apply moving average filter
    *x = moving_average(filter_accel_x, accel_x);
    *y = moving_average(filter_accel_y, accel_y);
    *z = moving_average(filter_accel_z, accel_z);

    // Apply low-pass filter
    *x = apply_low_pass_filter(old_accel_x, *x);
    *y = apply_low_pass_filter(old_accel_y, *y);
    *z = apply_low_pass_filter(old_accel_z, *z);

    // Update old values for the next filter cycle
    old_accel_x = *x;
    old_accel_y = *y;
    old_accel_z = *z;

    // Apply offsets to remove static gravity influence and center the axes
    *x -= accel_x_offset;
    *y -= accel_y_offset;
    *z -= accel_z_offset;

    // Apply maximum thresholds for X and Y axes
    if (*x > MAX_TILT) {
        *x = MAX_TILT;
    } else if (*x < -MAX_TILT) {
        *x = -MAX_TILT;
    }

    if (*y > MAX_TILT) {
        *y = MAX_TILT;
    } else if (*y < -MAX_TILT) {
        *y = -MAX_TILT;
    }

    // Process X-axis movement (Fixed X variable)
    if (*x > -1000 && *x < 1000) {
        *fixed_x = 0;  // No movement
    } else if (*x > 0) {
        *fixed_x = 1;  // Movement to the right
    } else {
        *fixed_x = -1;  // Movement to the left
    }

    // Process Y-axis movement (Fixed Y variable)
    if (*y > -1000 && *y < 1000) {
        *fixed_y = 0;  // No movement
    } else if (*y >= 1000 && *y < 4000) {
        *fixed_y = 1;  // Forward movement (small)
    } else if (*y >= 4000 && *y < 8000) {
        *fixed_y = 2;  // Forward movement (medium)
    } else if (*y >= 8000 && *y <= 10000) {
        *fixed_y = 3;  // Forward movement (large)
    } else if (*y < -1000 && *y > -4000) {
        *fixed_y = -1;  // Backward movement (small)
    } else if (*y <= -4000 && *y > -8000) {
        *fixed_y = -2;  // Backward movement (medium)
    } else if (*y <= -8000 && *y >= -10000) {
        *fixed_y = -3;  // Backward movement (large)
    }
}