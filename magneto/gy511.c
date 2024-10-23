#include "gy511.h"
// #include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Initialize the GY-511 (setup I2C and configure sensors)
void gy511_init() {
    // Initialize I2C at 100kHz
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C); // I2C SDA pin
    gpio_set_function(5, GPIO_FUNC_I2C); // I2C SCL pin
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Set up accelerometer control register
    uint8_t data[2];
    data[0] = CTRL_REG1_A; // Register address
    data[1] = 0x57;        // 100Hz, all axes enabled
    i2c_write_blocking(i2c0, ACCEL_ADDRESS, data, 2, false);
}

// Function to read accelerometer data
void gy511_read_accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    uint8_t reg = OUT_X_L_A | 0x80;  // Set MSB for auto-increment
    i2c_write_blocking(i2c0, ACCEL_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c0, ACCEL_ADDRESS, data, 6, false);
    
    *x = (int16_t)(data[1] << 8 | data[0]);
    *y = (int16_t)(data[3] << 8 | data[2]);
    *z = (int16_t)(data[5] << 8 | data[4]);
}

// Similarly, define gy511_read_gyro and gy511_read_mag
// int main() {
//     // Initialise standard I/O
//     stdio_init_all();
//     sleep_ms(1000);

//     // Initialise motor GPIO pins and PWM
//     init_motor_setup();
//     init_motor_pwm();

//     // Initialise ultrasonic sensor
//     gy511_init();
//     printf("Magnetometer pins initialised\n");
//     sleep_ms(500);

//     int16_t accel_x, accel_y, accel_z;

//     while (1)
//     {
//         sleep_ms(100); // Reduced sleep for more responsive readings

//         // Read accelerometer data
//         gy511_read_accel(&accel_x, &accel_y, &accel_z);
//         printf("Accel X: %d, Y: %d, Z: %d\n", accel_x, accel_y, accel_z);

//         // Adjust speed based on Y-axis tilt
//         if (accel_y > 5000) // Forward tilt
//         {
//             move_motor(2500, 2500);
//         }
//         else if (accel_y < -5000) // Backward tilt
//         {
//             reverse_motor(2500, 2500);
//         }

//         // Adjust turning based on X-axis tilt
//         if (accel_x > 5000) // Right tilt
//         {
//             move_motor(2500, -2500);
//         }
//         else if (accel_x < -5000) // Left tilt
//         {
//             move_motor(-2500, 2500);
//         }

//         // printf("Distance: %.2lf\n", cm);
//         sleep_ms(500); // Delay between readings
//     }
// }