#ifndef GY511_H
#define GY511_H

#include "i2c.h"   // For I2C communication
#include "pico/stdlib.h"    // For standard Pico functions

// I2C address for LSM303DLHC (Accelerometer & Magnetometer)
#define ACCEL_ADDRESS  0x19
#define MAG_ADDRESS    0x1E

// L3G4200D (Gyroscope) I2C address
#define GYRO_ADDRESS   0x69

// Register addresses (example for accelerometer)
#define CTRL_REG1_A    0x20
#define OUT_X_L_A      0x28
#define OUT_X_H_A      0x29
#define OUT_Y_L_A      0x2A
#define OUT_Y_H_A      0x2B
#define OUT_Z_L_A      0x2C
#define OUT_Z_H_A      0x2D

// Magnetometer register addresses (these may vary, refer to your datasheet)
#define OUT_X_L_M 0x03 // LSB of X-axis magnetometer data
#define OUT_X_H_M 0x04 // MSB of X-axis magnetometer data
#define OUT_Y_L_M 0x05 // LSB of Y-axis magnetometer data
#define OUT_Y_H_M 0x06 // MSB of Y-axis magnetometer data
#define OUT_Z_L_M 0x07 // LSB of Z-axis magnetometer data
#define OUT_Z_H_M 0x08 // MSB of Z-axis magnetometer data

// Function prototypes
void gy511_init();
void gy511_read_accel(int16_t *x, int16_t *y, int16_t *z);
void gy511_read_gyro(int16_t *x, int16_t *y, int16_t *z);
void gy511_read_mag(int16_t *x, int16_t *y, int16_t *z);
char* gy511_read_data();
void gy511_get_data(int16_t *x, int16_t *y, int16_t *z, int16_t *fixed_x, int16_t *fixed_y);
//void gy511_get_adjusted_data(int16_t *orig_x, int16_t *orig_y, int16_t *orig_z,
                             //int16_t *fixed_x, int16_t *fixed_y, int16_t *fixed_z);



#endif
