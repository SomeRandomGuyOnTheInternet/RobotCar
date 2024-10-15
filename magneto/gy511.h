#ifndef GY511_H
#define GY511_H

#include "hardware/i2c.h"   // For I2C communication
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

// Function prototypes
void gy511_init();
void gy511_read_accel(int16_t *x, int16_t *y, int16_t *z);
void gy511_read_gyro(int16_t *x, int16_t *y, int16_t *z);
void gy511_read_mag(int16_t *x, int16_t *y, int16_t *z);

#endif
