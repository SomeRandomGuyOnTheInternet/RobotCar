#include "gy511.h"
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#define FILTER_SIZE 3
#define ALPHA 0.8
#define MAX_TILT 10000
#define CALIBRATION_BUTTON 20

int16_t filter_accel_x[FILTER_SIZE] = {0};
int16_t filter_accel_y[FILTER_SIZE] = {0};
int16_t filter_accel_z[FILTER_SIZE] = {0};

int16_t accel_x_offset = 0;
int16_t accel_y_offset = 0;
int16_t accel_z_offset = 0;

int16_t old_accel_x = 0;
int16_t old_accel_y = 0;
int16_t old_accel_z = 0;

int is_calibrated = 0;
static volatile bool calibration_requested = false;
static volatile bool ready_for_calibration = false;

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == CALIBRATION_BUTTON)
    {
        calibration_requested = true;
    }
}

void my_busy_wait_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 1000; i++)
    {
        __asm__("nop");
    }
}

void gy511_init()
{
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Initialize calibration button with interrupt
    gpio_init(CALIBRATION_BUTTON);
    gpio_set_dir(CALIBRATION_BUTTON, GPIO_IN);
    gpio_pull_up(CALIBRATION_BUTTON);
    gpio_set_irq_enabled_with_callback(CALIBRATION_BUTTON, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    uint8_t data[2];
    data[0] = CTRL_REG1_A;
    data[1] = 0x57;
    i2c_write_blocking(i2c0, ACCEL_ADDRESS, data, 2, false);
}

void gy511_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    uint8_t reg = OUT_X_L_A | 0x80;
    i2c_write_blocking(i2c0, ACCEL_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c0, ACCEL_ADDRESS, data, 6, false);

    *x = (int16_t)(data[1] << 8 | data[0]);
    *y = (int16_t)(data[3] << 8 | data[2]);
    *z = (int16_t)(data[5] << 8 | data[4]);
}

int16_t moving_average(int16_t *filter_array, int16_t new_value)
{
    int sum = 0;
    for (int i = 1; i < FILTER_SIZE; i++)
    {
        filter_array[i - 1] = filter_array[i];
        sum += filter_array[i - 1];
    }
    filter_array[FILTER_SIZE - 1] = new_value;
    sum += new_value;

    return sum / FILTER_SIZE;
}

int16_t apply_low_pass_filter(int16_t old_value, int16_t new_value)
{
    if (abs(new_value - old_value) > 8000)
    {
        return new_value;
    }
    return (int16_t)(ALPHA * new_value + (1 - ALPHA) * old_value);
}

void calibrate_accelerometer()
{
    int16_t accel_x, accel_y, accel_z;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int num_samples = 50;

    for (int i = 0; i < num_samples; i++)
    {
        gy511_read_accel(&accel_x, &accel_y, &accel_z);
        sum_x += accel_x;
        sum_y += accel_y;
        sum_z += accel_z;
        busy_wait_ms(20);
    }

    accel_x_offset = sum_x / num_samples;
    accel_y_offset = sum_y / num_samples;
    accel_z_offset = sum_z / num_samples;
    calibration_requested = false;
}

void gy511_get_data(int16_t *x, int16_t *y, int16_t *z, int16_t *fixed_x, int16_t *fixed_y)
{
    static int initialized = 0;
    int16_t accel_x, accel_y, accel_z;

    if (!initialized)
    {
        gy511_init();
        calibrate_accelerometer();
        initialized = 1;
    }

    if (calibration_requested)
    {
        *x = 0;
        *y = 0;
        *z = 0;
        *fixed_x = 0;
        *fixed_y = 0;
        calibration_requested = false;
        ready_for_calibration = true;
    }
    else if (ready_for_calibration)
    {
        printf("Calibrating accelerometer...\n");
        calibrate_accelerometer();
        *x = 0;
        *y = 0;
        *z = 0;
        *fixed_x = 0;
        *fixed_y = 0;
        calibration_requested = false;
        ready_for_calibration = false;
        printf("Calibration complete.\n");
    }
    else
    {
        gy511_read_accel(&accel_x, &accel_y, &accel_z);

        *x = moving_average(filter_accel_x, accel_x);
        *y = moving_average(filter_accel_y, accel_y);
        *z = moving_average(filter_accel_z, accel_z);

        *x = apply_low_pass_filter(old_accel_x, *x);
        *y = apply_low_pass_filter(old_accel_y, *y);
        *z = apply_low_pass_filter(old_accel_z, *z);

        old_accel_x = *x;
        old_accel_y = *y;
        old_accel_z = *z;

        *x -= accel_x_offset;
        *y -= accel_y_offset;
        *z -= accel_z_offset;

        if (*x > MAX_TILT)
        {
            *x = MAX_TILT;
        }
        else if (*x < -MAX_TILT)
        {
            *x = -MAX_TILT;
        }

        if (*y > MAX_TILT)
        {
            *y = MAX_TILT;
        }
        else if (*y < -MAX_TILT)
        {
            *y = -MAX_TILT;
        }

        if (*x > -6000 && *x < 6000)
        {
            *fixed_x = 0;
        }
        else if (*x >= 6000 && *x < 7500)
        {
            *fixed_x = 1;
        }
        else if (*x >= 7500 && *x < 9000)
        {
            *fixed_x = 2;
        }
        else if (*x >= 9000 && *x <= 10000)
        {
            *fixed_x = 3;
        }
        else if (*x < -6000 && *x > -7500)
        {
            *fixed_x = -1;
        }
        else if (*x <= -7500 && *x > -9000)
        {
            *fixed_x = -2;
        }
        else if (*x <= -9000 && *x >= -10000)
        {
            *fixed_x = -3;
        }

        if (*y > -7000 && *y < 4000)
        {
            *fixed_y = 0;
        }
        else if (*y >= 4000 && *y < 6000)
        {
            *fixed_y = 1;
        }
        else if (*y >= 6000 && *y < 8000)
        {
            *fixed_y = 2;
        }
        else if (*y >= 8000 && *y <= 10000)
        {
            *fixed_y = 3;
        }
        else if (*y < -7000 && *y > -8000)
        {
            *fixed_y = -1;
        }
        else if (*y <= -8000 && *y > -9000)
        {
            *fixed_y = -2;
        }
        else if (*y <= -9000 && *y >= -10000)
        {
            *fixed_y = -3;
        }
    }
}
