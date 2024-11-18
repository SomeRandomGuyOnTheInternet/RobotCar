#include "ultrasonic.h"

// Global variables
volatile absolute_time_t start_time;
volatile uint64_t pulse_length = 0;
volatile double latest_distance = -1.0;

// FreeRTOS semaphore
SemaphoreHandle_t pulse_semaphore;

// Kalman filter instance
static kalman_state *kalman_filter;

// Initialize Kalman filter
kalman_state *kalman_init(double q, double r, double p, double initial_value)
{
    kalman_state *state = calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;

    return state;
}

void kalman_update(kalman_state *state, double measurement)
{
    // Prediction update
    state->p = state->p + state->q;

    // Measurement update
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;
}

// Send ultrasonic pulse
void send_echo_pulse()
{
    gpio_put(TRIGPIN, 1);
    sleep_us(10);
    gpio_put(TRIGPIN, 0);
}

// Interrupt service routine for echo pin
void read_echo_pulse(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_RISE)
    {
        // Rising edge detected, start the timer
        start_time = get_absolute_time();
    }
    else if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_FALL)
    {
        // Falling edge detected, calculate the pulse width
        pulse_length = absolute_time_diff_us(start_time, get_absolute_time());
        xSemaphoreGiveFromISR(pulse_semaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task to process ultrasonic sensor readings
void ultrasonic_task(void *params)
{
    while (1)
    {
        // Wait for pulse completion
        for (int i = 0; i < 20; i++)
        {
            if (xSemaphoreTake(pulse_semaphore, portMAX_DELAY) == pdTRUE)
            {
                // Calculate distance
                double measured = (pulse_length / 29.0 / 2.0) - 1;
                kalman_update(kalman_filter, measured);

                // Update the global distance variable
                latest_distance = kalman_filter->x;
            }

            // Trigger the ultrasonic sensor
            send_echo_pulse();
        }

        // Delay before the next measurement
        vTaskDelay(pdMS_TO_TICKS(10)); // 1 Hz update rate
    }
}

// Get filtered distance
double get_obstacle_distance()
{
    return latest_distance;
}

// Ultrasonic initialization
void ultrasonic_init()
{
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_put(TRIGPIN, 0);

    // Create FreeRTOS semaphore
    pulse_semaphore = xSemaphoreCreateBinary();

    // Initialize Kalman filter with appropriate parameters
    kalman_filter = kalman_init(5.0, 0.5, 0.1, 100.0);

    // Create a FreeRTOS task for ultrasonic sensor
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
}