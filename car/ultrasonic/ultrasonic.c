#include "ultrasonic.h"

// Global variables
volatile absolute_time_t start_time;
volatile uint64_t pulse_length = 0;
volatile double latest_distance = INVALID_DISTANCE;

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

    if (gpio == ECHOPIN)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            start_time = get_absolute_time();
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            pulse_length = absolute_time_diff_us(start_time, get_absolute_time());
            xSemaphoreGiveFromISR(pulse_semaphore, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task to process ultrasonic sensor readings
void ultrasonic_task(void *params)
{
    while (1)
    {
        // for (int i = 0; i < 5; i++)
        // {
            // Trigger the ultrasonic sensor
            send_echo_pulse();

            // Wait for pulse completion
            if (xSemaphoreTake(pulse_semaphore, pdMS_TO_TICKS(50)) == pdTRUE) // 50ms timeout
            {
                // Calculate distance from pulse length
                double measured = (pulse_length / 29.0 / 2.0) - 1;

                // Apply Kalman filter for stability
                kalman_update(kalman_filter, measured);

                // Update the global distance variable
                latest_distance = kalman_filter->x;
            }
            else
            {
                // printf("[ULTRASONIC] Ultrasonic sensor timeout or no pulse detected.\n");
            }

            // Delay to control the measurement rate (e.g., 10 Hz)
            vTaskDelay(pdMS_TO_TICKS(10)); // 100ms delay
        // }
    }
}

// Get filtered distance
double get_obstacle_distance()
{
    // printf("[ULTRASONIC] Distance: %.2f cm\n", latest_distance);
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

    // printf("[ULTRASONIC] Ultrasonic sensor waiting for first signal.\n");
    // while (get_obstacle_distance() == INVALID_DISTANCE) // Wait for the first reading
    // {
    //     vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay
    // }
}