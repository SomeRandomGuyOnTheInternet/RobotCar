#include "encoder.h"

// Global variables protected by mutex
volatile EncoderData left_data = {0, 0};
volatile EncoderData right_data = {0, 0};
static EncoderData left_last_data = {0, 0};
static EncoderData right_last_data = {0, 0};

// Mutexes for each encoder
SemaphoreHandle_t left_data_mutex;
SemaphoreHandle_t right_data_mutex;

// Separate queues for each encoder
static QueueHandle_t left_encoder_queue;
static QueueHandle_t right_encoder_queue;

void read_encoder_pulse(uint gpio, uint32_t events)
{
    if (gpio == L_ENCODER_OUT)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(left_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE)
        {
            left_data.pulse_count++;
            left_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(left_data_mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    else if (gpio == R_ENCODER_OUT)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(right_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE)
        {
            right_data.pulse_count++;
            right_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(right_data_mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Task to handle the left encoder
void left_encoder_task(void *params)
{
    EncoderData data, last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (xSemaphoreTake(left_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            data = left_data;
            xSemaphoreGive(left_data_mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count)
            {
                xQueueReset(left_encoder_queue);
                if (xQueueSendToBack(left_encoder_queue, &data, 0) == pdTRUE)
                {
                    // printf("Left encoder - Count: %lu, Timestamp: %llu\n", data.pulse_count, data.timestamp);
                    last_sent = data;
                }
                else
                {
                    printf("Left encoder queue send failed\n");
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10)); // Adjust frequency as needed
    }
}

// Task to handle the right encoder
void right_encoder_task(void *params)
{
    EncoderData data, last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            data = right_data;
            xSemaphoreGive(right_data_mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count)
            {
                xQueueReset(right_encoder_queue);
                if (xQueueSendToBack(right_encoder_queue, &data, 0) == pdTRUE)
                {
                    // printf("Right encoder - Count: %lu, Timestamp: %llu\n", data.pulse_count, data.timestamp);
                    last_sent = data;
                }
                else
                {
                    printf("Right encoder queue send failed\n");
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10)); // Adjust frequency as needed
    }
}

// Function to get distance for left encoder
float get_left_distance()
{
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(left_encoder_queue, &data, 0) == pdTRUE)
    {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        printf("Left Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               data.pulse_count, distance_per_pulse, distance);
    }

    return distance;
}

// Function to get distance for right encoder
float get_right_distance()
{
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(right_encoder_queue, &data, 0) == pdTRUE)
    {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        printf("Right Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               data.pulse_count, distance_per_pulse, distance);
    }
    return distance;
}

// Function to get speed for a left encoder
float get_left_speed()
{
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(left_encoder_queue, &current, 0) == pdTRUE)
    {
        if (current.pulse_count != left_last_data.pulse_count)
        {                                                                                  // Only calculate if count changed
            float time_diff = (current.timestamp - left_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - left_last_data.pulse_count;

            if (time_diff > 0)
            {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff; // Speed in cm/s
                printf("Left Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n",
                       speed, count_diff, time_diff);
            }

            left_last_data = current;
        }
        else
        {
            printf("No pulse count change detected for left encoder\n");
        }
    }
    else
    {
        printf("No data available in left encoder queue\n");
    }

    return speed;
}

// Function to get speed for right encoder
float get_right_speed()
{
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(right_encoder_queue, &current, 0) == pdTRUE)
    {
        if (current.pulse_count != right_last_data.pulse_count)
        {                                                                                   // Only calculate if count changed
            float time_diff = (current.timestamp - right_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - right_last_data.pulse_count;

            if (time_diff > 0)
            {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff; // Speed in cm/s
                printf("Right Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n",
                       speed, count_diff, time_diff);
            }

            right_last_data = current;
        }
        else
        {
            printf("No pulse count change detected for right encoder\n");
        }
    }
    else
    {
        printf("No data available in right encoder queue\n");
    }

    return speed;
}

// Reset Left Encoder
void reset_left_encoder()
{
    if (xSemaphoreTake(left_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        left_data.pulse_count = 0;
        left_data.timestamp = 0;
        left_last_data.pulse_count = 0;
        left_last_data.timestamp = 0;
        printf("Left Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(left_data_mutex);

        // Clear the left encoder queue
        xQueueReset(left_encoder_queue);
    }
}

// Reset Right Encoder
void reset_right_encoder()
{
    if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        right_data.pulse_count = 0;
        right_data.timestamp = 0;
        right_last_data.pulse_count = 0;
        right_last_data.timestamp = 0;
        printf("Right Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(right_data_mutex);

        // Clear the right encoder queue
        xQueueReset(right_encoder_queue);
    }
}

void encoder_init()
{
    gpio_init(L_ENCODER_POW);
    gpio_init(L_ENCODER_OUT);

    gpio_set_dir(L_ENCODER_POW, GPIO_OUT);
    gpio_set_dir(L_ENCODER_OUT, GPIO_IN);

    gpio_pull_up(L_ENCODER_OUT);

    left_data.pulse_count = 0;
    left_data.timestamp = 0;

    gpio_init(R_ENCODER_POW);
    gpio_init(R_ENCODER_OUT);

    gpio_set_dir(R_ENCODER_POW, GPIO_OUT);
    gpio_set_dir(R_ENCODER_OUT, GPIO_IN);

    gpio_pull_up(R_ENCODER_OUT);

    right_data.pulse_count = 0;
    right_data.timestamp = 0;

    gpio_put(L_ENCODER_POW, 1);
    gpio_put(R_ENCODER_POW, 1);

    left_encoder_queue = xQueueCreate(1, sizeof(EncoderData));
    right_encoder_queue = xQueueCreate(1, sizeof(EncoderData));

    left_data_mutex = xSemaphoreCreateMutex();
    right_data_mutex = xSemaphoreCreateMutex();

    // Create separate tasks for each encoder
    xTaskCreate(left_encoder_task, "Left Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(right_encoder_task, "Right Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
}

// int main()
// {
// stdio_init_all();

// // Initialise motor GPIO pins and PWM
// motor_init_setup();
// motor_pwm_init();

// // Initialise encoder GPIO pins
// encoder_init();
// printf("Encoder pins initialised\n");
// sleep_ms(500);

// // Set up a timer to generate interrupts every second
// struct repeating_timer timer;
// add_repeating_timer_ms(1000, encoder_set_distance_speed_callback, NULL, &timer);

// while (1) {
//     // Run at half duty cycle
//     move_motor(1563, 1563);
//     sleep_ms(5000);

//     // Turn left at full duty cycle
//     move_motor(3165, 3165);
//     turn_motor(1);
//     sleep_ms(250);

//     // Turn right at full duty cycle
//     move_motor(3165, 3165);
//     turn_motor(0);
//     sleep_ms(250);

//     // Run at 32% duty cycle
//     // move_motor(1000);
//     move_motor(1000, 1000);
//     sleep_ms(5000);
// }

// return 0;
//}
