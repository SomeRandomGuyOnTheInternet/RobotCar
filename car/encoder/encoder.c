#include "encoder.h"

// Global encoder objects
static Encoder left_encoder = {
    .data = {0, 0},
    .last_data = {0, 0},
    .mutex = NULL};

static Encoder right_encoder = {
    .data = {0, 0},
    .last_data = {0, 0},
    .mutex = NULL};

// ISR for handling encoder pulses
void read_encoder_pulse(uint gpio, uint32_t events)
{
    Encoder *encoder = NULL;

    if (gpio == L_ENCODER_OUT)
    {
        encoder = (Encoder *)&left_encoder;
    }
    else if (gpio == R_ENCODER_OUT)
    {
        encoder = (Encoder *)&right_encoder;
    }

    if (encoder != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(encoder->mutex, &xHigherPriorityTaskWoken) == pdTRUE)
        {
            encoder->last_data.pulse_count = encoder->data.pulse_count;
            encoder->last_data.timestamp = encoder->data.timestamp;
            encoder->data.pulse_count++;
            encoder->data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(encoder->mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Generalized distance calculation
float get_distance(Encoder *encoder)
{
    EncoderData data = encoder->data;
    float distance = 0.0f;

    if (xSemaphoreTake(encoder->mutex, portMAX_DELAY) == pdTRUE)
    {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        xSemaphoreGive(encoder->mutex);
    }

    return distance;
}

// Distance functions
float get_left_distance()
{
    float distance = get_distance((Encoder *)&left_encoder);
    // printf("[DISTANCE] Left distance: %f\n", distance);
    return distance;
}

float get_right_distance()
{
    float distance = get_distance((Encoder *)&right_encoder);
    // printf("[DISTANCE] Right distance: %f\n", distance);
    return distance;
}

float get_average_distance()
{
    float left_distance = get_left_distance();
    float right_distance = get_right_distance();
    float average_distance = (left_distance + right_distance) / 2.0f;
    // printf("[DISTANCE] Average distance: %f\n", average_distance);
    return average_distance;
}

// Generalized speed calculation
float get_speed(Encoder *encoder)
{
    EncoderData data = encoder->data;
    EncoderData last_data = encoder->last_data;
    float speed = 0.0f;

    if (xSemaphoreTake(encoder->mutex, portMAX_DELAY) == pdTRUE)
    {
        double count_diff = data.pulse_count - last_data.pulse_count;
        double time_diff = ((int64_t)data.timestamp - (int64_t)last_data.timestamp) / 1000000.0f;
        double now_time_diff = (double)(time_us_64() - data.timestamp) / 1000000.0f;
        
        // printf("[SPEED] Count diff: %f, Time diff: %f, Now time diff: %f\n", count_diff, time_diff, now_time_diff);

        if (time_diff > 0.0f && now_time_diff < 1.0f)
        {
            if (count_diff > 0.0f)
            {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff; // Speed in cm/s
            }
        }

        xSemaphoreGive(encoder->mutex);
    }

    return speed;
}

// Speed functions
float get_left_speed()
{
    float speed = get_speed((Encoder *)&left_encoder);
    if (speed != INVALID_SPEED)
    {
        // printf("[SPEED] Left speed: %f\n", speed);
    }
    return speed;
}

float get_right_speed()
{
    float speed = get_speed((Encoder *)&right_encoder);
    if (speed != INVALID_SPEED)
    {
        // printf("[SPEED] Right speed: %f\n", speed);
    }
    return speed;
}

float get_average_speed()
{
    float left_speed = get_left_speed();
    float right_speed = get_right_speed();
    float average_speed = (left_speed + right_speed) / 2.0f;
    if (average_speed > INVALID_SPEED)
    {
        // printf("[SPEED] Average speed: %f\n", average_speed);
    }
    return average_speed;
}

// Generalized encoder reset
void reset_encoder(Encoder *encoder)
{
    if (xSemaphoreTake(encoder->mutex, portMAX_DELAY) == pdTRUE)
    {
        encoder->data.pulse_count = 0;
        encoder->data.timestamp = 0;
        xSemaphoreGive(encoder->mutex);
    }
}

// Reset functions
void reset_left_encoder()
{
    reset_encoder((Encoder *)&left_encoder);
}

void reset_right_encoder()
{
    reset_encoder((Encoder *)&right_encoder);
}

void reset_encoders()
{
    reset_left_encoder();
    reset_right_encoder();
}

// Initialize encoders
void encoder_init()
{
    gpio_init(L_ENCODER_POW);
    gpio_init(L_ENCODER_OUT);
    gpio_set_dir(L_ENCODER_POW, GPIO_OUT);
    gpio_set_dir(L_ENCODER_OUT, GPIO_IN);
    gpio_pull_up(L_ENCODER_OUT);

    gpio_init(R_ENCODER_POW);
    gpio_init(R_ENCODER_OUT);
    gpio_set_dir(R_ENCODER_POW, GPIO_OUT);
    gpio_set_dir(R_ENCODER_OUT, GPIO_IN);
    gpio_pull_up(R_ENCODER_OUT);

    gpio_put(L_ENCODER_POW, 1);
    gpio_put(R_ENCODER_POW, 1);

    left_encoder.mutex = xSemaphoreCreateMutex();
    right_encoder.mutex = xSemaphoreCreateMutex();
}
