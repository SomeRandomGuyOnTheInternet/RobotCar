#include "encoder.h"

// Global encoder objects
static Encoder left_encoder = {
    .data = {0, 0},
    .mutex = NULL,
    .queue = NULL
};

static Encoder right_encoder = {
    .data = {0, 0},
    .mutex = NULL,
    .queue = NULL
};

// ISR for handling encoder pulses
void read_encoder_pulse(uint gpio, uint32_t events) {
    Encoder *encoder = NULL;

    if (gpio == L_ENCODER_OUT) {
        encoder = (Encoder *)&left_encoder;
    } else if (gpio == R_ENCODER_OUT) {
        encoder = (Encoder *)&right_encoder;
    }

    if (encoder != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(encoder->mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            encoder->data.pulse_count++;
            encoder->data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(encoder->mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Task to handle encoder updates
void encoder_task(void *params) {
    Encoder *encoder = (Encoder *)params;
    EncoderData last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(encoder->mutex, portMAX_DELAY) == pdTRUE) {
            EncoderData data = encoder->data;
            xSemaphoreGive(encoder->mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count) {
                xQueueReset(encoder->queue);
                if (xQueueSendToBack(encoder->queue, &data, 0) == pdTRUE) {
                    last_sent = data;
                } else {
                    printf("Encoder queue send failed\n");
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10)); // Adjust frequency as needed
    }
}

// Generalized distance calculation
float get_distance(Encoder *encoder) {
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(encoder->queue, &data, 0) == pdTRUE) {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
    }
    return distance;
}

// Distance functions
float get_left_distance() {
    return get_distance((Encoder *)&left_encoder);
}

float get_right_distance() {
    return get_distance((Encoder *)&right_encoder);
}

float get_average_distance() {
    return (get_left_distance() + get_right_distance()) / 2.0f;
}

// Generalized speed calculation
float get_speed(Encoder *encoder, EncoderData *last_data) {
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(encoder->queue, &current, 0) == pdTRUE) {
        if (current.pulse_count != last_data->pulse_count) {
            float time_diff = (current.timestamp - last_data->timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - last_data->pulse_count;

            if (time_diff > 0) {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff; // Speed in cm/s
            }

            *last_data = current;
        }
    }
    return speed;
}

// Speed functions
float get_left_speed() {
    static EncoderData left_last_data = {0, 0};
    return get_speed((Encoder *)&left_encoder, &left_last_data);
}

float get_right_speed() {
    static EncoderData right_last_data = {0, 0};
    return get_speed((Encoder *)&right_encoder, &right_last_data);
}

float get_average_speed() {
    return (get_left_speed() + get_right_speed()) / 2.0f;
}

// Generalized encoder reset
void reset_encoder(Encoder *encoder) {
    if (xSemaphoreTake(encoder->mutex, portMAX_DELAY) == pdTRUE) {
        encoder->data.pulse_count = 0;
        encoder->data.timestamp = 0;
        xQueueReset(encoder->queue);
        xSemaphoreGive(encoder->mutex);
    }
}

// Reset functions
void reset_left_encoder() {
    reset_encoder((Encoder *)&left_encoder);
}

void reset_right_encoder() {
    reset_encoder((Encoder *)&right_encoder);
}

void reset_encoders() {
    reset_left_encoder();
    reset_right_encoder();
}

// Initialize encoders
void encoder_init() {
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
    left_encoder.queue = xQueueCreate(1, sizeof(EncoderData));

    right_encoder.mutex = xSemaphoreCreateMutex();
    right_encoder.queue = xQueueCreate(1, sizeof(EncoderData));

    xTaskCreate(encoder_task, "Left Encoder Task", configMINIMAL_STACK_SIZE * 4, (void *)&left_encoder, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(encoder_task, "Right Encoder Task", configMINIMAL_STACK_SIZE * 4, (void *)&right_encoder, tskIDLE_PRIORITY + 1, NULL);
}
