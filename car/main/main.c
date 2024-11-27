// Main program

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h" // For TCP server and client
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "tcp_server.h"
#include "barcode.h"

#define MAIN_BTN_PIN 20
#define CONDITION_MOTOR_BTN_PIN 21
#define RESET_BARCODE_BTN_PIN 22

int NO_TASK = 0;
int MAIN_TASK = 1;
int CONDITION_MOTOR_TASK = 2;
int RESET_BARCODE_TASK = 3;

// Global variables to track tasks
int current_task = 0;
TaskHandle_t current_task_handle = NULL;
QueueHandle_t button_queue;

#define MAGNETO_MAX_SLICES 3

int rcvd_direction = NEUTRAL;
int rcvd_turn_direction = NEUTRAL;
float rcvd_target_speed = 0.00f;
float rcvd_turn_offset = 0.00f;

void driver_callbacks(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    // Left wheel encoder callback
    case L_ENCODER_OUT:
        read_encoder_pulse(L_ENCODER_OUT, events);
        break;
    // Right wheel encoder callback
    case R_ENCODER_OUT:
        read_encoder_pulse(R_ENCODER_OUT, events);
        break;
    // Ultrasonic callback
    case ECHOPIN:
        read_echo_pulse(ECHOPIN, events);
        break;
    // IR sensor callback
    case IR_SENSOR_PIN:
        gpio_isr_handler(IR_SENSOR_PIN, events);
        break;
    default:
        break;
    }
}

void task_btn_callbacks(uint gpio, uint32_t events)
{
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        switch (gpio)
        {
        // Main task button callback
        case MAIN_BTN_PIN:
            xQueueSendFromISR(button_queue, &MAIN_TASK, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        // Condition motor button callback
        case CONDITION_MOTOR_BTN_PIN:
            xQueueSendFromISR(button_queue, &CONDITION_MOTOR_TASK, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        // Reset barcode button callback
        case RESET_BARCODE_BTN_PIN:
            xQueueSendFromISR(button_queue, &RESET_BARCODE_TASK, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        default:
            break;
        }
    }
}

// Function to init all sensors and motors
void init_drivers()
{
    // Initialise encoder sensor
    encoder_init();
    printf("[MAIN/START] Encoder pins initialised\n");
    sleep_ms(500);

    // Initialise motor pins and PWM
    motor_init();
    printf("[MAIN/START] Motor pins and PWM initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    ultrasonic_init();
    printf("[MAIN/START] Ultrasonic pins initialised\n");
    sleep_ms(500);

    // Initialise barcode sensor
    barcode_init();
    printf("[MAIN/START] Barcode pins initialised\n");
    sleep_ms(500);
}

void init_buttons()
{
    gpio_init(MAIN_BTN_PIN);
    gpio_set_dir(MAIN_BTN_PIN, GPIO_IN);
    gpio_pull_up(MAIN_BTN_PIN);

    gpio_init(CONDITION_MOTOR_BTN_PIN);
    gpio_set_dir(CONDITION_MOTOR_BTN_PIN, GPIO_IN);
    gpio_pull_up(CONDITION_MOTOR_BTN_PIN);

    // gpio_init(RESET_BARCODE_BTN_PIN);
    // gpio_set_dir(RESET_BARCODE_BTN_PIN, GPIO_IN);
    // gpio_pull_up(RESET_BARCODE_BTN_PIN);

    gpio_set_irq_enabled_with_callback(MAIN_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &task_btn_callbacks);
    gpio_set_irq_enabled_with_callback(CONDITION_MOTOR_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &task_btn_callbacks);
    // gpio_set_irq_enabled_with_callback(RESET_BARCODE_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &task_btn_callbacks);

    button_queue = xQueueCreate(10, sizeof(int));
    if (button_queue == NULL)
    {
        printf("[MAIN/START] Failed to create button queue!\n");
        while (1)
            ; // Halt if queue creation fails
    }

    printf("[MAIN/START] Task buttons initialised\n");
}

void init_interrupts()
{
    // Initialise interrupts for needed sensors
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &driver_callbacks);
    printf("[MAIN/START] Interrupts initialised\n");
}

void reset_barcode_task()
{
    printf("Reset button pressed! Resetting barcode data...\n");
    reset_barcode_data();
    vTaskDelay(pdMS_TO_TICKS(500));
}

void motor_conditioning_task()
{
    motor_conditioning();
    current_task = NO_TASK; // Reset global state
    current_task_handle = NULL;
    vTaskDelete(NULL); // Delete the current task
}

void process_magneto_data(int x, int y)
{
    if (y > 0 && y <= MAGNETO_MAX_SLICES)
    {
        rcvd_direction = FORWARDS;
        rcvd_target_speed = 40 + ((MAX_SPEED - MIN_SPEED) * (y / MAGNETO_MAX_SLICES));
    }
    else if (y < 0 && y >= -MAGNETO_MAX_SLICES)
    {
        rcvd_direction = BACKWARDS;
        rcvd_target_speed = 40 + ((MAX_SPEED - MIN_SPEED) * (abs(y) / MAGNETO_MAX_SLICES));
    }
    else
    {
        rcvd_direction = NEUTRAL;
        rcvd_target_speed = 0.0f;
    }

    if (x > 0 && x <= MAGNETO_MAX_SLICES)
    {
        rcvd_turn_direction = RIGHT;
        rcvd_turn_offset = x / MAGNETO_MAX_SLICES;
    }
    else if (x < 0 && x >= -MAGNETO_MAX_SLICES)
    {
        rcvd_turn_direction = LEFT;
        rcvd_turn_offset = abs(x) / MAGNETO_MAX_SLICES;
    }
    else
    {
        rcvd_turn_direction = NEUTRAL;
        rcvd_turn_offset = 0.0f;
    }

    // printf("[MAIN/MAGNETO] Direction: %s\n", (rcvd_direction == FORWARDS) ? "FORWARDS" : (rcvd_direction == BACKWARDS) ? "BACKWARDS" : "NEUTRAL");
    // printf("[MAIN/MAGNETO] Turn direction: %s\n", (rcvd_turn_direction == RIGHT) ? "RIGHT" : (rcvd_turn_direction == LEFT) ? "LEFT" : "NEUTRAL");
    // printf("[MAIN/MAGNETO] Target Speed: %f\n", rcvd_target_speed);
    // printf("[MAIN/MAGNETO] Direction Offset: %f\n", rcvd_turn_offset);
}

int get_tcp_magneto_data()
{
    const char *data = get_tcp_last_rcvd();

    // printf("[MAIN/MAGNETO] Received Data: %s\n", data);

    // Check if the data is non-empty
    if (data != NULL && data[0] != '\0')
    {
        int x = 0, y = 0;
        if (sscanf(data, "X: %*d, Y: %*d, Z: %*d, Fixed_X: %d, Fixed_Y: %d", &x, &y) == 2)
        {
            // printf("[MAIN/MAGNETO] Parsed values - X: %d, Y: %d\n", x, y);
            free((void *)data);
            process_magneto_data(x, y);
            return 1;
        }
        else
        {
            // printf("[MAIN/MAGNETO] Failed to parse values from the data.\n");
        }
    }
    else
    {
        // printf("[MAIN/MAGNETO] No data received yet.\n");
    }

    free((void *)data);

    return 0;
}

void main_task()
{
    init_interrupts();
    init_server();

    printf("[MAIN] Starting test\n");

    stop_motor_manual();
    reset_encoders();

    while (1)
    {
        get_tcp_magneto_data();

        if (rcvd_direction == FORWARDS && get_obstacle_distance() <= OBSTACLE_DISTANCE)
        {
            printf("[MAIN] Obstacle detected at %f cm. Stopping.\n", OBSTACLE_DISTANCE);
            stop_motor_manual();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (rcvd_direction == NEUTRAL && rcvd_turn_direction == NEUTRAL)
        {
            // printf("[MAIN] Stopping\n");
            stop_motor_pid();
        }
        else if (rcvd_direction != NEUTRAL && rcvd_turn_direction == NEUTRAL)
        {
            // printf("[MAIN] Moving straight\n");
            if (rcvd_direction == FORWARDS)
            {
                // printf("[MAIN] Moving forwards\n");
                forward_motor_pid(rcvd_target_speed);
            }
            else if (rcvd_direction == BACKWARDS)
            {
                // printf("[MAIN] Moving backwards\n");
                reverse_motor_pid(rcvd_target_speed);
            }
        }
        else if (rcvd_direction == NEUTRAL && rcvd_turn_direction != NEUTRAL)
        {
            if (rcvd_turn_direction == LEFT)
            {
                // printf("[MAIN] Moving left\n");
                turn_motor_manual(LEFT, CONTINUOUS, PWM_TURN, PWM_TURN);
            }
            else if (rcvd_turn_direction == RIGHT)
            {
                // printf("[MAIN] Moving right\n");
                turn_motor_manual(RIGHT, CONTINUOUS, PWM_TURN, PWM_TURN);
            }
        }
        else
        {
            // printf("[MAIN] Moving straight and turning\n");
            offset_move_motor(rcvd_direction, rcvd_turn_direction, rcvd_turn_offset);
        }

        send_decoded_data_to_server("Hello from Pico!");

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    stop_motor_manual(); // Stop after reaching target distance

    current_task = NO_TASK; // Reset global state
    current_task_handle = NULL;
    vTaskDelete(NULL); // Delete the current task
}

void reset_tasks()
{
    stop_motor_manual();
    vTaskDelete(current_task_handle);
    current_task_handle = NULL;
    current_task = NO_TASK;
    printf("[MAIN/TASK] Previous task stopped.\n");
}

void task_manager()
{
    int selected_task;

    while (1)
    {
        // Wait for a button press signal
        if (xQueueReceive(button_queue, &selected_task, portMAX_DELAY))
        {
            // If the pressed button matches the currently active task
            if (current_task == selected_task)
            {
                // Stop the currently running task
                if (current_task_handle != NULL)
                {
                    reset_tasks();
                }
            }
            else
            {
                // Stop the currently running task
                if (current_task_handle != NULL)
                {
                    reset_tasks();
                }

                // Start the new task
                if (selected_task == MAIN_TASK)
                {
                    xTaskCreate(main_task, "Main Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                }
                else if (selected_task == CONDITION_MOTOR_TASK)
                {
                    xTaskCreate(motor_conditioning_task, "Motor Conditioning", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                }
                // else if (selected_task == RESET_BARCODE_TASK)
                // {
                //     xTaskCreate(reset_barcode_task, "Reset Barcode", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                // }

                current_task = selected_task;
                printf("[MAIN/TASK] Task for button %d started.\n", current_task);
            }
        }
    }
}

int main()
{
    // Initialise standard I/O
    stdio_init_all();
    sleep_ms(1000);

    // Init all required
    init_drivers();
    init_buttons();

    xTaskCreate(task_manager, "Task Manager", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);

    printf("[MAIN/START] Press button GPIO %d to start main task\n", MAIN_BTN_PIN);
    printf("[MAIN/START] Press button GPIO %d to start motor conditioning task\n", CONDITION_MOTOR_BTN_PIN);
    printf("============\n");

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}