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
#include "line_following.h"

#define MAIN_BTN_PIN 22
#define CONDITION_MOTOR_BTN_PIN 21
#define LED_PIN 28

int NO_TASK = 0;
int MAIN_TASK = 1;
int CONDITION_MOTOR_TASK = 2;

// Global variables to track tasks
int current_task = 0;
TaskHandle_t current_task_handle = NULL;
QueueHandle_t button_queue;

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
        // Left wheel encoder callback
        case MAIN_BTN_PIN:
            printf("[MAIN/BTN] Main button pressed\n");
            xQueueSendFromISR(button_queue, &MAIN_TASK, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        // Right wheel encoder callback
        case CONDITION_MOTOR_BTN_PIN:
            printf("[MAIN/BTN] Conditioning button pressed\n");
            xQueueSendFromISR(button_queue, &CONDITION_MOTOR_TASK, &xHigherPriorityTaskWoken);
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
    printf("[MAIN/INIT] Starting driver initialization\n");

    // Initialise encoder sensor
    encoder_init();
    printf("[MAIN/INIT] Encoder pins initialized\n");
    sleep_ms(500);

    // Initialise motor pins and PWM
    motor_init();
    motor_pwm_init(); // Added PWM initialization
    printf("[MAIN/INIT] Motor pins and PWM initialized\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    ultrasonic_init();
    printf("[MAIN/INIT] Ultrasonic pins initialized\n");
    sleep_ms(500);

    // Initialize barcode scanning
    start_barcode();
    printf("[MAIN/INIT] Barcode pins initialized\n");
    sleep_ms(500);

    // start_server();
    // printf("[MAIN/INIT] TCP server initialized\n");
    // sleep_ms(500);

    // Initialize line sensor
    init_line_sensor();
    printf("[MAIN/INIT] Line sensor initialized\n");
    sleep_ms(500);

    // Seed random number generator with current time
    srand(time_us_64());
    printf("[MAIN/INIT] Random number generator seeded\n");

    printf("[MAIN/INIT] All drivers initialized successfully\n");
}

void init_interrupts()
{
    printf("[MAIN/INIT] Starting interrupt initialization\n");

    // Initialise interrupts for needed sensors
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);

    printf("[MAIN/INIT] Interrupts initialized successfully\n");
}

void init_buttons()
{
    printf("[MAIN/INIT] Starting button initialization\n");

    gpio_init(MAIN_BTN_PIN);
    gpio_set_dir(MAIN_BTN_PIN, GPIO_IN);
    gpio_pull_up(MAIN_BTN_PIN);

    gpio_init(CONDITION_MOTOR_BTN_PIN);
    gpio_set_dir(CONDITION_MOTOR_BTN_PIN, GPIO_IN);
    gpio_pull_up(CONDITION_MOTOR_BTN_PIN);

    gpio_set_irq_enabled_with_callback(MAIN_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &task_btn_callbacks);
    gpio_set_irq_enabled_with_callback(CONDITION_MOTOR_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &task_btn_callbacks);

    button_queue = xQueueCreate(10, sizeof(int));
    if (button_queue == NULL)
    {
        printf("[MAIN/INIT] ERROR: Failed to create button queue!\n");
        while (1)
            ; // Halt if queue creation fails
    }

    printf("[MAIN/INIT] Buttons initialized successfully\n");
}

void condition_motor()
{
    printf("[MOTOR/CONDITIONING] Starting motor conditioning\n");
    stop_motor();
    move_motor(PWM_KICKSTART, PWM_KICKSTART);
    sleep_ms(15000);
    printf("[MOTOR/CONDITIONING] Reversing motor conditioning\n");
    reverse_motor(PWM_KICKSTART, PWM_KICKSTART);
    sleep_ms(15000);
    stop_motor();
    printf("[MOTOR/CONDITIONING] Motor conditioning complete\n");

    current_task = NO_TASK; // Reset global state
    current_task_handle = NULL;
    vTaskDelete(NULL); // Delete the current task
}

void line_following()
{
    printf("[MAIN/LINE] Starting line following initialization\n");

    init_interrupts();
    printf("[MAIN/LINE] Interrupts initialized for line following\n");

    // Create the line following task
    TaskHandle_t lineFollowHandle = NULL;
    printf("[MAIN/LINE] Creating line following task\n");

    BaseType_t result = xTaskCreate(lineFollowTask, "Line Following", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &lineFollowHandle);

    if (result != pdPASS)
    {
        printf("[MAIN/LINE] ERROR: Failed to create line following task\n");
        current_task = NO_TASK;
        current_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    printf("[MAIN/LINE] Line following task created successfully\n");

    // Wait until the task is deleted or interrupted
    printf("[MAIN/LINE] Entering monitoring loop\n");
    while (current_task == MAIN_TASK)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clean up if task is interrupted
    if (lineFollowHandle != NULL)
    {
        printf("[MAIN/LINE] Cleaning up line following task\n");
        vTaskDelete(lineFollowHandle);
    }

    printf("[MAIN/LINE] Line following task cleanup complete\n");
    current_task = NO_TASK;
    current_task_handle = NULL;
    vTaskDelete(NULL);
}

void task_manager()
{
    int selected_task;
    printf("[MAIN/TASK] Task manager started\n");

    while (1)
    {
        // Wait for a button press signal
        if (xQueueReceive(button_queue, &selected_task, portMAX_DELAY))
        {
            printf("[MAIN/TASK] Received task selection: %d\n", selected_task);

            // If the pressed button matches the currently active task
            if (current_task == selected_task)
            {
                printf("[MAIN/TASK] Stopping current task: %d\n", current_task);
                // Stop the currently running task
                if (current_task_handle != NULL)
                {
                    vTaskDelete(current_task_handle);
                    current_task_handle = NULL;
                    current_task = NO_TASK;
                    printf("[MAIN/TASK] Task %d stopped\n", selected_task);
                }
            }
            else
            {
                // Stop the currently running task
                if (current_task_handle != NULL)
                {
                    printf("[MAIN/TASK] Stopping previous task: %d\n", current_task);
                    vTaskDelete(current_task_handle);
                    current_task_handle = NULL;
                    current_task = NO_TASK;
                    printf("[MAIN/TASK] Previous task stopped\n");
                }

                // Start the new task
                if (selected_task == MAIN_TASK)
                {
                    xTaskCreate(line_following, "Main Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                }
                else if (selected_task == CONDITION_MOTOR_TASK)
                {
                    xTaskCreate(condition_motor, "Motor Conditioning", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                }

                current_task = selected_task;
                printf("[MAIN/TASK] Task %d started\n", current_task);
            }
        }
    }
}

int main()
{
    // Initialise standard I/O
    stdio_init_all();
    sleep_ms(1000);
    printf("\n[MAIN] Starting robot program...\n");

    // Init all required
    init_drivers();
    init_buttons();

    printf("[MAIN] Creating task manager\n");
    xTaskCreate(task_manager, "Task Manager", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);

    printf("[MAIN] Starting FreeRTOS scheduler\n");
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    printf("[MAIN] ERROR: Scheduler failed to start!\n");
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}
