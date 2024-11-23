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
            xQueueSendFromISR(button_queue, &MAIN_TASK, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        // Right wheel encoder callback
        case CONDITION_MOTOR_BTN_PIN:
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
    // Initialise encoder sensor
    encoder_init();
    // printf("[MAIN/START] Encoder pins initialised\n");
    sleep_ms(500);

    // Initialise motor pins and PWM
    motor_init();
    // printf("[MAIN/START] Motor pins and PWM initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    ultrasonic_init();
    // printf("[MAIN/START] Ultrasonic pins initialised\n");
    sleep_ms(500);
}

void init_interrupts()
{
    // Initialise interrupts for needed sensors
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
    // printf("[MAIN/START] Interrupts initialised\n");
}

void init_buttons()
{
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
        // printf("[MAIN/START] Failed to create button queue!\n");
        while (1)
            ; // Halt if queue creation fails
    }

    // printf("[MAIN/START] Task buttons initialised\n");
}

void condition_motor()
{
    // printf("[MOTOR/CONDITIONING] Running motor conditioning.\n");
    stop_motor();
    move_motor(PWM_KICKSTART, PWM_KICKSTART);
    sleep_ms(15000);
    // printf("[MOTOR/CONDITIONING] Reversing motor conditioning.\n");
    reverse_motor(PWM_KICKSTART, PWM_KICKSTART);
    sleep_ms(15000);
    stop_motor();
    // printf("[MOTOR/CONDITIONING] Motor conditioning complete.\n");

    current_task = NO_TASK; // Reset global state
    current_task_handle = NULL;
    vTaskDelete(NULL); // Delete the current task
}

void station_1_run()
{
    init_interrupts();

    // printf("[MAIN] Starting test\n");

    // // OFFSET MOTOR UNTIL OBSTCLE
    // float obstacle_distance = 25.0f;
    // int turn_direction = RIGHT;
    // float offset = 0.5;
    // reset_encoders();
    // disable_pid_control();
    // // printf("==========\n");
    // // printf("[MAIN] Offset motor %s until obstacle at %f cm.\n", (turn_direction == LEFT) ? "left" : "right", obstacle_distance);
    // sleep_ms(500);
    // offset_current_motor(turn_direction, offset);
    // while (get_obstacle_distance() > obstacle_distance)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    // }
    // stop_motor(); // Stop after reaching target distance
    // // printf("[MAIN] Obstacle detected at %f cm. Stopping.\n", obstacle_distance);

    // GO STRAIGHT UNTIL OBSTACLE
    float obstacle_distance = 25.0f;
    float target_speed = 30;
    reset_encoders();
    enable_pid_control();
    sleep_ms(500);
    printf("==========\n");
    printf("[MAIN] Started ultrasonic obstacle detection.\n");
    forward_motor_pid(target_speed);
    while (get_obstacle_distance() > obstacle_distance)
    {
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    }
    printf("[MAIN] Obstacle detected at %f cm. Stopping.\n", obstacle_distance);
    stop_motor_pid(); // Stop after reaching target distance

    // TURN AT TARGET ANGLE
    float target_angle = 180.0f;
    int turn_direction = LEFT;
    disable_pid_control();
    for (int i = 0; i < 1; i++)
    {
        reset_encoders();
        sleep_ms(1000);
        // printf("==========\n");
        // printf("[MAIN] Turning %s %f degrees.\n", (turn_direction == LEFT) ? "left" : "right", target_angle);
        turn_motor(turn_direction, target_angle, PWM_TURN, PWM_TURN);
        // printf("[MAIN] Turned %s %f degrees. Stopping.\n", (turn_direction == LEFT) ? "left" : "right", target_angle);
        stop_motor();
    }

    // GO STRAIGHT AT SPEED UNTIL TARGET DISTANCE
    target_speed = 35.0f;
    float target_distance = 100.0f;
    reset_encoders();
    enable_pid_control();
    // printf("==========\n");
    // printf("[MAIN] Doing PID move motor at %f cm/s until %f cm.\n", target_speed, target_distance);
    sleep_ms(500);
    forward_motor_pid(target_speed);
    while (get_average_distance() < target_distance)
    {
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    }
    stop_motor_pid(); // Stop after reaching target distance
    // printf("[MAIN] Reached %f cm. Stopping.\n", target_distance);

    // // MOVE FASTER AND FASTER WITH PID
    // float target_speed = MIN_SPEED;
    // int turn_direction = RIGHT;
    // reset_encoders();
    // enable_pid_control();
    // // printf("==========\n");
    // while (1)
    // {
    //     sleep_ms(100);
    //     // printf("[MAIN] Moving %s continuously at %f cm/s.\n", (turn_direction == LEFT) ? "left" : "right", target_speed);
    //     forward_motor_pid(target_speed);

    //     target_speed = (target_speed < MAX_SPEED) ? target_speed * 1.01f : MAX_SPEED;
    // }
    // stop_motor_pid();

    // // GO STRAIGHT AT SPEED UNTIL TARGET DISTANCE
    // float target_speed = 20.0f;
    // float target_distance = 200.0f;
    // reset_encoders();
    // enable_pid_control();
    // sleep_ms(500);
    // // printf("==========\n");
    // // printf("[MAIN] Doing PID move motor at %f cm/s until %f cm.\n", target_speed, target_distance);
    // forward_motor_pid(target_speed);
    // while (get_average_distance() < target_distance)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
    // }
    // stop_motor_pid(); // Stop after reaching target distance
    // // printf("[MAIN] Reached %f cm. Stopping.\n", target_distance);

    // // TURN CONTINUOUSLY IN ONE DIRECTION
    // int turn_direction = LEFT;
    // reset_encoders();
    // disable_pid_control();
    // sleep_ms(500);
    // // printf("==========\n");
    // // printf("[MAIN] Turning %s continuously.\n", (turn_direction == LEFT) ? "left" : "right");
    // turn_motor(turn_direction, CONTINUOUS, PWM_MAX, PWM_MAX);
    // sleep_ms(2000);
    // stop_motor();
    // // printf("[MAIN] Stopped turning left.\n");

    current_task = NO_TASK; // Reset global state
    current_task_handle = NULL;
    vTaskDelete(NULL); // Delete the current task
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
                    vTaskDelete(current_task_handle);
                    current_task_handle = NULL;
                    current_task = NO_TASK;
                    // printf("[MAIN/TASK] Task for button %d stopped.\n", selected_task);
                }
            }
            else
            {
                // Stop the currently running task
                if (current_task_handle != NULL)
                {
                    vTaskDelete(current_task_handle);
                    current_task_handle = NULL;
                    current_task = NO_TASK;
                    // printf("[MAIN/TASK] Previous task stopped.\n");
                }

                // Start the new task
                if (selected_task == MAIN_TASK)
                {
                    xTaskCreate(station_1_run, "Main Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                }
                else if (selected_task == CONDITION_MOTOR_TASK)
                {
                    xTaskCreate(condition_motor, "Motor Conditioning", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &current_task_handle);
                }

                current_task = selected_task;
                // printf("[MAIN/TASK] Task for button %d started.\n", current_task);
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

    if (start_server() != 0)
    {
        printf("Server startup failed\n");
        return -1;
    }
    if (start_barcode() != 0)
    {
        printf("Barcode startup failed\n");
        return -1;
    }

    xTaskCreate(task_manager, "Task Manager", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}