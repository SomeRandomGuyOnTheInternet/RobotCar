#include "line_following.h"

// Idle task memory allocation
static StaticTask_t idleTaskTCB;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];


void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &idleTaskTCB;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

// Timer task memory allocation
static StaticTask_t timerTaskTCB;
static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &timerTaskTCB;
    *ppxTimerTaskStackBuffer = timerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// Initialize line sensor using ADC
void init_line_sensor()
{
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // Select ADC input 2 for GPIO28
}

// Turn motor function
void line_follow_turn_motor(int direction, uint64_t steer_duration)
{
    disable_pid_control();
    
    // Convert direction from 0/1 to LEFT/RIGHT constants
    int motor_direction = (direction == 0) ? LEFT : RIGHT;
    
    // Use turn_motor_manual with continuous turning and fixed PWM values
    turn_motor_manual(motor_direction, CONTINUOUS, 2000, 2000);

    uint64_t start_time = time_us_64();
    uint64_t max_turn_duration = steer_duration * 1000; // Convert ms to microseconds

    while ((time_us_64() - start_time) < max_turn_duration)
    {
        uint16_t sensor_value = adc_read();
        if (sensor_value > LINE_THRESHOLD)
        {
            printf("Line detected during turn. Exiting turn early. Sensor value: %d\n", sensor_value);
            stop_motor_pid();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motor_pid();
    vTaskDelay(pdMS_TO_TICKS(50));
}

void lineFollowTask(void *pvParameters)
{
    uint64_t last_transition_time = 0;
    int initial_turn_direction = 1;                   
    int alternate_turn_direction = 1 - initial_turn_direction; // Opposite direction for the second turn
    bool needs_second_turn = false;
    uint64_t initial_steer_duration = STEER_DURATION;
    uint64_t second_steer_duration = initial_steer_duration + 20;
    int consecutive_reversals = 0;
    int non_reversal_turns = 0; // Counts turns after max consecutive reversals reached

    printf("Line follow task started. Initial turn direction: %s\n", initial_turn_direction == 0 ? "Left" : "Right");

    while (true)
    {
        if (get_obstacle_distance() <= OBSTACLE_DISTANCE) {
            printf("[MAIN] Obstacle detected during line following at %f cm. Stopping.\n", OBSTACLE_DISTANCE);
            stop_motor_manual();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        uint64_t current_time = time_us_64();
        uint16_t sensor_value = adc_read();

        printf("Sensor value: %d\n", sensor_value);

        if ((current_time - last_transition_time) > (DEBOUNCE_DELAY_MS * 1000))
        {
            last_transition_time = current_time;

            if (sensor_value > LINE_THRESHOLD)
            {
                printf("Line detected. Moving forward.\n");
                forward_motor_pid(MAX_SPEED);

                // Reset steering durations, turn directions, and counters on line detection
                initial_turn_direction = 1; // Randomize the initial turn again
                alternate_turn_direction = 1 - initial_turn_direction;
                initial_steer_duration = STEER_DURATION;
                second_steer_duration = initial_steer_duration + 20;
                consecutive_reversals = 0;
                non_reversal_turns = 0;
                needs_second_turn = false; // Reset the need for a second turn
            }
            else
            {
                if (!needs_second_turn)
                {
                    // First turn
                    printf("Out of course. Initiating first turn: %s\n", initial_turn_direction == 0 ? "Left" : "Right");

                    if (consecutive_reversals < 3 || non_reversal_turns >= 3)
                    {
                        // Reverse if we haven't reached max reversals, or if we've done 3 turns without reversing
                        stop_motor_pid();
                        reverse_motor_manual(2500, 2500);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        if (consecutive_reversals < 3)
                        {
                            consecutive_reversals++;
                        }
                    }
                    else
                    {
                        // After 3 consecutive reversals, do not reverse, only increase steer duration
                        printf("Max consecutive reversals reached. Increasing steer duration.\n");
                    }

                    line_follow_turn_motor(initial_turn_direction, initial_steer_duration);
                    needs_second_turn = true; // Flag to perform the second turn after this
                }
                else
                {
                    // Second turn
                    printf("Performing second turn to counter back: %s\n", alternate_turn_direction == 0 ? "Left" : "Right");

                    if (consecutive_reversals < 3 || non_reversal_turns >= 3)
                    {
                        stop_motor_pid();
                        reverse_motor_manual(2200, 2200);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        if (consecutive_reversals < 3)
                        {
                            consecutive_reversals++;
                        }
                    }
                    else
                    {
                        // No further reversal; only increase steer duration
                        printf("Max consecutive reversals reached. Increasing steer duration.\n");
                    }

                    line_follow_turn_motor(alternate_turn_direction, second_steer_duration);
                    needs_second_turn = false; // Reset after the second turn
                }

                // Increase the steer duration if maximum reversals have been reached and count non-reversal turns
                if (consecutive_reversals >= 3)
                {
                    initial_steer_duration += 50;
                    second_steer_duration = initial_steer_duration + 40;
                    non_reversal_turns++;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// int main() {
//     stdio_init_all();
//     printf("Starting robot program...\n");

//     init_line_sensor();
//     printf("Line sensor initialized.\n");

//     motor_init();
//     motor_pwm_init();
//     printf("Motors initialized.\n");

//     // Seed the random number generator
//     srand(time_us_64()); // Seed with microsecond time for better randomness

//     // Create line following task
//     xTaskCreate(lineFollowTask, "Line Follow Task", 1024, NULL, 1, NULL);

//     printf("Starting FreeRTOS scheduler...\n");
//     vTaskStartScheduler();

//     printf("Scheduler failed to start!\n");
//     while (true) {
//         // Error handling if scheduler fails
//     }

//     return 0;
// }
