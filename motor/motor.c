#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "../encoder/encoder.h" 

/* Remove these after Vignesh ENCODER DONE
float get_left_speed() {
    // Temporary stub, replace with actual implementation later
    return 0.0f;
}
float get_right_speed() {
    // Temporary stub, replace with actual implementation later
    return 0.0f;
}
float get_left_distance() {
    // Temporary stub, replace with actual implementation later
    return 0.0f;
}
float get_right_distance() {
    // Temporary stub, replace with actual implementation later
    return 0.0f;
}
bool black_line_detected = false; 
float get_simulated_speed(float target_speed);*/


// Global motor configurations
static MotorConfig left_motor = {L_MOTOR_ENA, L_MOTOR_IN1, L_MOTOR_IN2, 0.0f, 0.0f, 0.0f, 0.0f};
static MotorConfig right_motor = {R_MOTOR_ENB, R_MOTOR_IN3, R_MOTOR_IN4, 0.0f, 0.0f, 0.0f, 0.0f};

//Mutex for PID
static SemaphoreHandle_t pid_mutex;

static PIDState left_speed_pid = {0.0f, 0.0f};
static PIDState right_speed_pid = {0.0f, 0.0f};

// Global variable to store the current movement direction
static MovementDirection current_movement = STOP;

// Global variable to store if car is currently turning.
bool turning_active = false;

// Global variable to store latest duty cycle set to control speed
static float current_duty_cycle = 0.0;

// Global variable to store status of CONTROL_MOTOR_ON_LINE
volatile int stop_running = 0;

// Function to set PWM for a motor
void set_motor_pwm(uint gpio, float duty_cycle, float freq) {
    current_duty_cycle = duty_cycle;
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    float clock_freq = 125000000.0f;  // System clock frequency
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    
    printf("Setting PWM on GPIO %d with duty cycle %.2f\n", gpio, duty_cycle);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), duty_cycle * 65535);

    pwm_set_enabled(slice_num, true);
}

// PID calculation function
float calculate_pid(float set_point, float current_value, PIDState* pid_state) {
    float error = set_point - current_value;
    printf("Calculating PID: Set Point = %.2f, Current Value = %.2f, Error = %.2f\n", set_point, current_value, error);

    // Proportional term
    float p_term = (KP * error);

    // Integral term
    float tmp_integral = pid_state->integral + error;
    float i_term = KI * tmp_integral;

    // Derivative term
    float d_term = KD * (error - pid_state->prev_error);

    // Update previous error
    pid_state->prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;
    // output = scaleToPWM(output);

    float corrected_speed = current_duty_cycle += output;

    if (corrected_speed< MAX_DUTY_CYCLE && corrected_speed > MIN_DUTY_CYCLE) {
        pid_state->integral = tmp_integral;  // Update the integrator if within bounds to prevent integrator windup
    } else if (corrected_speed > MAX_DUTY_CYCLE) {
       corrected_speed = MAX_DUTY_CYCLE;
    } else if (corrected_speed < MIN_DUTY_CYCLE) {
        corrected_speed = MIN_DUTY_CYCLE;
    }

    printf("PID output = %.2f (P: %.2f, I: %.2f, D: %.2f)\n", corrected_speed, p_term, i_term, d_term);
    return corrected_speed;
}

// Control motor forward and backward
void control_motor_forward_backward(MotorConfig* motor, bool forward, float target_speed) {
    printf("Controlling motor on PWM pin %d - %s\n", motor->ena_pin, (motor == &left_motor) ? "Left motor" : "Right motor");

    gpio_put(motor->in1_pin, !forward);
    printf("Set GPIO %d to %d\n", motor->in1_pin, !forward);
    gpio_put(motor->in2_pin, forward);
    printf("Set GPIO %d to %d\n", motor->in2_pin, forward);

    motor->target_speed = target_speed;

    // Obtain current speed from encoder
    motor->current_speed = (motor == &left_motor) ? get_left_speed() : get_right_speed();
    printf("Current speed from encoder: %.2f\n", motor->current_speed);
    
    set_motor_pwm(motor->ena_pin, MAX_DUTY_CYCLE, 1000.0f);
}



/* Task that aligns car based on line detection
void control_motor_on_line_task(void *pvParameters) {
    while (1) {
        // Check if line following mode is active
        if (current_movement == MOTOR_ON_LINE) {
            // Check the current line detection status
            if (black_line_detected) {
                // Line is detected; move slightly to the right to stay on the line
                set_motor_pwm(left_motor.ena_pin, MAX_LINE_DUTY_CYCLE, 256.0f);
                set_motor_pwm(right_motor.ena_pin, MIN_LINE_DUTY_CYCLE, 256.0f);
                vTaskDelay(pdMS_TO_TICKS(6000)); // Larger correction time as car keeps veering towards the left
            } else {
                // Line not detected; move slightly to the left to search for the line
                set_motor_pwm(left_motor.ena_pin, MIN_LINE_DUTY_CYCLE, 256.0f);
                set_motor_pwm(right_motor.ena_pin, MAX_LINE_DUTY_CYCLE, 256.0f);
                vTaskDelay(pdMS_TO_TICKS(200));

                // If line still not detected; move to the right to search for the line
                if (!black_line_detected) {
                    set_motor_pwm(left_motor.ena_pin, MAX_LINE_DUTY_CYCLE, 256.0f);
                    set_motor_pwm(right_motor.ena_pin, MIN_LINE_DUTY_CYCLE, 256.0f);
                    vTaskDelay(pdMS_TO_TICKS(6000)); // // Larger correction time as car keeps veering towards the left
                }  
            }
            stop_running += 1;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Short delay for responsive line checking
    }
}*/

// Helper function to calculate angle when performing pivot turns
float get_angle_turned_pivot() {
    // Implement an approximate calculation for pivot turn angle
    float wheel_base = 10.0f; // Distance between wheels in cm, adjust based on robot dimensions
    float left_dist = fabs(get_left_distance());  // Take absolute values to sum distances
    float right_dist = fabs(get_right_distance());
    printf("Left Distance: %f", left_dist);
    printf("Right Distance: %f", right_dist);

    // Calculate the angle turned in degrees using the sum of wheel distances
    float angle = (left_dist + right_dist) / wheel_base * (180.0f / M_PI);
    printf("Pivot turn angle turned: %.2f degrees\n", angle);
    return angle;
}

// Task to control the motor during turning
void turn_control_task(void *pvParameters) {
    float target_angle = *(float *)pvParameters;
    MovementDirection direction = current_movement;
    
    // Turn until the angle is reached (+-5.0) 
    float current_angle = 0.0f;
    while (current_angle < target_angle - 5.0f || current_angle > target_angle + 5.0f) {  // Keep adjusting until close to target angle
        if ((current_angle - target_angle) > 5.0f) {
            break;
        } 
        current_angle = get_angle_turned_pivot(); // Update current angle

        if (direction == TURN_RIGHT) {
            // Apply reverse and forward direction to left and right motor respectively for right turn
            gpio_put(left_motor.in1_pin, 0);
            gpio_put(left_motor.in2_pin, 1);
            gpio_put(right_motor.in1_pin, 1);
            gpio_put(right_motor.in2_pin, 0);
            
            set_motor_pwm(left_motor.ena_pin, MAX_DUTY_CYCLE, 1000.0f);
            set_motor_pwm(right_motor.ena_pin, MAX_DUTY_CYCLE, 1000.0f);
                
        } else if (direction == TURN_LEFT) {
            // Apply forward and reverse direction to left and right motor respectively for left turn
            gpio_put(left_motor.in1_pin, 1);
            gpio_put(left_motor.in2_pin, 0);
            gpio_put(right_motor.in1_pin, 0);
            gpio_put(right_motor.in2_pin, 1);

            set_motor_pwm(left_motor.ena_pin, MAX_DUTY_CYCLE, 1000.0f); 
            set_motor_pwm(right_motor.ena_pin, MAX_DUTY_CYCLE, 1000.0f); 
        }

        // Update current angle based on encoder data
        current_angle = get_angle_turned_pivot();
        
        vTaskDelay(pdMS_TO_TICKS(2.5));  // Delay to let motors adjust
    }
    
    // Reset turning flag and clean up
    turning_active = false;
    vPortFree(pvParameters); // Free allocated memory for angle
    vTaskDelete(NULL); // Delete this task
}

// Control motor turning Left or Right
void control_motor_turn(float target_angle) {
    if (!turning_active) {
        // Set turning_active to true to indicate tasks are running
        turning_active = true;
        // Allocate memory for passing target angle and direction
        float *angle_param = pvPortMalloc(sizeof(float));
        if (angle_param == NULL) {
            printf("Failed to allocate memory for angle parameter\n");
            return;
        }
        *angle_param = target_angle;
         // Start the turn control task with the angle and direction as parameters
        if (xTaskCreate(turn_control_task, "Turn Control", configMINIMAL_STACK_SIZE, (void *)angle_param, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
            printf("Failed to create Turn Control task\n");
            vPortFree(angle_param);
            turning_active = false;
        }
    }   
}

// Disable all pins to stop motor
void stop_motor() {
    gpio_put(left_motor.ena_pin, 0);
    gpio_put(right_motor.ena_pin, 0);
    gpio_put(left_motor.in1_pin, 0);
    gpio_put(left_motor.in2_pin, 0);
    gpio_put(right_motor.in1_pin, 0);
    gpio_put(right_motor.in2_pin, 0);
}

// Unified movement function
void move_car(MovementDirection direction, float speed, float angle) {
    current_movement = direction; // Update current direction
    printf("Moving car - Direction: %d, Speed: %.2f\n", direction, speed);
    switch(direction) {
        case FORWARD:
            control_motor_forward_backward(&left_motor, true, speed);
            control_motor_forward_backward(&right_motor, true, speed);
            break;

        case BACKWARD:
            control_motor_forward_backward(&left_motor, false, speed);
            control_motor_forward_backward(&right_motor, false, speed);
            break;

        case TURN_LEFT:
            control_motor_turn(angle);
            break;

        case TURN_RIGHT:
            control_motor_turn(angle);
            break;
        
        case MOTOR_ON_LINE:
            control_motor_forward_backward(&left_motor, true, speed);
            control_motor_forward_backward(&right_motor, true, speed);
            break;

        case STOP:
            stop_motor();
            break;
    }
}

// PID control task that stabilse car when moving forward USE THIS WHEN ENCODER DONE VIGNESH
void pid_update_task(void *pvParameters) {
    while (1) {
        if (current_movement == FORWARD) {
            // Only adjust if the target speed is greater than zero
            if (left_motor.target_speed > 0 || right_motor.target_speed > 0) {
                // Update the PID output for left motor
                float left_current_speed = get_left_speed();
                left_motor.current_speed = left_current_speed;
                if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                    float left_pwm_output = calculate_pid(left_motor.target_speed, left_current_speed, &left_speed_pid);
                    set_motor_pwm(left_motor.ena_pin, left_pwm_output, 1000.0f);
                    xSemaphoreGive(pid_mutex);
                }

                // Update the PID output for right motor
                float right_current_speed = get_right_speed();
                right_motor.current_speed = right_current_speed;
                if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                    float right_pwm_output = calculate_pid(right_motor.target_speed, right_current_speed, &right_speed_pid);
                    set_motor_pwm(right_motor.ena_pin, right_pwm_output, 1000.0f);
                    xSemaphoreGive(pid_mutex);
                }
            } 
        } 
        
        else {
            // If the robot is stopped, skip PID calculations and wait before checking again
            vTaskDelay(pdMS_TO_TICKS(500)); // Adjust the delay as needed
        }   
            
        vTaskDelay(pdMS_TO_TICKS(30)); // Adjust delay as needed for responsiveness
    }
} 


// Initialization function
void init_motor() {
    // Initialize GPIO for motors
    gpio_init(L_MOTOR_IN1);
    gpio_init(L_MOTOR_IN2);
    gpio_set_dir(L_MOTOR_IN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_IN2, GPIO_OUT);

    gpio_init(R_MOTOR_IN3);
    gpio_init(R_MOTOR_IN4);
    gpio_set_dir(R_MOTOR_IN3, GPIO_OUT);
    gpio_set_dir(R_MOTOR_IN4, GPIO_OUT);
    
    
    // Create mutex for PID calculations
    pid_mutex = xSemaphoreCreateMutex();

    // Create PID update task for maintaining speed alignment between wheels
    xTaskCreate(pid_update_task, "PID Update", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    //xTaskCreate(control_motor_on_line_task, "Control Motor on Line", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL); integrate with hakam
}



void test_motor_task(void *pvParameters) {
    // Wait for system initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Starting motor tests with PID feedback...\n");

    // Test moving forward with both motors
    printf("Testing forward movement with both motors...\n");
    move_car(FORWARD, 0.5, 0); // Move forward at 50% speed
    vTaskDelay(pdMS_TO_TICKS(3000)); // Run for 3 seconds

    // Check encoder feedback
    float left_speed = get_left_speed();
    float right_speed = get_right_speed();
    printf("Left Speed after forward test: %.2f cm/s\n", left_speed);
    printf("Right Speed after forward test: %.2f cm/s\n", right_speed);

    // Stop the motor and let it stabilize
    move_car(STOP, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second

    // Test moving backward with both motors
    printf("Testing backward movement with both motors...\n");
    move_car(BACKWARD, 0.5, 0); // Move backward at 50% speed
    vTaskDelay(pdMS_TO_TICKS(3000)); // Run for 3 seconds

    // Check encoder feedback
    left_speed = get_left_speed();
    right_speed = get_right_speed();
    printf("Left Speed after backward test: %.2f cm/s\n", left_speed);
    printf("Right Speed after backward test: %.2f cm/s\n", right_speed);

    // Stop the motor
    move_car(STOP, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 1 second

    // Test moving only the left motor forward
    printf("Testing forward movement with only the left motor...\n");
    control_motor_forward_backward(&left_motor, true, 0.5); // Move left motor forward at 50% speed
    vTaskDelay(pdMS_TO_TICKS(3000)); // Run for 3 seconds

    // Check left encoder feedback
    left_speed = get_left_speed();
    printf("Left Speed after single motor forward test: %.2f cm/s\n", left_speed);

    // Stop the left motor
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second

    // Test moving only the right motor forward
    printf("Testing forward movement with only the right motor...\n");
    control_motor_forward_backward(&right_motor, true, 0.5); // Move right motor forward at 50% speed
    vTaskDelay(pdMS_TO_TICKS(3000)); // Run for 3 seconds

    // Check right encoder feedback
    right_speed = get_right_speed();
    printf("Right Speed after single motor forward test: %.2f cm/s\n", right_speed);

    // Stop the right motor
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second

    // End of test
    printf("Motor tests completed.\n");
    vTaskDelete(NULL);
}



// Entry point function for the program
int main() {
    // Initialize standard I/O for debugging
    stdio_init_all();

    // Initialize motors and set up FreeRTOS tasks
    init_motor();

    // Create a FreeRTOS task for motor testing with simulated PID
    xTaskCreate(test_motor_task, "Test Motor Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The program should not reach here under normal circumstances
    while (1) {
    }
}

