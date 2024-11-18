#include "motor.h"

// PID parameters
float Kp = 0.009f;
float Ki = 0.002f;
float Kd = 0.006f;

// PID control variables
float integral_left = 0.0;
float integral_right = 0.0;
float prev_error_left = 0.0;
float prev_error_right = 0.0;

static bool use_pid_control = false;
static float target_speed = 15.0;

// Function to move motors forward
void move_motor(float new_pwm_left, float new_pwm_right)
{
    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), (uint16_t)new_pwm_left);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), (uint16_t)new_pwm_right);

    // Set motor directions
    gpio_put(L_MOTOR_IN1, 0);
    gpio_put(L_MOTOR_IN2, 1);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 1);
}

// Function to move backward
void reverse_motor(float new_pwm_left, float new_pwm_right)
{
    use_pid_control = false;
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(1000));

    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), new_pwm_left);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), new_pwm_right);

    // Turn on both motors
    gpio_put(L_MOTOR_IN1, 1);
    gpio_put(L_MOTOR_IN2, 0);
    gpio_put(R_MOTOR_IN3, 1);
    gpio_put(R_MOTOR_IN4, 0);

    // Enable the enable pins
    gpio_put(L_MOTOR_ENA, 1);
    gpio_put(R_MOTOR_ENB, 1);
}

// Function to stop
void stop_motor()
{
    use_pid_control = false;
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Turn off all motors
    gpio_put(L_MOTOR_IN1, 0);
    gpio_put(L_MOTOR_IN2, 0);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 0);

    // Disable the enable pins
    gpio_put(L_MOTOR_ENA, 0);
    gpio_put(R_MOTOR_ENB, 0);
}

// Function to turn
void turn_motor(int direction, float angle)
{
    use_pid_control = false;
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    reset_encoders();
    move_motor(PWM_MAX, PWM_MAX);

    // Motor to turn left
    if (direction == LEFT_WHEEL)
    {
        // Reverse left wheel, forward right wheel
        gpio_put(L_MOTOR_IN1, 1);
        gpio_put(L_MOTOR_IN2, 0);
        gpio_put(R_MOTOR_IN3, 0);
        gpio_put(R_MOTOR_IN4, 1);

        // Enable the enable pins
        gpio_put(L_MOTOR_ENA, 1);
        gpio_put(R_MOTOR_ENB, 1);
    }
    // Motor to turn right
    else
    {
        // Reverse right wheel, forward left wheel
        gpio_put(L_MOTOR_IN1, 0);
        gpio_put(L_MOTOR_IN2, 1);
        gpio_put(R_MOTOR_IN3, 1);
        gpio_put(R_MOTOR_IN4, 0);

        // Enable the enable pins
        gpio_put(L_MOTOR_ENA, 1);
        gpio_put(R_MOTOR_ENB, 1);
    }

    if (angle != NO_ANGLE)
    {
        int target_distance = (angle / FULL_CIRCLE) * (PI * WHEEL_TO_WHEEL_DISTANCE);
        while (get_average_distance() < target_distance)
        {
            vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
        }
        stop_motor();
        reset_encoders();
        sleep_ms(50);
    }
}

// PID Computation
float compute_pid_pwm(float target_speed, float current_value, float *integral, float *prev_error)
{
    float error = target_speed - current_value;
    *integral += error;
    float derivative = error - *prev_error;
    float control_signal = Kp * error + Ki * (*integral) + Kd * derivative;
    *prev_error = error;

    // Scale the control signal to the PWM range
    float pwm_value = (control_signal / MAX_SPEED) * PWM_MAX;

    return pwm_value;
}

// PID Task
void pid_task(void *params)
{
    while (1)
    {
        if (use_pid_control)
        {
            // Compute PID control signals
            float pwm_left = compute_pid_pwm(target_speed, get_left_speed(), &integral_left, &prev_error_left);
            float pwm_right = compute_pid_pwm(target_speed, get_right_speed(), &integral_right, &prev_error_right);

            printf("Computed Left PID PWM: %.2f, Right PID PWM: %.2f\n", pwm_left, pwm_right);

            // Move motors with the computed PWM values
            move_motor(pwm_left, pwm_right);
        }

        // Wait for the next control period
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void move_motor_pid(float new_target_speed)
{
    stop_motor();
    target_speed = new_target_speed;
    use_pid_control = true;
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("PID control enabled.\n");
}

void move_motor_constant(float new_pwm_left, float new_pwm_right)
{
    use_pid_control = false;
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    move_motor(new_pwm_left, new_pwm_right);
    printf("PID control disabled. Motors will run at constant speed.\n");
}

// Motor PWM Initialization
void motor_pwm_init()
{
    // Set GPIO pins for ENA and ENB to PWM mode
    gpio_set_function(L_MOTOR_ENA, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_ENB, GPIO_FUNC_PWM);

    // Get PWM slices for left and right motors
    uint slice_left = pwm_gpio_to_slice_num(L_MOTOR_ENA);
    uint slice_right = pwm_gpio_to_slice_num(R_MOTOR_ENB);

    // Configure PWM frequency to 40 kHz (125 MHz / PWM_MAX)
    pwm_set_wrap(slice_left, PWM_MAX);
    pwm_set_wrap(slice_right, PWM_MAX);

    // Set clock divider to 125
    pwm_set_clkdiv(slice_left, 125);
    pwm_set_clkdiv(slice_right, 125);

    // Enable PWM for both motors
    pwm_set_enabled(slice_left, true);
    pwm_set_enabled(slice_right, true);
}

void motor_init()
{
    // Initialize GPIO pins for motors
    gpio_init(L_MOTOR_IN1);
    gpio_init(L_MOTOR_IN2);
    gpio_init(L_MOTOR_ENA);
    gpio_init(R_MOTOR_IN3);
    gpio_init(R_MOTOR_IN4);
    gpio_init(R_MOTOR_ENB);

    // Set pins as outputs
    gpio_set_dir(L_MOTOR_IN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_IN2, GPIO_OUT);
    gpio_set_dir(L_MOTOR_ENA, GPIO_OUT);
    gpio_set_dir(R_MOTOR_IN3, GPIO_OUT);
    gpio_set_dir(R_MOTOR_IN4, GPIO_OUT);
    gpio_set_dir(R_MOTOR_ENB, GPIO_OUT);

    // Initialize PWM for motors
    motor_pwm_init();

    // Start PID control task
    xTaskCreate(pid_task, "PID Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
}