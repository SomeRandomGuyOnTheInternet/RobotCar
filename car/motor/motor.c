#include "motor.h"

static bool use_pid_control = false;
static PIDState pid_state = DISABLED;
static float target_speed = MIN_SPEED;
static float target_turn_angle = CONTINUOUS_TURN;

// Function to move motors forward
void forward_motor(float new_pwm_left, float new_pwm_right)
{
    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), (uint16_t)new_pwm_left);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), (uint16_t)new_pwm_right);

    // Set motor directions
    gpio_put(L_MOTOR_IN1, 0);
    gpio_put(L_MOTOR_IN2, 1);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 1);

    // Enable the enable pins
    gpio_put(L_MOTOR_ENA, 1);
    gpio_put(R_MOTOR_ENB, 1);
}

// Function to move backward
void reverse_motor(float new_pwm_left, float new_pwm_right)
{
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

// Function to turn
void turn_motor(int direction, float new_pwm_left, float new_pwm_right)
{
    pwm_set_chan_level(pwm_gpio_to_slice_num(L_MOTOR_ENA), pwm_gpio_to_channel(L_MOTOR_ENA), (uint16_t)new_pwm_left);
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_MOTOR_ENB), pwm_gpio_to_channel(R_MOTOR_ENB), (uint16_t)new_pwm_right);

    // Motor to turn left
    if (direction == LEFT)
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
}

// Function to stop
void stop_motor()
{
    // Turn off all motors
    gpio_put(L_MOTOR_IN1, 0);
    gpio_put(L_MOTOR_IN2, 0);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 0);

    // Disable the enable pins
    gpio_put(L_MOTOR_ENA, 0);
    gpio_put(R_MOTOR_ENB, 0);
}

void motor_conditioning()
{
    printf("[MOTOR/CONDITIONING] Running motor conditioning.\n");
    stop_motor();
    forward_motor(PWM_JUMPSTART, PWM_JUMPSTART);
    sleep_ms(15000);
    printf("[MOTOR/CONDITIONING] Reversing motor conditioning.\n");
    reverse_motor(PWM_JUMPSTART, PWM_JUMPSTART);
    sleep_ms(15000);
    stop_motor();
    printf("[MOTOR/CONDITIONING] Motor conditioning complete.\n");
}

void disable_pid_control()
{
    use_pid_control = false;
}

void forward_motor_manual(float new_pwm_left, float new_pwm_right)
{
    disable_pid_control();
    forward_motor(new_pwm_left, new_pwm_right);
    // printf("[MOTOR/MANUAL] Moving forward with PWM Left: %f, Right: %f\n", new_pwm_left, new_pwm_right);
}

void reverse_motor_manual(float new_pwm_left, float new_pwm_right)
{
    disable_pid_control();
    reverse_motor(new_pwm_left, new_pwm_right);
    // printf("[MOTOR/MANUAL] Reversing with PWM Left: %f, Right: %f\n", new_pwm_left, new_pwm_right);
}

void turn_motor_manual(int direction, float angle, float new_pwm_left, float new_pwm_right)
{
    disable_pid_control();
    turn_motor(direction, new_pwm_left, new_pwm_right);
    if (angle != CONTINUOUS_TURN)
    {
        reset_encoders();
        while (turn_until_angle(angle))
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay to periodically check distance
        }
    }
    // printf("[MOTOR/MANUAL] Turning %s with PWM Left: %f, Right: %f\n", (direction == LEFT) ? "left" : "right", new_pwm_left, new_pwm_right);
}

void stop_motor_manual()
{
    disable_pid_control();
    stop_motor();
    // printf("[MOTOR/MANUAL] Stopping motor.\n");
}

void offset_move_motor(int direction, int turn, float offset)
{
    if (offset < 0.0f)
    {
        offset = 0.0f;
    }
    else if (offset > 1.0f)
    {
        offset = 1.0f;
    }

    int pwm_left = PWM_MID_LEFT;
    int pwm_right = PWM_MID_RIGHT;
    int pwm_left_offset_range = (PWM_MAX_LEFT - PWM_MIN_LEFT) / 2;
    int pwm_right_offset_range = (PWM_MAX_RIGHT - PWM_MIN_RIGHT) / 2;

    if (turn == LEFT)
    {
        pwm_left -= pwm_left_offset_range * offset;
        pwm_right += pwm_right_offset_range * offset;
    }
    else if (turn == RIGHT)
    {
        pwm_left += pwm_left_offset_range * offset;
        pwm_right -= pwm_right_offset_range * offset;
    }

    // printf("[MOTOR/OFFSET] Offset motor with turn %s, direction %s, offset %f.\n", (turn == LEFT) ? "left" : "right", (direction == FORWARDS) ? "forwards" : "backwards", offset);
    if (direction == FORWARDS)
    {
        forward_motor_manual(pwm_left, pwm_right);
    }
    else if (direction == BACKWARDS)
    {
        reverse_motor_manual(pwm_left, pwm_right);
    }
}

void enable_pid_control()
{
    use_pid_control = true;
}

void forward_motor_pid(float new_target_speed)
{
    enable_pid_control();
    target_speed = new_target_speed;
    pid_state = FORWARD;
    // printf("[PID] PID Forward.\n");
}

void reverse_motor_pid(float new_target_speed)
{
    enable_pid_control();
    target_speed = new_target_speed;
    pid_state = REVERSE;
    // printf("[PID] PID reverse.\n");
}

void turn_motor_pid(int direction, float new_target_speed, float new_target_turn_angle)
{
    enable_pid_control();
    target_speed = new_target_speed;
    target_turn_angle = new_target_turn_angle;
    pid_state = (direction == LEFT) ? LEFT_TURN : RIGHT_TURN;
    // printf("[PID] PID %s turn.\n", (direction == LEFT) ? "left" : "right");
}

void stop_motor_pid()
{
    disable_pid_control();
    stop_motor();
    target_speed = 0.0f;
    pid_state = STOP;
    enable_pid_control();
    // printf("[PID] PID stop.\n");
}

// Must call turn motor and reset encoders before calling this function
bool turn_until_angle(float angle)
{
    if (angle == CONTINUOUS_TURN)
    {
        return true;
    }
    if (angle < 0.0f || angle > FULL_CIRCLE)
    {
        return false;
    }

    float target_distance = (angle / FULL_CIRCLE) * (PI * WHEEL_TO_WHEEL_DISTANCE);
    if (target_distance - get_average_distance() <= 0.05)
    {
        if (use_pid_control)
        {
            stop_motor_pid();
        }
        else
        {
            stop_motor_manual();
        }
        return false;
    }

    return true;
}

// PID Computation
float compute_pid_pwm(float target_speed, float current_value, float *integral, float *prev_error)
{
    float error = target_speed - current_value;
    *integral += error;
    float derivative = error - *prev_error;
    float control_signal = Kp * error + Ki * (*integral) + Kd * derivative;
    *prev_error = error;

    return control_signal;
}

// PID Task
void pid_task(void *params)
{
    // PID control variables
    float integral_left = 0.0;
    float integral_right = 0.0;
    float prev_error_left = 0.0;
    float prev_error_right = 0.0;

    float pid_pwm_left = PWM_MIN_LEFT;
    float pid_pwm_right = PWM_MIN_RIGHT;

    bool jumpstarted = false;

    while (1)
    {
        // printf("[PID] PID Task Running.\n");
        if (use_pid_control)
        {
            if (target_speed < MIN_SPEED)
            {
                pid_state = STOP;
            }
            if (target_speed > MAX_SPEED)
            {
                target_speed = MAX_SPEED;
            }

            float left_speed = get_left_speed();
            float right_speed = get_right_speed();
            // printf("[PID/VALIDATED] Target Speed: %.2f, Left Speed: %.2f, Right Speed: %.2f\n", target_speed, left_speed, right_speed);

            pid_pwm_left += compute_pid_pwm(target_speed, left_speed, &integral_left, &prev_error_left);
            pid_pwm_right += compute_pid_pwm(target_speed, right_speed, &integral_right, &prev_error_right);

            if (left_speed < JUMPSTART_SPEED_THRESHOLD || right_speed < JUMPSTART_SPEED_THRESHOLD)
            {
                // printf("[PID] Jumpstarting motors.\n");
                pid_pwm_left = PWM_JUMPSTART;
                pid_pwm_right = PWM_JUMPSTART;
                jumpstarted = true;
            }
            else
            {
                // printf("[PID] Normal motor operation.\n");
                if (pid_pwm_left < PWM_MIN_LEFT || jumpstarted)
                {
                    pid_pwm_left = PWM_MIN_LEFT;
                }
                else if (pid_pwm_left > PWM_MAX)
                {
                    pid_pwm_left = PWM_MAX;
                }
                else if (pid_pwm_right < PWM_MIN_RIGHT || jumpstarted)
                {
                    pid_pwm_right = PWM_MIN_RIGHT;
                }
                else if (pid_pwm_right > PWM_MAX)
                {
                    pid_pwm_right = PWM_MAX;
                }

                jumpstarted = false;
            }

            // printf("[PID/VALIDATED] Computed Left PID PWM: %.2f, Right PID PWM: %.2f\n", pid_pwm_left, pid_pwm_right);

            switch (pid_state)
            {
            case FORWARD:
                forward_motor(pid_pwm_left, pid_pwm_right);
                break;
            case REVERSE:
                reverse_motor(pid_pwm_left, pid_pwm_right);
                break;
            case LEFT_TURN:
                turn_motor(LEFT, pid_pwm_left, pid_pwm_right);
                if (target_turn_angle != CONTINUOUS_TURN)
                    reset_encoders();
                pid_state = TURNING;
                break;
            case RIGHT_TURN:
                turn_motor(RIGHT, pid_pwm_left, pid_pwm_right);
                if (target_turn_angle != CONTINUOUS_TURN)
                    reset_encoders();
                pid_state = TURNING;
                break;
            case TURNING:
                turn_until_angle(target_turn_angle);
                break;
            case STOP:
                stop_motor();
                break;
            case DISABLED:
            default:
                disable_pid_control();
                break;
            }

            // Wait for the next control period
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
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

    // Configure PWM frequency (125 MHz / PWM_MAX)
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
