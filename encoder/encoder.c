#include "encoder.h"

const double DISTANCE_PER_HOLE = ENCODER_CIRCUMFERENCE / ENCODER_NOTCH;

uint32_t pulse_count_left = 0;
uint32_t pulse_count_right = 0;
volatile uint32_t oscillation = 0;
volatile double total_average_distance = 0.0;
double total_average_speed = 0.0;
float actual_distance_left = 0.0;
float actual_distance_right = 0.0;
volatile float actual_speed_left = 0.0;
volatile float actual_speed_right = 0.0;

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
}

bool encoder_set_distance_speed_callback(struct repeating_timer *t)
{
    int *interval = (int *)t->user_data;

    set_distance_speed(LEFT_WHEEL, pulse_count_left, *interval);
    set_distance_speed(RIGHT_WHEEL, pulse_count_right, *interval);

    pulse_count_left = 0;
    pulse_count_right = 0;

    return true;
}

void read_encoder_pulse(uint gpio, uint32_t events)
{
    if (gpio == L_ENCODER_OUT)
    {
        pulse_count_left++;
    }

    else if (gpio == R_ENCODER_OUT)
    {
        pulse_count_right++;
    }

    oscillation++;

    // Call the callback function if it is not NULL
    // if (callback != NULL)
    // {
    //     callback(gpio, events);
    // }
}

void start_tracking()
{
    total_average_distance = 0;
    return;
}

void set_distance_speed(int encoder, uint32_t pulse_count, int interval_ms)
{
    double distance = DISTANCE_PER_HOLE * pulse_count;
    double speed = distance / (interval_ms / 1000.0);

    if (encoder == LEFT_WHEEL)
    {
        if (pulse_count > 0)
        {
            actual_distance_left += distance;
            actual_speed_left = speed;
            printf("=====\n");
            printf("Total distance (Left): %.2lf cm\n", actual_distance_left);
            printf("Current speed (Left): %.2lf cm/s\n", actual_speed_left);

        }
    }
    else if (encoder == RIGHT_WHEEL)
    {
        actual_speed_right = speed;
        if (pulse_count > 0)
        {
            actual_distance_right += distance;
            actual_speed_right = speed;
            printf("=====\n");
            printf("Total distance (Right): %.2lf cm\n", actual_distance_right);
            printf("Current speed (Right): %.2lf cm/s\n", actual_speed_right);
        }
    }

    total_average_distance = (actual_distance_left + actual_distance_right) / 2;
    total_average_speed = (actual_distance_left + actual_distance_right) / 2;

    // printf("=====\n");
    // printf("Total distance (total): %.2lf cm\n", total_average_distance);
    // printf("Current speed (total): %.2lf cm/s\n", total_average_speed);

    return;
}


// int main() {
//     stdio_init_all();

//     // Initialise motor GPIO pins and PWM
//     motor_init_setup();
//     motor_pwm_init();

//     // Initialise encoder GPIO pins
//     encoder_init();
//     printf("Encoder pins initialised\n");
//     sleep_ms(500);

//     // Set up a timer to generate interrupts every second
//     struct repeating_timer timer;
//     add_repeating_timer_ms(1000, encoder_set_distance_speed_callback, NULL, &timer);

//     while (1) {
//         // Run at half duty cycle
//         move_motor(1563, 1563);
//         sleep_ms(5000);

//         // Turn left at full duty cycle
//         move_motor(3165, 3165);
//         turn_motor(1);
//         sleep_ms(250);

//         // Turn right at full duty cycle
//         move_motor(3165, 3165);
//         turn_motor(0);
//         sleep_ms(250);

//         // Run at 32% duty cycle
//         // move_motor(1000);
//         move_motor(1000, 1000);
//         sleep_ms(5000);
//     }

//     return 0;
// }
