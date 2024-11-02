#include "encoder.h"

uint32_t pulse_count_left = 0;
uint32_t pulse_count_right = 0;
volatile uint32_t oscillation = 0;
double moved_distance = 0.0;
volatile float actual_speed_left;
volatile float actual_speed_right;

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

bool encoder_1s_callback()
{
    get_speed_and_distance(LEFT_WHEEL, pulse_count_left);
    get_speed_and_distance(RIGHT_WHEEL, pulse_count_right);

    pulse_count_left = 0;
    pulse_count_right = 0;

    return true;
}

void encoder_pulse_callback(uint gpio, uint32_t events)
{
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, read_encoder_pulse);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, read_encoder_pulse);
}

void start_tracking()
{
    moved_distance = 0;
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
}

void get_speed_and_distance(int encoder, uint32_t pulse_count)
{
    double distance_per_hole = ENCODER_CIRCUMFERENCE / ENCODER_NOTCH;
    double distance = distance_per_hole * pulse_count;
    double speed = distance;

    moved_distance += distance;

    if (encoder == 0)
    {
        actual_speed_left = speed;
    }
    else if (encoder == 1)
    {
        actual_speed_right = speed;
    }

    if (pulse_count > 0) {
        printf("=====\n");
        printf("Total distance: %.2lf cm\n", moved_distance);
        printf("Current speed: %.2lf cm/s\n", speed);
        printf("=====\n");
    }

    return;
}

// int main() {
//     stdio_init_all();

//     // Initialise motor GPIO pins and PWM
//     init_motor_setup();
//     init_motor_pwm();

//     // Initialise encoder GPIO pins
//     encoder_init();
//     printf("Encoder pins initialised\n");
//     sleep_ms(500);

//     // Set up a timer to generate interrupts every second
//     struct repeating_timer timer;
//     add_repeating_timer_ms(1000, encoder_1s_callback, NULL, &timer);

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
