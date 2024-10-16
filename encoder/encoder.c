// Control L and R encoder

#include "encoder.h"

// Global variable declaration
volatile bool complete_movement = false;
uint32_t target_grid_number = 0;
uint32_t pulse_count_left = 0;
uint32_t pulse_count_right = 0;
volatile uint32_t oscillation = 0;
double moved_distance = 0.0;
volatile float actual_speed_left;
volatile float actual_speed_right;

// Function to get motor speed and distance
void get_speed_and_distance(int encoder, uint32_t pulse_count)
{
    // Calculate motor speed in cm/s
    double distance_per_hole = ENCODER_CIRCUMFERENCE / ENCODER_NOTCH;
    double distance = distance_per_hole * pulse_count;
    double speed = distance / 0.075;

    // Calculate and accumulate the distance
    moved_distance += distance;

    // Print motor speed and total distance
    if (encoder == 0)
    {
        actual_speed_left = speed;
    }
    else if (encoder == 1)
    {
        actual_speed_right = speed;
    }
    return;
}

void start_tracking(int target_grids)
{
    moved_distance = 0;              // Reset the distance moved
    target_grid_number = target_grids; // set the target number of grids
    complete_movement = false;
    return;
}

uint32_t get_grids_moved(bool reset)
{
    encoder_callback(); // Calculate final moved_distancce

    uint32_t grids_moved = moved_distance / 14;
    printf("DISTANCE TRAVELLED: %.2lf\n", moved_distance);

    if (reset)
    {
        // Reset the distance moved
        moved_distance = 0.0;
    }

    return grids_moved;
}

// Function to count each encoder's pulse
void encoder_pulse(uint gpio)
{
    // L encoder interrupted
    if (gpio == L_ENCODER_OUT)
    {
        pulse_count_left++;
    }
    // R encoder interrupted
    else if (gpio == R_ENCODER_OUT)
    {
        pulse_count_right++;
    }

    oscillation++;
}

// Function to interrupt every second
bool encoder_callback()
{
    // Call getSpeedAndDistance function every second
    get_speed_and_distance(0, pulse_count_left);
    get_speed_and_distance(1, pulse_count_right);

    // Reset the pulse counts
    pulse_count_left = 0;
    pulse_count_right = 0;

    if (target_grid_number > 0)
    {
        uint32_t grids_moved = moved_distance / 10.5;
        if (grids_moved >= target_grid_number)
        {
            target_grid_number = 0;    // Reset target number of grids
            moved_distance = 0;       // Reset moved distance
            complete_movement = true; // Set flag to indicate target number of grids reached
        }
    }
    return true;
}

// Function to initialize pins for encoders
void init_encoder_setup()
{
    // Initialize GPIO pins for L encoder
    gpio_init(L_ENCODER_POW);
    gpio_init(L_ENCODER_OUT);

    // Set GPIO pins as outputs for L encoder
    gpio_set_dir(L_ENCODER_POW, GPIO_OUT);
    gpio_set_dir(L_ENCODER_OUT, GPIO_IN);

    // Set GPIO settings for L encoder
    gpio_pull_up(L_ENCODER_OUT);
    // gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &encoderPulse);

    // Initialize GPIO pins for R encoder
    gpio_init(R_ENCODER_POW);
    gpio_init(R_ENCODER_OUT);

    // Set GPIO pins as outputs for R encoder
    gpio_set_dir(R_ENCODER_POW, GPIO_OUT);
    gpio_set_dir(R_ENCODER_OUT, GPIO_IN);

    // Set GPIO settings for R encoder
    gpio_pull_up(R_ENCODER_OUT);

    // Enable the POW pins
    gpio_put(L_ENCODER_POW, 1);
    gpio_put(R_ENCODER_POW, 1);
}

/*
int main() {
    stdio_init_all();

    // Initialise motor GPIO pins and PWM
    initMotorSetup();
    initMotorPWM();

    // Initialise encoder GPIO pins
    initEncoderSetup();

    // Set up a timer to generate interrupts every second
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, encoderCallback, NULL, &timer);

    while (1) {
        // Run at half duty cycle
        moveMotor(1563);
        sleep_ms(5000);

        // Turn left at full duty cycle
        moveMotor(3165);
        turnMotor(1);
        sleep_ms(250);

        // Turn right at full duty cycle
        moveMotor(3165);
        turnMotor(0);
        sleep_ms(250);

        // Run at 32% duty cycle
        // moveMotor(1000);
        moveMotor(3165);
        sleep_ms(5000);
    }

    return 0;
}
*/